using System.Diagnostics;
using System.Text;
using Gurobi;

namespace MCFStatic
{
    internal sealed record ShortestPath(
        IReadOnlyList<string> Nodes,
        IReadOnlyList<Arc> Arcs,
        double TotalCost);

    internal sealed record FeasibleSubNetwork(
        Network Network,
        IReadOnlyDictionary<string, double> ForwardDelayMs,
        IReadOnlyDictionary<string, double> BackwardDelayMs);

    internal sealed record ArcFlowSolution(
        double Objective,
        IReadOnlyDictionary<string, IReadOnlyList<Arc>> RequestPaths,
        IReadOnlyList<Arc> UsedArcs);

    // Result of the integer master: the objective plus, per request id, the
    // list of path ids selected (x > 0.5) in the integer-feasible solution.
    internal sealed record IntegerMasterSolution(
        double Objective,
        Dictionary<string, List<string>> SelectedPaths);

    // A fractional arc-activation (y[a]) variable chosen as a branching candidate
    // at the end of column generation: the arc id, its LP value, how far that value
    // is from 0.5, and the arc cost used to break ties (higher cost preferred).
    internal sealed record BranchingArc(
        string ArcId,
        double Value,
        double DistanceToHalf,
        double Cost);

    // One node of the branch-and-price tree: the arc branched on, the direction
    // (1 = y[a] forced open, 0 = y[a] forced closed), the LP objective of the
    // restricted master after the bound change, and whether that LP was feasible.
    // DFS picks the child with the smaller LpBound first.
    internal sealed record BranchNode(
        string ArcId,
        int Direction,
        double LpBound,
        bool Feasible);

    // Outcome of column generation at one branch-and-price child: the best globally
    // feasible incumbent seen there, the node's final Lagrangian dual bound, the next
    // fractional arc to branch on, and whether the node was pruned before CG ran.
    internal sealed record BranchChildResult(
        IntegerMasterSolution? Best,
        double DualBound,
        BranchingArc? Candidate,
        bool Pruned);

    internal static class Optimization
    {
        public static FeasibleSubNetwork? FindFeasibleSubNetwork(Network network, Request request)
        {
            ArgumentNullException.ThrowIfNull(request);
            return FindFeasibleSubNetwork(network, request.Source, request.Target, request.MaxPingMs);
        }

        public static FeasibleSubNetwork? FindFeasibleSubNetwork(
            Network network,
            string origin,
            string destination,
            double maxPingMs)
        {
            ArgumentNullException.ThrowIfNull(network);
            if (string.IsNullOrWhiteSpace(origin))
                throw new ArgumentException("Origin must be provided.", nameof(origin));
            if (string.IsNullOrWhiteSpace(destination))
                throw new ArgumentException("Destination must be provided.", nameof(destination));
            if (!double.IsNaN(maxPingMs) && maxPingMs < 0)
                throw new ArgumentOutOfRangeException(nameof(maxPingMs), "max ping must be non-negative.");
            if (!network.Nodes.Contains(origin))
                throw new ArgumentException($"Origin node '{origin}' not found in network.", nameof(origin));
            if (!network.Nodes.Contains(destination))
                throw new ArgumentException($"Destination node '{destination}' not found in network.", nameof(destination));

            // Fall back to plain BFS when ping data is unavailable: either the request has
            // no max_ping_ms or the network has no delay_ms on its arcs.
            var delaysAvailable = network.ArcCount > 0 && !double.IsNaN(network.Arcs[0].DelayMs);
            if (double.IsNaN(maxPingMs) || !delaysAvailable)
                return FindReachableSubNetwork(network, origin, destination);

            // 1) Backward Dijkstra from destination on the reversed graph.
            //    For each reachable node u, backward[u] = min ping from u to destination.
            //    Anything that would exceed maxPingMs is pruned.
            var backward = new Dictionary<string, double>(StringComparer.Ordinal)
            {
                [destination] = 0d,
            };
            var backwardSettled = new HashSet<string>(StringComparer.Ordinal);
            var backwardQueue = new PriorityQueue<string, double>();
            backwardQueue.Enqueue(destination, 0d);

            while (backwardQueue.TryDequeue(out var node, out var nodeDelay))
            {
                if (!backwardSettled.Add(node))
                    continue;

                foreach (var arc in network.IncomingTo(node))
                {
                    if (arc.DelayMs < 0)
                        throw new InvalidOperationException(
                            $"Dijkstra requires non-negative delays; arc '{arc.Source}' -> '{arc.Target}' has delay {arc.DelayMs}.");

                    var predecessor = arc.Source;
                    var alt = nodeDelay + arc.DelayMs;
                    if (alt > maxPingMs)
                        continue;

                    if (!backward.TryGetValue(predecessor, out var existing) || alt < existing)
                    {
                        backward[predecessor] = alt;
                        backwardQueue.Enqueue(predecessor, alt);
                    }
                }
            }

            // If the origin was never reached in the backward pass, no path can satisfy max ping.
            if (!backward.ContainsKey(origin))
                return null;

            // 2) Forward Dijkstra from origin, restricted to the backward-reachable set.
            //    Drop any node v where forward[v] + backward[v] > maxPingMs.
            var forward = new Dictionary<string, double>(StringComparer.Ordinal)
            {
                [origin] = 0d,
            };
            var forwardSettled = new HashSet<string>(StringComparer.Ordinal);
            var forwardQueue = new PriorityQueue<string, double>();
            forwardQueue.Enqueue(origin, 0d);

            while (forwardQueue.TryDequeue(out var node, out var nodeDelay))
            {
                if (!forwardSettled.Add(node))
                    continue;

                foreach (var arc in network.OutgoingFrom(node))
                {
                    var successor = arc.Target;
                    if (!backward.TryGetValue(successor, out var bSucc))
                        continue; // cannot reach destination within maxPing from successor

                    var alt = nodeDelay + arc.DelayMs;
                    if (alt + bSucc > maxPingMs)
                        continue;

                    if (!forward.TryGetValue(successor, out var existing) || alt < existing)
                    {
                        forward[successor] = alt;
                        forwardQueue.Enqueue(successor, alt);
                    }
                }
            }

            // Build the sub-network: keep arcs whose endpoints are both labeled and
            // whose end-to-end ping bound stays within maxPingMs.
            var keptArcs = new List<Arc>();
            foreach (var arc in network.Arcs)
            {
                if (!forward.TryGetValue(arc.Source, out var fSrc))
                    continue;
                if (!forward.ContainsKey(arc.Target))
                    continue;
                if (!backward.TryGetValue(arc.Target, out var bTgt))
                    continue;

                if (fSrc + arc.DelayMs + bTgt <= maxPingMs)
                    keptArcs.Add(arc);
            }

            return new FeasibleSubNetwork(new Network(keptArcs), forward, backward);
        }

        // BFS fallback used when ping data is missing. Finds nodes reachable
        // from origin that can also reach destination; arcs between them form
        // the sub-network. Labels are returned as NaN to indicate that delays
        // were not computed.
        private static FeasibleSubNetwork? FindReachableSubNetwork(
            Network network,
            string origin,
            string destination)
        {
            // Backward BFS from destination using IncomingTo arcs.
            var backwardReachable = new HashSet<string>(StringComparer.Ordinal) { destination };
            var backwardQueue = new Queue<string>();
            backwardQueue.Enqueue(destination);
            while (backwardQueue.Count > 0)
            {
                var node = backwardQueue.Dequeue();
                foreach (var arc in network.IncomingTo(node))
                {
                    if (backwardReachable.Add(arc.Source))
                        backwardQueue.Enqueue(arc.Source);
                }
            }

            if (!backwardReachable.Contains(origin))
                return null;

            // Forward BFS from origin, restricted to nodes that can reach destination.
            var forwardReachable = new HashSet<string>(StringComparer.Ordinal) { origin };
            var forwardQueue = new Queue<string>();
            forwardQueue.Enqueue(origin);
            while (forwardQueue.Count > 0)
            {
                var node = forwardQueue.Dequeue();
                foreach (var arc in network.OutgoingFrom(node))
                {
                    if (!backwardReachable.Contains(arc.Target))
                        continue;
                    if (forwardReachable.Add(arc.Target))
                        forwardQueue.Enqueue(arc.Target);
                }
            }

            var keptArcs = new List<Arc>();
            foreach (var arc in network.Arcs)
            {
                if (forwardReachable.Contains(arc.Source) && forwardReachable.Contains(arc.Target))
                    keptArcs.Add(arc);
            }

            var forwardLabels = new Dictionary<string, double>(StringComparer.Ordinal);
            foreach (var node in forwardReachable)
                forwardLabels[node] = double.NaN;

            var backwardLabels = new Dictionary<string, double>(StringComparer.Ordinal);
            foreach (var node in backwardReachable)
                backwardLabels[node] = double.NaN;

            return new FeasibleSubNetwork(new Network(keptArcs), forwardLabels, backwardLabels);
        }

        public static ShortestPath? FindShortestPath(
            Network network,
            string source,
            string target,
            Func<Arc, double>? weightSelector = null,
            double maxCost = double.PositiveInfinity,
            double tolerance = 0d)
        {
            ArgumentNullException.ThrowIfNull(network);
            if (string.IsNullOrWhiteSpace(source))
                throw new ArgumentException("Source must be provided.", nameof(source));
            if (string.IsNullOrWhiteSpace(target))
                throw new ArgumentException("Target must be provided.", nameof(target));
            if (!network.Nodes.Contains(source))
                throw new ArgumentException($"Source node '{source}' not found in network.", nameof(source));
            if (!network.Nodes.Contains(target))
                throw new ArgumentException($"Target node '{target}' not found in network.", nameof(target));

            weightSelector ??= static arc => arc.Weight;

            if (string.Equals(source, target, StringComparison.Ordinal))
                return new ShortestPath(new[] { source }, Array.Empty<Arc>(), 0d);

            var distances = new Dictionary<string, double>(StringComparer.Ordinal);
            var predecessor = new Dictionary<string, Arc>(StringComparer.Ordinal);
            var settled = new HashSet<string>(StringComparer.Ordinal);
            var queue = new PriorityQueue<string, double>();

            foreach (var node in network.Nodes)
                distances[node] = double.PositiveInfinity;

            distances[source] = 0d;
            queue.Enqueue(source, 0d);

            while (queue.TryDequeue(out var current, out var currentDistance))
            {
                if (!settled.Add(current))
                    continue;

                // Stop once the smallest label exceeds maxCost (plus tolerance):
                // every remaining label is at least this large, so no path can stay
                // within budget. The tolerance lets a path that overshoots maxCost by
                // a tiny amount still be returned (absorbs floating-point noise).
                if (currentDistance > maxCost + tolerance)
                    return null;

                if (string.Equals(current, target, StringComparison.Ordinal))
                    break;

                foreach (var arc in network.OutgoingFrom(current))
                {
                    var weight = weightSelector(arc);
                    if (weight < 0)
                        throw new InvalidOperationException(
                            $"Dijkstra requires non-negative weights; arc '{arc.Source}' -> '{arc.Target}' has weight {weight}.");

                    if (settled.Contains(arc.Target))
                        continue;

                    var alt = currentDistance + weight;
                    if (alt < distances[arc.Target])
                    {
                        distances[arc.Target] = alt;
                        predecessor[arc.Target] = arc;
                        queue.Enqueue(arc.Target, alt);
                    }
                }
            }

            if (double.IsPositiveInfinity(distances[target]))
                return null;

            var pathArcs = new List<Arc>();
            var pathNodes = new List<string> { target };
            var cursor = target;
            while (!string.Equals(cursor, source, StringComparison.Ordinal))
            {
                var arc = predecessor[cursor];
                pathArcs.Add(arc);
                cursor = arc.Source;
                pathNodes.Add(cursor);
            }
            pathArcs.Reverse();
            pathNodes.Reverse();

            return new ShortestPath(pathNodes, pathArcs, distances[target]);
        }

        // Finds up to k loopless shortest paths from source to target using Yen's
        // algorithm on top of a (banned-node / banned-arc) Dijkstra. The same
        // weightSelector / maxCost / tolerance semantics as FindShortestPath apply:
        // a path is only returned if its total weight stays within maxCost+tolerance,
        // and the search stops as soon as every remaining candidate exceeds it.
        //
        // The returned list is ordered best-first (ascending total cost) and contains
        // between 0 and k paths.
        public static IReadOnlyList<ShortestPath> FindKShortestPaths(
            Network network,
            string source,
            string target,
            int k,
            Func<Arc, double>? weightSelector = null,
            double maxCost = double.PositiveInfinity,
            double tolerance = 0d,
            IReadOnlySet<string>? forbiddenArcIds = null)
        {
            ArgumentNullException.ThrowIfNull(network);
            if (string.IsNullOrWhiteSpace(source))
                throw new ArgumentException("Source must be provided.", nameof(source));
            if (string.IsNullOrWhiteSpace(target))
                throw new ArgumentException("Target must be provided.", nameof(target));
            if (!network.Nodes.Contains(source))
                throw new ArgumentException($"Source node '{source}' not found in network.", nameof(source));
            if (!network.Nodes.Contains(target))
                throw new ArgumentException($"Target node '{target}' not found in network.", nameof(target));
            if (k <= 0)
                throw new ArgumentOutOfRangeException(nameof(k), "k must be positive.");

            weightSelector ??= static arc => arc.Weight;

            var accepted = new List<ShortestPath>();

            // First shortest path is unconstrained.
            var first = DijkstraConstrained(network, source, target, weightSelector, null, forbiddenArcIds);
            if (first is null || first.TotalCost > maxCost + tolerance)
                return accepted;
            accepted.Add(first);
            if (k == 1)
                return accepted;

            // Candidate spur paths, keyed by total cost, plus a set of arc-id
            // signatures already produced so duplicates are never re-queued.
            var candidates = new PriorityQueue<ShortestPath, double>();
            var seen = new HashSet<string>(StringComparer.Ordinal) { Signature(first.Arcs) };

            while (accepted.Count < k)
            {
                var prev = accepted[^1];

                // Spur from every node of the most recently accepted path except the
                // target (the last node).
                for (var i = 0; i < prev.Nodes.Count - 1; i++)
                {
                    var spurNode = prev.Nodes[i];

                    // Root path = prev[0..i] (its first i arcs).
                    var rootArcs = new List<Arc>(i);
                    for (var j = 0; j < i; j++)
                        rootArcs.Add(prev.Arcs[j]);

                    // Ban the arc that leaves the spur node on every accepted path that
                    // shares this root, so the spur is forced onto a different edge.
                    var bannedArcIds = new HashSet<string>(StringComparer.Ordinal);
                    if (forbiddenArcIds is not null)
                        bannedArcIds.UnionWith(forbiddenArcIds);
                    foreach (var path in accepted)
                    {
                        if (path.Arcs.Count <= i)
                            continue;
                        var sharesRoot = true;
                        for (var j = 0; j < i; j++)
                        {
                            if (!string.Equals(path.Arcs[j].Id, rootArcs[j].Id, StringComparison.Ordinal))
                            {
                                sharesRoot = false;
                                break;
                            }
                        }
                        if (sharesRoot)
                            bannedArcIds.Add(path.Arcs[i].Id);
                    }

                    // Ban every root node except the spur node so the spur path cannot
                    // loop back through the root.
                    var bannedNodes = new HashSet<string>(StringComparer.Ordinal);
                    for (var j = 0; j < i; j++)
                        bannedNodes.Add(prev.Nodes[j]);

                    var spur = DijkstraConstrained(
                        network, spurNode, target, weightSelector, bannedNodes, bannedArcIds);
                    if (spur is null)
                        continue;

                    // Stitch root + spur into a full source -> target path.
                    var totalArcs = new List<Arc>(rootArcs);
                    totalArcs.AddRange(spur.Arcs);

                    var sig = Signature(totalArcs);
                    if (!seen.Add(sig))
                        continue;

                    var totalNodes = new List<string>(i + spur.Nodes.Count);
                    for (var j = 0; j <= i; j++)
                        totalNodes.Add(prev.Nodes[j]);
                    for (var j = 1; j < spur.Nodes.Count; j++)
                        totalNodes.Add(spur.Nodes[j]);

                    var rootCost = 0d;
                    foreach (var arc in rootArcs)
                        rootCost += weightSelector(arc);

                    var totalCost = rootCost + spur.TotalCost;
                    candidates.Enqueue(new ShortestPath(totalNodes, totalArcs, totalCost), totalCost);
                }

                if (candidates.Count == 0)
                    break;

                var next = candidates.Dequeue();
                // Every remaining candidate costs at least this much, so once the best
                // one exceeds the budget no further path can stay within it.
                if (next.TotalCost > maxCost + tolerance)
                    break;
                accepted.Add(next);
            }

            return accepted;
        }

        // Dijkstra variant for Yen's algorithm: identical to FindShortestPath but
        // skips any node in bannedNodes and any arc whose id is in bannedArcIds.
        // Returns null when no path exists under those restrictions.
        private static ShortestPath? DijkstraConstrained(
            Network network,
            string source,
            string target,
            Func<Arc, double> weightSelector,
            HashSet<string>? bannedNodes,
            IReadOnlySet<string>? bannedArcIds)
        {
            if (string.Equals(source, target, StringComparison.Ordinal))
                return new ShortestPath(new[] { source }, Array.Empty<Arc>(), 0d);

            var distances = new Dictionary<string, double>(StringComparer.Ordinal);
            var predecessor = new Dictionary<string, Arc>(StringComparer.Ordinal);
            var settled = new HashSet<string>(StringComparer.Ordinal);
            var queue = new PriorityQueue<string, double>();

            foreach (var node in network.Nodes)
            {
                if (bannedNodes is not null && bannedNodes.Contains(node))
                    continue;
                distances[node] = double.PositiveInfinity;
            }

            distances[source] = 0d;
            queue.Enqueue(source, 0d);

            while (queue.TryDequeue(out var current, out var currentDistance))
            {
                if (!settled.Add(current))
                    continue;

                if (string.Equals(current, target, StringComparison.Ordinal))
                    break;

                foreach (var arc in network.OutgoingFrom(current))
                {
                    if (bannedArcIds is not null && bannedArcIds.Contains(arc.Id))
                        continue;
                    if (bannedNodes is not null && bannedNodes.Contains(arc.Target))
                        continue;
                    if (settled.Contains(arc.Target))
                        continue;

                    var weight = weightSelector(arc);
                    weight = Math.Max(0d, weight); // treat negative weights as zero to avoid exceptions
                    if (weight < 0)
                        throw new InvalidOperationException(
                            $"Dijkstra requires non-negative weights; arc '{arc.Source}' -> '{arc.Target}' has weight {weight}.");

                    if (!distances.TryGetValue(arc.Target, out var existing))
                        existing = double.PositiveInfinity;

                    var alt = currentDistance + weight;
                    if (alt < existing)
                    {
                        distances[arc.Target] = alt;
                        predecessor[arc.Target] = arc;
                        queue.Enqueue(arc.Target, alt);
                    }
                }
            }

            if (!distances.TryGetValue(target, out var targetDistance) ||
                double.IsPositiveInfinity(targetDistance))
                return null;

            var pathArcs = new List<Arc>();
            var pathNodes = new List<string> { target };
            var cursor = target;
            while (!string.Equals(cursor, source, StringComparison.Ordinal))
            {
                var arc = predecessor[cursor];
                pathArcs.Add(arc);
                cursor = arc.Source;
                pathNodes.Add(cursor);
            }
            pathArcs.Reverse();
            pathNodes.Reverse();

            return new ShortestPath(pathNodes, pathArcs, targetDistance);
        }

        // Alternative pricing: one exact Dijkstra shortest path plus up to k-1
        // randomly explored source->target paths whose total (reduced) weight stays
        // within maxCost+tolerance (i.e. negative-reduced-cost columns). The walks
        // are guided by backward shortest-distance labels so every step can still
        // reach the target within budget. The shortest path is always returned first
        // (index 0); the random walks carry no individual optimality guarantee.
        // random defaults to the thread-safe Random.Shared.
        public static IReadOnlyList<ShortestPath> FindRandomNegativeReducedCostPaths(
            Network network,
            string source,
            string target,
            int k,
            Func<Arc, double>? weightSelector = null,
            double maxCost = double.PositiveInfinity,
            double tolerance = 0d,
            Random? random = null,
            IReadOnlySet<string>? forbiddenArcIds = null)
        {
            ArgumentNullException.ThrowIfNull(network);
            if (!network.Nodes.Contains(source))
                throw new ArgumentException($"Source node '{source}' not found in network.", nameof(source));
            if (!network.Nodes.Contains(target))
                throw new ArgumentException($"Target node '{target}' not found in network.", nameof(target));
            if (k <= 0)
                throw new ArgumentOutOfRangeException(nameof(k), "k must be positive.");

            weightSelector ??= static arc => arc.Weight;
            random ??= Random.Shared;

            var distToTarget = BackwardDistances(network, target, weightSelector, forbiddenArcIds);

            var accepted = new List<ShortestPath>();
            var seen = new HashSet<string>(StringComparer.Ordinal);
            var budget = maxCost + tolerance;

            // 1) Exact shortest path via greedy descent on the labels.
            if (distToTarget.TryGetValue(source, out var best) && best <= budget)
            {
                var shortest = GreedyLabelPath(network, source, target, weightSelector, distToTarget, forbiddenArcIds);
                if (shortest is not null && seen.Add(Signature(shortest.Arcs)))
                    accepted.Add(shortest);
            }

            if (accepted.Count == 0)
                return accepted; // target unreachable within budget

            if (k == 1)
                return accepted;

            // 2) Up to k-1 label-guided random walks, each kept within budget.
            var attempts = 0;
            var maxAttempts = 20 * k;
            while (accepted.Count < k && attempts++ < maxAttempts)
            {
                var path = RandomLabelWalk(
                    network, source, target, weightSelector, distToTarget, budget, random, forbiddenArcIds);
                if (path is not null && seen.Add(Signature(path.Arcs)))
                    accepted.Add(path);
            }

            return accepted;
        }

        // Backward shortest-distance labels: distToTarget[v] = min weight v -> target,
        // computed with a backward Dijkstra over the same (clamped non-negative)
        // weightSelector used by the forward walks.
        private static Dictionary<string, double> BackwardDistances(
            Network network, string target, Func<Arc, double> weightSelector, IReadOnlySet<string>? forbiddenArcIds)
        {
            var dist = new Dictionary<string, double>(StringComparer.Ordinal) { [target] = 0d };
            var settled = new HashSet<string>(StringComparer.Ordinal);
            var queue = new PriorityQueue<string, double>();
            queue.Enqueue(target, 0d);

            while (queue.TryDequeue(out var node, out var d))
            {
                if (!settled.Add(node))
                    continue;
                foreach (var arc in network.IncomingTo(node))
                {
                    if (forbiddenArcIds is not null && forbiddenArcIds.Contains(arc.Id))
                        continue;
                    var alt = d + Math.Max(0d, weightSelector(arc));
                    if (!dist.TryGetValue(arc.Source, out var existing) || alt < existing)
                    {
                        dist[arc.Source] = alt;
                        queue.Enqueue(arc.Source, alt);
                    }
                }
            }
            return dist;
        }

        // Exact shortest path: at each node follow the arc minimizing
        // weight + distToTarget[successor]. Visited nodes are skipped so the path
        // stays simple. Returns null if the target cannot be reached.
        private static ShortestPath? GreedyLabelPath(
            Network network, string source, string target,
            Func<Arc, double> weightSelector, Dictionary<string, double> distToTarget,
            IReadOnlySet<string>? forbiddenArcIds)
        {
            var arcs = new List<Arc>();
            var nodes = new List<string> { source };
            var visited = new HashSet<string>(StringComparer.Ordinal) { source };
            var current = source;
            var total = 0d;

            while (!string.Equals(current, target, StringComparison.Ordinal))
            {
                Arc bestArc = default;
                var found = false;
                var bestVal = double.PositiveInfinity;
                foreach (var arc in network.OutgoingFrom(current))
                {
                    if (visited.Contains(arc.Target))
                        continue;
                    if (forbiddenArcIds is not null && forbiddenArcIds.Contains(arc.Id))
                        continue;
                    if (!distToTarget.TryGetValue(arc.Target, out var dt))
                        continue;
                    var val = Math.Max(0d, weightSelector(arc)) + dt;
                    if (val < bestVal)
                    {
                        bestVal = val;
                        bestArc = arc;
                        found = true;
                    }
                }
                if (!found)
                    return null;

                arcs.Add(bestArc);
                total += Math.Max(0d, weightSelector(bestArc));
                current = bestArc.Target;
                nodes.Add(current);
                visited.Add(current);
            }
            return new ShortestPath(nodes, arcs, total);
        }

        // Randomized source->target walk that never exceeds the budget: at each node
        // only arcs whose successor can still reach the target within the remaining
        // budget are eligible, and one is chosen uniformly at random. Returns null if
        // the walk dead-ends (the caller retries).
        private static ShortestPath? RandomLabelWalk(
            Network network, string source, string target,
            Func<Arc, double> weightSelector, Dictionary<string, double> distToTarget,
            double budget, Random random, IReadOnlySet<string>? forbiddenArcIds)
        {
            var arcs = new List<Arc>();
            var nodes = new List<string> { source };
            var visited = new HashSet<string>(StringComparer.Ordinal) { source };
            var current = source;
            var total = 0d;
            var choices = new List<Arc>();

            while (!string.Equals(current, target, StringComparison.Ordinal))
            {
                choices.Clear();
                foreach (var arc in network.OutgoingFrom(current))
                {
                    if (visited.Contains(arc.Target))
                        continue;
                    if (forbiddenArcIds is not null && forbiddenArcIds.Contains(arc.Id))
                        continue;
                    if (!distToTarget.TryGetValue(arc.Target, out var dt))
                        continue;
                    var w = Math.Max(0d, weightSelector(arc));
                    if (total + w + dt > budget)
                        continue;
                    choices.Add(arc);
                }
                if (choices.Count == 0)
                    return null;

                var pick = choices[random.Next(choices.Count)];
                arcs.Add(pick);
                total += Math.Max(0d, weightSelector(pick));
                current = pick.Target;
                nodes.Add(current);
                visited.Add(current);
            }
            return new ShortestPath(nodes, arcs, total);
        }

        // Arc-based multi-commodity flow MILP solved with Gurobi.
        //
        // Each request r is routed over its OWN sub-network (request.SubNetwork),
        // so x[a, r] only exists for arcs a in that request's sub-network. The arc
        // activation variables y[a] and the capacity constraints are global per arc
        // so that requests sharing a leg compete for the same capacity.
        //
        // Variables:
        //   x[a, r] in {0,1}  : 1 iff request r is routed over arc a (a in subnet(r))
        //   y[a]    in {0,1}  : 1 iff arc a carries any flow
        //
        // Flow balance per request r with origin o_r and destination d_r:
        //   sum_{a in out(o_r)} x[a,r] = 1,   sum_{a in in(o_r)} x[a,r] = 0
        //   sum_{a in in(d_r)}  x[a,r] = 1,   sum_{a in out(d_r)} x[a,r] = 0
        //   For every transit node n: sum_{a in in(n)} x[a,r] = sum_{a in out(n)} x[a,r]
        //
        // Capacity / leg usage:
        //   sum_r v_r * x[a,r] <= c_a * y[a]                for every arc a
        //
        // Objective:
        //   minimize sum_a FixedCost_a * y[a] + sum_{a,r} v_r * UsageCostPerGbps_a * x[a,r]
        //
        // Requests without a sub-network (infeasible) are skipped.
        public static double WeakLPRelax(Network network, Dictionary<string, Request> requests, out double lowerBound, out double timeToMatchMs, double targetObjective = double.NaN, bool feasibilityOnly = false, string? logFile = null, double maxRunTimeSeconds = 600d, bool addSiCuts = true)
        {
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(requests);
            if (maxRunTimeSeconds <= 0d)
                throw new ArgumentOutOfRangeException(nameof(maxRunTimeSeconds), "must be positive.");

            lowerBound = double.NegativeInfinity;
            timeToMatchMs = double.NaN;

            if (requests.Count == 0)
                throw new InvalidOperationException("No requests to route.");

            // Only requests with a precomputed sub-network can be routed.
            var routable = new List<Request>(requests.Count);
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is not null)
                    routable.Add(req);
            }

            if (routable.Count == 0)
                throw new InvalidOperationException("No request has a feasible sub-network to route.");

            using var env = new GRBEnv(empty: true);
            
            var licenseId = Environment.GetEnvironmentVariable("GUROBI_LICENSEID");
            var accessId = Environment.GetEnvironmentVariable("GUROBI_WLSACCESSID");
            var secret = Environment.GetEnvironmentVariable("GUROBI_WLSSECRET");
            
            if (!string.IsNullOrEmpty(licenseId))
                env.Set("LICENSEID", licenseId);
            
            if (!string.IsNullOrEmpty(accessId))
                env.Set("WLSACCESSID", accessId);
            
            if (!string.IsNullOrEmpty(secret))
                env.Set("WLSSECRET", secret);
            
            ExecutionSettings.ConfigureGurobiEnvironment(env);
            
            env.Start();
            using var model = new GRBModel(env);
            model.ModelName = "ArcFlow";
            // model.Parameters.TimeLimit = maxRunTimeSeconds;
            // When a log file is supplied, also write the Gurobi solver log there.
            if (!string.IsNullOrEmpty(logFile))
            {
                model.Parameters.LogFile = logFile;
                model.Parameters.LogToConsole = 1;
            }
            model.Parameters.Method = ExecutionSettings.WeakLpMethod;

            // Variables stored as dictionaries.
            //   x : (arcId, requestId) -> binary routing variable
            //   y : arcId              -> binary arc-activation variable
            // The same directed arc can appear in several requests' sub-networks, so a
            // stable string arc id (arc.Id) keys the shared y variable and capacity.
            var x = new Dictionary<(string Arc, string Request), GRBVar>();
            var y = new Dictionary<string, GRBVar>(StringComparer.Ordinal);
            var arcById = new Dictionary<string, Arc>(StringComparer.Ordinal);

            // Constraints stored as dictionaries.
            var outOriginConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var inOriginConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var inDestConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var outDestConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var balanceConstr = new Dictionary<(string Request, string Node), GRBConstr>();
            var capacityConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);

            // Accumulated capacity usage per arc: sum_r v_r * x[a,r].
            var capacityUsage = new Dictionary<string, GRBLinExpr>(StringComparer.Ordinal);

            var obj = new GRBLinExpr();

            foreach (var req in routable)
            {
                var sub = req.SubNetwork!.Network;

                foreach (var arc in sub.Arcs)
                {
                    var arcId = arc.Id;
                    var xVar = model.AddVar(0d, 1d, 0d, GRB.CONTINUOUS, $"x[{arcId},{req.Id}]");
                    x[(arcId, req.Id)] = xVar;

                    arcById.TryAdd(arcId, arc);

                    // Ensure an activation variable y_s exists for the arc's resource s(a).
                    if (arc.ResourceId is { } resId && !y.ContainsKey(resId))
                        y[resId] = model.AddVar(0d, 1d, 0d, GRB.CONTINUOUS, $"y[{resId}]");

                    // Usage cost contribution.
                    obj.AddTerm(req.DemandGbps * arc.UsageCostPerGbps, xVar);

                    // Capacity usage contribution.
                    if (!capacityUsage.TryGetValue(arcId, out var usage))
                    {
                        usage = new GRBLinExpr();
                        capacityUsage[arcId] = usage;
                    }
                    usage.AddTerm(req.DemandGbps, xVar);
                }

                // Flow balance over the request's own sub-network using its adjacency.
                var outOrigin = new GRBLinExpr();
                foreach (var arc in sub.OutgoingFrom(req.Source))
                    outOrigin.AddTerm(1d, x[(arc.Id, req.Id)]);
                outOriginConstr[req.Id] = model.AddConstr(outOrigin == 1d, $"out_origin[{req.Id}]");

                var inOrigin = new GRBLinExpr();
                foreach (var arc in sub.IncomingTo(req.Source))
                    inOrigin.AddTerm(1d, x[(arc.Id, req.Id)]);
                inOriginConstr[req.Id] = model.AddConstr(inOrigin == 0d, $"in_origin[{req.Id}]");

                var inDest = new GRBLinExpr();
                foreach (var arc in sub.IncomingTo(req.Target))
                    inDest.AddTerm(1d, x[(arc.Id, req.Id)]);
                inDestConstr[req.Id] = model.AddConstr(inDest == 1d, $"in_dest[{req.Id}]");

                var outDest = new GRBLinExpr();
                foreach (var arc in sub.OutgoingFrom(req.Target))
                    outDest.AddTerm(1d, x[(arc.Id, req.Id)]);
                outDestConstr[req.Id] = model.AddConstr(outDest == 0d, $"out_dest[{req.Id}]");

                foreach (var node in sub.Nodes)
                {
                    if (string.Equals(node, req.Source, StringComparison.Ordinal))
                        continue;
                    if (string.Equals(node, req.Target, StringComparison.Ordinal))
                        continue;

                    var inN = new GRBLinExpr();
                    foreach (var arc in sub.IncomingTo(node))
                        inN.AddTerm(1d, x[(arc.Id, req.Id)]);

                    var outN = new GRBLinExpr();
                    foreach (var arc in sub.OutgoingFrom(node))
                        outN.AddTerm(1d, x[(arc.Id, req.Id)]);

                    balanceConstr[(req.Id, node)] = model.AddConstr(inN == outN, $"balance[{req.Id},{node}]");
                }
            }

            // (1b) Capacity: sum_r v_r * x[a,r] <= c_a * y_{s(a)}.
            foreach (var (arcId, usage) in capacityUsage)
            {
                var arc = arcById[arcId];
                if (arc.ResourceId is { } resId)
                {
                    var cap = new GRBLinExpr();
                    cap.AddTerm(arc.CapacityGbps, y[resId]);
                    capacityConstr[arcId] = model.AddConstr(usage <= cap, $"cap[{arcId}]");
                }
                else
                {
                    capacityConstr[arcId] = model.AddConstr(usage <= arc.CapacityGbps, $"cap[{arcId}]");
                }
            }

            // Strong-inequality (SI) cuts: x[a,r] <= y_{s(a)} for every a, r.
            if (addSiCuts)
            {
                foreach (var ((arcId, reqId), xVar) in x)
                {
                    if (arcById[arcId].ResourceId is not { } siRes)
                        continue;
                    var lhs = new GRBLinExpr();
                    lhs.AddTerm(1d, xVar);
                    var rhs = new GRBLinExpr();
                    rhs.AddTerm(1d, y[siRes]);
                    model.AddConstr(lhs <= rhs, $"si[{arcId},{reqId}]");
                }
            }

            // Fixed cost sigma_s of opened resources.
            foreach (var (resId, yVar) in y)
                obj.AddTerm(network.GetResource(resId).FixedCost, yVar);

            // Feasibility-only mode: drop the objective (route anything that fits) and
            // stop as soon as the first feasible solution is found.

            model.SetObjective(obj, GRB.MINIMIZE);
            model.Optimize();
            return model.ObjVal;
        }
        public static ArcFlowSolution? SolveArcBasedModel(Network network, Dictionary<string, Request> requests, out double lowerBound, out double timeToMatchMs, double targetObjective = double.NaN, bool feasibilityOnly = false, string? logFile = null, double maxRunTimeSeconds = 600d)
        {
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(requests);
            if (maxRunTimeSeconds <= 0d)
                throw new ArgumentOutOfRangeException(nameof(maxRunTimeSeconds), "must be positive.");

            lowerBound = double.NegativeInfinity;
            timeToMatchMs = double.NaN;

            if (requests.Count == 0)
                throw new InvalidOperationException("No requests to route.");

            // Only requests with a precomputed sub-network can be routed.
            var routable = new List<Request>(requests.Count);
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is not null)
                    routable.Add(req);
            }

            if (routable.Count == 0)
                throw new InvalidOperationException("No request has a feasible sub-network to route.");

            using var env = new GRBEnv(empty: true);
            
            var licenseId = Environment.GetEnvironmentVariable("GUROBI_LICENSEID");
            var accessId = Environment.GetEnvironmentVariable("GUROBI_WLSACCESSID");
            var secret = Environment.GetEnvironmentVariable("GUROBI_WLSSECRET");
            
            if (!string.IsNullOrEmpty(licenseId))
                env.Set("LICENSEID", licenseId);
            
            if (!string.IsNullOrEmpty(accessId))
                env.Set("WLSACCESSID", accessId);
            
            if (!string.IsNullOrEmpty(secret))
                env.Set("WLSSECRET", secret);
            
            ExecutionSettings.ConfigureGurobiEnvironment(env);
            
            env.Start();
            using var model = new GRBModel(env);
            model.ModelName = "ArcFlow";
            model.Parameters.TimeLimit = maxRunTimeSeconds;
            // When a log file is supplied, also write the Gurobi solver log there.
            if (!string.IsNullOrEmpty(logFile))
            {
                model.Parameters.LogFile = logFile;
                model.Parameters.LogToConsole = 1;
            }

            // Variables stored as dictionaries.
            //   x : (arcId, requestId) -> binary routing variable
            //   y : arcId              -> binary arc-activation variable
            // The same directed arc can appear in several requests' sub-networks, so a
            // stable string arc id (arc.Id) keys the shared y variable and capacity.
            var x = new Dictionary<(string Arc, string Request), GRBVar>();
            var y = new Dictionary<string, GRBVar>(StringComparer.Ordinal);
            var arcById = new Dictionary<string, Arc>(StringComparer.Ordinal);

            // Constraints stored as dictionaries.
            var outOriginConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var inOriginConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var inDestConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var outDestConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);
            var balanceConstr = new Dictionary<(string Request, string Node), GRBConstr>();
            var capacityConstr = new Dictionary<string, GRBConstr>(StringComparer.Ordinal);

            // Accumulated capacity usage per arc: sum_r v_r * x[a,r].
            var capacityUsage = new Dictionary<string, GRBLinExpr>(StringComparer.Ordinal);

            var obj = new GRBLinExpr();

            foreach (var req in routable)
            {
                var sub = req.SubNetwork!.Network;

                foreach (var arc in sub.Arcs)
                {
                    var arcId = arc.Id;
                    var xVar = model.AddVar(0d, 1d, 0d, GRB.BINARY, $"x[{arcId},{req.Id}]");
                    x[(arcId, req.Id)] = xVar;

                    arcById.TryAdd(arcId, arc);

                    // Ensure an activation variable y_s exists for the arc's resource s(a).
                    if (arc.ResourceId is { } resId && !y.ContainsKey(resId))
                        y[resId] = model.AddVar(0d, 1d, 0d, GRB.BINARY, $"y[{resId}]");

                    // Usage cost contribution.
                    obj.AddTerm(req.DemandGbps * arc.UsageCostPerGbps, xVar);

                    // Capacity usage contribution.
                    if (!capacityUsage.TryGetValue(arcId, out var usage))
                    {
                        usage = new GRBLinExpr();
                        capacityUsage[arcId] = usage;
                    }
                    usage.AddTerm(req.DemandGbps, xVar);
                }

                // Flow balance over the request's own sub-network using its adjacency.
                var outOrigin = new GRBLinExpr();
                foreach (var arc in sub.OutgoingFrom(req.Source))
                    outOrigin.AddTerm(1d, x[(arc.Id, req.Id)]);
                outOriginConstr[req.Id] = model.AddConstr(outOrigin == 1d, $"out_origin[{req.Id}]");

                var inOrigin = new GRBLinExpr();
                foreach (var arc in sub.IncomingTo(req.Source))
                    inOrigin.AddTerm(1d, x[(arc.Id, req.Id)]);
                inOriginConstr[req.Id] = model.AddConstr(inOrigin == 0d, $"in_origin[{req.Id}]");

                var inDest = new GRBLinExpr();
                foreach (var arc in sub.IncomingTo(req.Target))
                    inDest.AddTerm(1d, x[(arc.Id, req.Id)]);
                inDestConstr[req.Id] = model.AddConstr(inDest == 1d, $"in_dest[{req.Id}]");

                var outDest = new GRBLinExpr();
                foreach (var arc in sub.OutgoingFrom(req.Target))
                    outDest.AddTerm(1d, x[(arc.Id, req.Id)]);
                outDestConstr[req.Id] = model.AddConstr(outDest == 0d, $"out_dest[{req.Id}]");

                foreach (var node in sub.Nodes)
                {
                    if (string.Equals(node, req.Source, StringComparison.Ordinal))
                        continue;
                    if (string.Equals(node, req.Target, StringComparison.Ordinal))
                        continue;

                    var inN = new GRBLinExpr();
                    foreach (var arc in sub.IncomingTo(node))
                        inN.AddTerm(1d, x[(arc.Id, req.Id)]);

                    var outN = new GRBLinExpr();
                    foreach (var arc in sub.OutgoingFrom(node))
                        outN.AddTerm(1d, x[(arc.Id, req.Id)]);

                    balanceConstr[(req.Id, node)] = model.AddConstr(inN == outN, $"balance[{req.Id},{node}]");
                }
            }

            // (1b) Capacity: sum_r v_r * x[a,r] <= c_a * y_{s(a)}.
            // Arcs without a resource (e.g. dummies) are bounded by c_a only.
            foreach (var (arcId, usage) in capacityUsage)
            {
                var arc = arcById[arcId];
                if (arc.ResourceId is { } resId)
                {
                    var cap = new GRBLinExpr();
                    cap.AddTerm(arc.CapacityGbps, y[resId]);
                    capacityConstr[arcId] = model.AddConstr(usage <= cap, $"cap[{arcId}]");
                }
                else
                {
                    capacityConstr[arcId] = model.AddConstr(usage <= arc.CapacityGbps, $"cap[{arcId}]");
                }
            }

            // Fixed cost sigma_s of opened resources.
            foreach (var (resId, yVar) in y)
                obj.AddTerm(network.GetResource(resId).FixedCost, yVar);

            // Feasibility-only mode: drop the objective (route anything that fits) and
            // stop as soon as the first feasible solution is found.
            if (feasibilityOnly)
            {
                model.SetObjective(new GRBLinExpr(), GRB.MINIMIZE);
                model.Parameters.SolutionLimit = 1;
            }
            else
            {
                model.SetObjective(obj, GRB.MINIMIZE);
            }

            // Record when the solver first reaches a target-quality incumbent.
            ArcMipTimeToTargetCallback? cb = null;
            if (!double.IsNaN(targetObjective))
            {
                cb = new ArcMipTimeToTargetCallback(targetObjective);
                model.SetCallback(cb);
            }

            model.Optimize();

            lowerBound = model.ObjBound;
            if (cb is not null)
                timeToMatchMs = cb.TimeToTargetSeconds;

            // Accept an optimal solution, or any feasible incumbent found before the
            // time/solution limit halted the search (so a 300s timeout still reports
            // the best incumbent + bound rather than nothing).
            if (model.Status != GRB.Status.OPTIMAL && model.SolCount == 0)
                return null;

            // Extract per-request paths from their own sub-network arcs.
            var requestPaths = new Dictionary<string, IReadOnlyList<Arc>>(StringComparer.Ordinal);
            foreach (var req in routable)
            {
                var sub = req.SubNetwork!.Network;

                var used = new List<Arc>();
                foreach (var arc in sub.Arcs)
                {
                    if (x[(arc.Id, req.Id)].X > 0.5)
                        used.Add(arc);
                }
                requestPaths[req.Id] = OrderPath(used, req.Source, req.Target);
            }

            // Extract arcs belonging to opened resources.
            var usedArcsList = new List<Arc>();
            foreach (var (resId, yVar) in y)
            {
                if (yVar.X > 0.5)
                    usedArcsList.AddRange(network.GetResource(resId).Arcs);
            }

            return new ArcFlowSolution(model.ObjVal, requestPaths, usedArcsList);
        }

        // Orders a set of arcs that form a single source-to-target path.
        private static IReadOnlyList<Arc> OrderPath(List<Arc> arcs, string source, string target)
        {
            if (arcs.Count == 0)
                return Array.Empty<Arc>();

            var bySource = new Dictionary<string, Arc>(StringComparer.Ordinal);
            foreach (var arc in arcs)
                bySource[arc.Source] = arc;

            var ordered = new List<Arc>(arcs.Count);
            var current = source;
            while (bySource.TryGetValue(current, out var next))
            {
                ordered.Add(next);
                if (string.Equals(next.Target, target, StringComparison.Ordinal))
                    break;
                current = next.Target;
                if (ordered.Count > arcs.Count)
                    break; // safety against unexpected cycles
            }
            return ordered;
        }

        // Builds an INTEGER version of the master problem from a set of candidate
        // paths and optimizes it in one shot.
        //
        //   paths    : requestId -> list of path ids eligible for that request
        //   pathdict : pathId   -> RoutePath (carries its Request, Arcs and RawCost)
        //
        // Variables:
        //   x[r, p] in {0,1} : 1 iff request r is routed over path p
        //   y[a]    in {0,1} : 1 iff arc a carries any flow
        // Convexity : sum_p x[r, p] = 1                     for every request r
        // Capacity  : sum_{r,p : a in p} d_r * x[r, p] <= c_a * y[a]   per arc a
        // Objective : minimize sum_{r,p} RawCost_p * x[r, p] + sum_a FixedCost_a * y[a]
        //
        // Returns the optimal objective value, or null if the model is not optimal.
        public static double findshortestpaths(Dictionary<string,Request> requests)
        {
            double totalcost = 0;
            foreach(string r in requests.Keys)
            {
                double Weight(Arc arc)
                {
                    double w = arc.UsageCostPerGbps * requests[r].DemandGbps;
                    return w;
                }
                Request req = requests[r];
                
        
                var paths = Optimization.FindKShortestPaths(
                        req.SubNetwork.Network,
                        req.Source,
                        req.Target,
                        1,
                        Weight,
                        10000000,
                        tolerance: 0d);
                double cost =  paths[0].TotalCost;
                totalcost = totalcost + cost;
            }
            return totalcost;

           
        }
        public static int findcut(double bestincsol, double bestvariablecost, Network network)
        {
            List<Resource> resources = network.Resources.OrderByDescending(s => s.FixedCost).ToList();
            double totalcost = 0;
            foreach(Resource s in resources)
            {
               totalcost = totalcost + s.FixedCost+bestvariablecost;
            }
            List<double>cumulativecost = new List<double>();
            int cut =0;
            foreach(Resource s in resources)
            {
                totalcost = totalcost - s.FixedCost;
                cumulativecost.Add(totalcost);
                if(totalcost < bestincsol)
                {
                    cut++;
                    break;
                }
                
            }
            return cut;
            
        }


        public static IntegerMasterSolution? SolveIntegerMaster(
            Dictionary<string, List<string>> paths,
            Dictionary<string, RoutePath> pathdict,
            Network network,
            GRBEnv env,
            bool includeArcFixedCost = true,
            Dictionary<(string Request, string Path), double>? masterXValues = null,
            bool sequentialAdd = false,
            Dictionary<string, List<string>>? guaranteedPaths = null,
            CGMode mode = CGMode.Leg)
        {
            ArgumentNullException.ThrowIfNull(paths);
            ArgumentNullException.ThrowIfNull(pathdict);
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(env);

            // Optional progressive build: grow the candidate pool from high-LP-value
            // paths to the full set, re-solving the MIP at each step. Only available
            // when the LP master x values are supplied to drive the thresholds.
            if (sequentialAdd && masterXValues is not null)
                return SolveIntegerMasterSequential(
                    paths, pathdict, network, env, masterXValues, includeArcFixedCost, guaranteedPaths, mode);

            // Reuse the caller's environment (single-use license: only one env).
            using var model = new GRBModel(env);
            model.ModelName = "IntegerMaster";
            model.Parameters.TimeLimit = 3; // seconds
            // model.Parameters.Method = 2;
            //model.Parameters.MIPGap = 0.01; // require optimality
            model.Parameters.LogToConsole = 1; // silence Gurobi model logging
            // model.Parameters.ConcurrentMIP = 2; // single-threaded for reproducibility
            var x = new Dictionary<(string Request, string Path), GRBVar>();
            var y = new Dictionary<string, GRBVar>(StringComparer.Ordinal);
            var capUsage = new Dictionary<string, GRBLinExpr>(StringComparer.Ordinal);

            // LegRequest mode: sum of a request's path vars that traverse a given arc,
            // keyed by (request, arc). Each becomes a linking constraint <= y[a] below.
            var legReqUsage = new Dictionary<(string Request, string Arc), GRBLinExpr>();

            // Path-selection variables + convexity constraints + capacity usage.
            foreach (var (requestId, pathIds) in paths)
            {
                var convex = new GRBLinExpr();
                foreach (var pathId in pathIds)
                {
                    var path = pathdict[pathId];
                    var xVar = model.AddVar(0d, 1d, path.Cost, GRB.BINARY, $"x[{requestId},{pathId}]");
                    if (masterXValues is not null &&
                        masterXValues.TryGetValue((requestId, pathId), out var startVal))
                        xVar.Start = Math.Clamp(startVal, 0d, 1d);
                    x[(requestId, pathId)] = xVar;
                    convex.AddTerm(1d, xVar);

                    foreach (var arc in path.Arcs)
                    {
                        if (!capUsage.TryGetValue(arc.Id, out var usage))
                        {
                            usage = new GRBLinExpr();
                            capUsage[arc.Id] = usage;
                        }
                        usage.AddTerm(path.Request.DemandGbps, xVar);

                        if (mode == CGMode.LegRequest)
                        {
                            var lrKey = (requestId, arc.Id);
                            if (!legReqUsage.TryGetValue(lrKey, out var lrExpr))
                            {
                                lrExpr = new GRBLinExpr();
                                legReqUsage[lrKey] = lrExpr;
                            }
                            lrExpr.AddTerm(1d, xVar);
                        }
                    }
                }
                model.AddConstr(convex == 1d, $"convex[{requestId}]");
            }

            // Arc-activation variables (objective) + capacity constraints, one per
            // arc in the MAIN network (so a leg shared by several requests yields a
            // single y[a] / capacity row instead of being counted once per request).
            // In Phase 1 the fixed cost is dropped so the incumbent objective is a
            // pure count of selected dummy columns (x cost = 1 dummy, 0 real).
            // One y[s] per resource (sigma_s); capacity usage <= c_a * y[s(a)], or
            // usage <= c_a for arcs without a resource (dummy arcs).
            foreach (var res in network.Resources)
            {
                var yFixed = includeArcFixedCost ? res.FixedCost : 0d;
                y[res.Id] = model.AddVar(0d, 1d, yFixed, GRB.BINARY, $"y[{res.Id}]");
            }
            foreach (var arc in network.Arcs)
            {
                if (!capUsage.TryGetValue(arc.Id, out var usage))
                    continue;
                if (arc.ResourceId is not null && y.TryGetValue(arc.ResourceId, out var yRes))
                {
                    var cap = new GRBLinExpr();
                    cap.AddTerm(arc.CapacityGbps, yRes);
                    model.AddConstr(usage <= cap, $"cap[{arc.Id}]");
                }
                else
                {
                    model.AddConstr(usage <= arc.CapacityGbps, $"cap[{arc.Id}]");
                }
            }

            // LegRequest mode: one linking constraint per (request, arc) the request
            // actually uses: sum_{p of r using a} x[r,p] <= y[a], mirroring the LP master.
            // if (mode == CGMode.LegRequest)
            // {
            //     foreach (var ((requestId, arcId), expr) in legReqUsage)
            //     {
            //         if (y.TryGetValue(arcId, out var yVar))
            //         {
            //             var rhs = new GRBLinExpr();
            //             rhs.AddTerm(1d, yVar);
            //             model.AddConstr(expr <= rhs, $"legreq[{requestId},{arcId}]");
            //         }
            //     }
            // }

            model.ModelSense = GRB.MINIMIZE;
            model.Optimize();

            // if (model.Status != GRB.Status.OPTIMAL)
            if (model.SolCount == 0)
            {
                ReportIfInfeasible(model, "IntegerMaster");
            }
            double gap = model.Get(GRB.DoubleAttr.MIPGap);
            if (gap > 0.05)
            {
                model.Parameters.TimeLimit = 3;
                model.Parameters.MIPGap = 0.05;
                model.Optimize();

            }
            

            // No integer-feasible solution was found (e.g. infeasible pool or hit the
            // time limit with no incumbent): report nothing instead of reading ObjVal.
            if (model.SolCount == 0)
                return null;

            // Collect the selected path ids (x > 0.5) per request.
            var selected = new Dictionary<string, List<string>>(StringComparer.Ordinal);
            foreach (var ((requestId, pathId), xVar) in x)
            {
                if (xVar.X > 0.5)
                {
                    if (!selected.TryGetValue(requestId, out var list))
                    {
                        list = new List<string>();
                        selected[requestId] = list;
                    }
                    list.Add(pathId);
                }
            }

            return new IntegerMasterSolution(model.ObjVal, selected);
        }

        // Sequentially grows the candidate path pool and re-solves the integer
        // master at each step, warm-starting from the LP master's x values.
        //
        // The pool is built from the LP solution in masterXValues: the first solve
        // only sees paths whose LP value is >= 0.5, the next also admits those
        // and finally >= 0 (every path). Each solve warm-starts
        // the next, and because the last threshold admits the whole pool its result
        // is equivalent to a single full solve. The best feasible solution found
        // across the thresholds is returned.
        //
        // guaranteedPaths (e.g. the previous column-generation iteration's integer
        // solution) are always added to the pool regardless of the threshold, so the
        // prior incumbent stays selectable at every step.
        public static IntegerMasterSolution? SolveIntegerMasterSequential(
            Dictionary<string, List<string>> paths,
            Dictionary<string, RoutePath> pathdict,
            Network network,
            GRBEnv env,
            Dictionary<(string Request, string Path), double> masterXValues,
            bool includeArcFixedCost = true,
            Dictionary<string, List<string>>? guaranteedPaths = null,
            CGMode mode = CGMode.Leg)
        {
            ArgumentNullException.ThrowIfNull(paths);
            ArgumentNullException.ThrowIfNull(pathdict);
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(env);
            ArgumentNullException.ThrowIfNull(masterXValues);

            // Decreasing inclusion thresholds; the final 0 admits every path.
            var thresholds = new[] {0.5d, 0.1d, 0d };

            IntegerMasterSolution? best = null;
            // Integer solution from the previous threshold. Its selected paths warm-
            // start the next threshold's MIP so each solve builds on the last.
            IntegerMasterSolution? prev = null;

            foreach (var threshold in thresholds)
            {
                // Keep only paths whose LP value clears the current threshold. A path
                // missing from masterXValues is treated as having an LP value of 0.
                var pool = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                foreach (var (requestId, pathIds) in paths)
                {
                    var set = new HashSet<string>(StringComparer.Ordinal);
                    foreach (var pathId in pathIds)
                    {
                        masterXValues.TryGetValue((requestId, pathId), out var value);
                        if (value >= threshold)
                            set.Add(pathId);
                    }

                    // Always include the previous integer solution's paths, even when
                    // their LP value falls below the current threshold.
                    if (guaranteedPaths is not null &&
                        guaranteedPaths.TryGetValue(requestId, out var guaranteed))
                        set.UnionWith(guaranteed);

                    // A request with no qualifying path cannot satisfy convexity, so
                    // seed it with its highest-LP-value path to keep the pool feasible.
                    if (set.Count == 0 && pathIds.Count > 0)
                    {
                        var seedPathId = pathIds[0];
                        var seedValue = double.NegativeInfinity;
                        foreach (var pathId in pathIds)
                        {
                            masterXValues.TryGetValue((requestId, pathId), out var value);
                            if (value > seedValue)
                            {
                                seedValue = value;
                                seedPathId = pathId;
                            }
                        }
                        set.Add(seedPathId);
                    }

                    pool[requestId] = new List<string>(set);
                }

                // Warm start: begin from the LP master values, then force the previous
                // threshold's selected paths to 1 so this solve resumes from that
                // integer-feasible point instead of restarting from the LP solution.
                var startValues = new Dictionary<(string Request, string Path), double>(masterXValues);
                if (prev is not null)
                {
                    foreach (var (reqId, pathIds) in prev.SelectedPaths)
                        foreach (var pid in pathIds)
                            startValues[(reqId, pid)] = 1d;
                }

                var solution = SolveIntegerMaster(
                    pool,
                    pathdict,
                    network,
                    env,
                    includeArcFixedCost,
                    startValues,
                    mode: mode);

                if (solution is not null)
                {
                    prev = solution; // chain into the next threshold's warm start
                    if (best is null || solution.Objective < best.Objective)
                        best = solution;
                }
            }

            return best;
        }

        // Big-M usage cost for the artificial (dummy) arcs that guarantee a
        // feasible master: routing over one is always possible but very expensive.
        private const double DummyArcCost = 1e9;

        // Slight headroom on dummy capacity so rounding / coupling never makes the
        // artificial fallback itself infeasible.
        private const double DummyCapacityFactor = 1.1;

        // Penalty level assigned to a dummy column the first time the incumbent is
        // caught routing over it: a large jump (from the seed cost of 1) that strongly
        // discourages reuse without immediately swamping the model in big-M numbers.
        private const double DummyPenaltyBase = 10000d;

        // Multiplicative growth applied to the shared dummy penalty once the current
        // level has already failed to dislodge the violators. Kept small so the level
        // creeps up gradually rather than exploding.
        private const double DummyCostGrowth = 4.0d;

        // Branch-and-price driver: runs the usual Phase 1 / Phase 2 column generation
        // at the root, then explores the arc-branching tree depth-first down to
        // branchDepth levels.
        public static IntegerMasterSolution? BranchAndPrice(Network network, Dictionary<string, Request> requests, out double cgTimeToBestMs, out double cgDualBound, out Dictionary<string, double> cgArcValues, out BranchingArc? branchingCandidate, CGMode mode = CGMode.Leg, int maxIterations = 100, int numPaths = 1, bool randomizedPricing = false, IReadOnlyList<int>? dualLbPolicies = null, string? outputDir = null, int integerMasterInterval = 1, double phase2OptimalityTol = 0.001, int branchDepth = 5, int legReqIdleLimit = 2, double hybridSwitchTolerance = 0.01, int hybridSwitchStallIterations = 5, int columnIdleLimit = 0)
        {
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(requests);

            return CGoptimize(
                network, requests, out cgTimeToBestMs, out cgDualBound, out cgArcValues, out branchingCandidate,
                mode, maxIterations, numPaths, randomizedPricing, dualLbPolicies, outputDir, integerMasterInterval, phase2OptimalityTol,
                branchAndPrice: true, branchDepth: branchDepth, legReqIdleLimit: legReqIdleLimit, hybridSwitchTolerance: hybridSwitchTolerance, hybridSwitchStallIterations: hybridSwitchStallIterations, columnIdleLimit: columnIdleLimit);
        }

        // Column-generation driver. As a first step it injects a dummy
        // origin -> destination arc into every feasible request's sub-network and
        // into the shared main network, guaranteeing the master is always feasible.
        //
        // Each request's dummy arc is added only to that request's own sub-network
        // (plus the main network), so dummy capacity is never shared between
        // requests. The same arc instance is registered in the main network so its
        // y[a] activation variable exists globally.
        public static IntegerMasterSolution? CGoptimize(Network network, Dictionary<string, Request> requests, out double cgTimeToBestMs, out double cgDualBound, out Dictionary<string, double> cgArcValues, out BranchingArc? branchingCandidate, CGMode mode = CGMode.Leg, int maxIterations = 100, int numPaths = 1, bool randomizedPricing = false, IReadOnlyList<int>? dualLbPolicies = null, string? outputDir = null, int integerMasterInterval = 1, double phase2OptimalityTol = 0.001, bool branchAndPrice = false, int branchDepth = 5, bool rootRelaxationOnly = false, int legReqIdleLimit = 2, double hybridSwitchTolerance = 0.01, int hybridSwitchStallIterations = 5, int columnIdleLimit = 0)
        {
            ArgumentNullException.ThrowIfNull(network);
            ArgumentNullException.ThrowIfNull(requests);
            if (integerMasterInterval <= 0)
                throw new ArgumentOutOfRangeException(nameof(integerMasterInterval), "must be positive.");
            if (!double.IsFinite(hybridSwitchTolerance) || hybridSwitchTolerance <= 0d)
                throw new ArgumentOutOfRangeException(nameof(hybridSwitchTolerance), "must be finite and positive.");
            if (hybridSwitchStallIterations <= 0)
                throw new ArgumentOutOfRangeException(nameof(hybridSwitchStallIterations), "must be positive.");
            // Policies run in Phase 2 one after another from the same Phase 1 basis.
            // Phase 1 itself uses the first listed policy.
            IReadOnlyList<int> policies = dualLbPolicies ?? new[] { 1 };
            if (policies.Count == 0)
                throw new ArgumentException("must list at least one policy.", nameof(dualLbPolicies));
            if (policies.Any(p => p < 0 || p > 4))
                throw new ArgumentOutOfRangeException(nameof(dualLbPolicies), "each policy must be 0, 1, 2, 3, or 4.");
            if (policies.Distinct().Count() != policies.Count)
                throw new ArgumentException("must not repeat a policy.", nameof(dualLbPolicies));
            var phase1Policy = policies[0];
            cgTimeToBestMs = double.NaN;
            cgDualBound = double.NegativeInfinity;
            cgArcValues = new Dictionary<string, double>(StringComparer.Ordinal);            branchingCandidate = null;

            // Single shared Gurobi environment for the whole column-generation run
            // (single-use license: the LP master and every integer master reuse it).
            // Owned here and explicitly disposed in the finally below.
            using var env = new GRBEnv(empty: true);
            
            var licenseId = Environment.GetEnvironmentVariable("GUROBI_LICENSEID");
            var accessId = Environment.GetEnvironmentVariable("GUROBI_WLSACCESSID");
            var secret = Environment.GetEnvironmentVariable("GUROBI_WLSSECRET");
            
            if (!string.IsNullOrEmpty(licenseId))
                env.Set("LICENSEID", licenseId);
            
            if (!string.IsNullOrEmpty(accessId))
                env.Set("WLSACCESSID", accessId);
            
            if (!string.IsNullOrEmpty(secret))
                env.Set("WLSSECRET", secret);
            
            ExecutionSettings.ConfigureGurobiEnvironment(env);
            env.Set(GRB.IntParam.OutputFlag, 0); // silence Gurobi model logging
            env.Start();

            CGMaster? master = null;
            CGMaster? phase2Master = null;

            // Captured phase 1 / phase 2 iteration logs, flushed to <outputDir> below.
            var phase1Log = new StringBuilder();
            var phase2Runs = new List<(string Suffix, StringBuilder Log, StringBuilder Csv)>();

            // CSV mirrors of the iteration tables (one row per iteration).
            var phase1Csv = new StringBuilder();
            const string CsvHeader = "iter,master_lp,incumbent,dual_bound,lp_s,int_s,sub_s";
            phase1Csv.AppendLine(CsvHeader);
            try
            {

            // Mirror a log line to the console and the matching phase log buffer.
            void LogP1(string line)
            {
                Console.WriteLine(line);
                phase1Log.AppendLine(line);
            }

            // pathId -> RoutePath, shared lookup used to build the master.
            var pathdict = new Dictionary<string, RoutePath>(StringComparer.Ordinal);

            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is null)
                    continue;

                // Route each request's dummy through its OWN artificial node so the
                // two legs get unique ids in the shared main network. Distinct
                // requests that share an origin/destination therefore never collide
                // on a single Source->Target dummy arc, and never share dummy
                // capacity (which previously caused infeasibility).
                var dummyNode = $"__dummy__[{req.Id}]";
                var dummyCap = req.DemandGbps * DummyCapacityFactor;

                var leg1 = new Arc(
                    LinkId: $"dummy[{req.Id}]#1",
                    Source: req.Source,
                    Target: dummyNode,
                    LengthKm: 0d,
                    DelayMs: 0d,
                    CapacityGbps: dummyCap,
                    FixedCost: 0d,
                    UsageCostPerGbps: DummyArcCost,
                    Weight: DummyArcCost);

                var leg2 = new Arc(
                    LinkId: $"dummy[{req.Id}]#2",
                    Source: dummyNode,
                    Target: req.Target,
                    LengthKm: 0d,
                    DelayMs: 0d,
                    CapacityGbps: dummyCap,
                    FixedCost: 0d,
                    UsageCostPerGbps: DummyArcCost,
                    Weight: DummyArcCost);

                // Both legs go into this request's sub-network AND the shared main
                // network; the unique dummy node keeps every id distinct.
                req.SubNetwork.Network.AddArc(leg1);
                req.SubNetwork.Network.AddArc(leg2);
                network.AddArc(leg1);
                network.AddArc(leg2);

                // Seed the master with a dummy path (the two dummy legs) so every
                // request starts with at least one feasible column.
                var pathId = $"dummy[{req.Id}]";
                var dummyPath = new RoutePath(pathId, req, new[] { leg1, leg2 });
                dummyPath.CalculateCost(); // RawCost, for completeness
                dummyPath.Cost = 1d;       // Phase-1: dummy columns cost 1, real cost 0

                req.Paths.Add(pathId);
                pathdict[pathId] = dummyPath;
            }
            master = new CGMaster(env, requests, pathdict, network, mode: mode, enable_dual_lb: phase1Policy, legReqIdleLimit: legReqIdleLimit, hybridSwitchTolerance: hybridSwitchTolerance, hybridSwitchStallIterations: hybridSwitchStallIterations, columnIdleLimit: columnIdleLimit);

            // Per request: arc-id signature -> the pathId already created for it.
            // Lets us recognise a re-proposed path and avoid adding duplicates.
            var sigToId = new Dictionary<string, Dictionary<string, string>>(StringComparer.Ordinal);
            foreach (var (pid, rp) in pathdict)
            {
                if (!sigToId.TryGetValue(rp.Request.Id, out var map))
                {
                    map = new Dictionary<string, string>(StringComparer.Ordinal);
                    sigToId[rp.Request.Id] = map;
                }
                map[Signature(rp.Arcs)] = pid;
            }

            // ---- Phase 1: feasibility -------------------------------------------------
            // Real columns cost 0 and dummy columns keep their big-M cost, so the master
            // objective only penalizes dummy usage. Root-relaxation-only mode stops
            // at a zero LP objective; otherwise require a dummy-free integer solution.
            const double RcTol = 1e-8;
            var columnCounter = 0;
            // Monotonic penalty for dummy columns the incumbent keeps selecting. It
            // only ever grows: a dummy that violated earlier retains its (already high)
            // cost, and when a different dummy becomes the violator it is raised
            // straight to this same level instead of restarting from the seed cost.
            // Starts at the base level so the FIRST violating dummy jumps from its seed
            // cost (1) straight to DummyPenaltyBase.
            var dummyPenalty = DummyPenaltyBase;
            IntegerMasterSolution? feasible = null;
            var phase1LpFeasible = false;
            // Basic x columns of the last optimal Phase 1 LP; Phase 2 starts from these.
            HashSet<string>? phase1Basis = null;

            LogP1("");
            LogP1("Phase 1 (feasibility):");
            LogP1($"  {"iter",4} | {"master LP",14} | {"incumbent",14} | {"dual bound",14} | {"LP s",10} | {"int s",10} | {"sub s",10}");

            var phase1Sw = Stopwatch.StartNew();
            for (var iter = 0; iter < 200; iter++)
            {
                // 1) Solve the LP master and refresh duals.
                var lpSw = Stopwatch.StartNew();
                master.Optimize();
                lpSw.Stop();
                if (master.Status != GRB.Status.OPTIMAL)
                    break;

                phase1Basis = CaptureBasicColumns(master);

                if (rootRelaxationOnly && Math.Abs(master.Model.ObjVal) <= RcTol)
                {
                    phase1LpFeasible = true;
                    LogP1($"  {iter,4} | {master.Model.ObjVal,14:F4} | {"-",14} | {"-",14} | {lpSw.Elapsed.TotalSeconds,10:F1} | {0d,10:F1} | {"-",10}");
                    phase1Csv.AppendLine($"{iter},{master.Model.ObjVal:F4},-,,{lpSw.Elapsed.TotalSeconds:F1},0.0,");
                    LogP1("  Phase 1 LP objective is zero; LP feasibility established.");
                    break;
                }

                // 2) Columns with non-zero x in the LP solution.
                var lpSupport = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                foreach (var ((reqId, pathId), xVar) in master.X)
                {
                    if (xVar.X > 0)
                    {
                        if (!lpSupport.TryGetValue(reqId, out var list))
                        {
                            list = new List<string>();
                            lpSupport[reqId] = list;
                        }
                        list.Add(pathId);
                    }
                }

                // 3 + 4) Combine LP support with every known column and resolve the
                //        integer master to look for a dummy-free incumbent.
                var combined = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                foreach (var req in requests.Values)
                {
                    combined[req.Id] = new List<string>(req.Paths);
                }

                var intSw = new Stopwatch();
                IntegerMasterSolution? incumbent = null;
                if (!rootRelaxationOnly)
                {
                    intSw.Start();
                    incumbent = SolveIntegerMaster(
                        combined, pathdict, network, env, includeArcFixedCost: false, mode: mode);
                    intSw.Stop();
                }

                var incStr1 = incumbent is null ? "-" : incumbent.Objective.ToString("F4");

                if (incumbent is not null && UsesNoDummy(incumbent))
                {
                    LogP1($"  {iter,4} | {master.Model.ObjVal,14:F4} | {incStr1,14} | {"-",14} | {lpSw.Elapsed.TotalSeconds,10:F1} | {intSw.Elapsed.TotalSeconds,10:F1} | {"-",10}");
                    phase1Csv.AppendLine($"{iter},{master.Model.ObjVal:F4},{incStr1},,{lpSw.Elapsed.TotalSeconds:F1},{intSw.Elapsed.TotalSeconds:F1},");
                    feasible = incumbent; // dummy-free integer solution -> Phase 1 done.
                    break;
                }

                // 5) Column cleaning: remove every non-dummy column whose reduced
                //    cost has been strictly positive (with X == 0) for more than
                //    columnIdleLimit consecutive rounds. Zero-reduced-cost columns are kept.
                var toRemove = master.CollectIdleColumns(
                    1e-7, keep: pathId => pathId.StartsWith("dummy[", StringComparison.Ordinal)); // dummies stay until Phase 2

                // 6) Price out new columns on the original network. Phase 1 uses
                //    zero real-arc cost (only dummy columns are penalized), so the
                //    pricing weight excludes the fixed/variable arc cost.
                var subSw = Stopwatch.StartNew();
                // Original-network pricing returns one path per request; numPaths governs Phase 2 only.
                var priced = master.SolveSubproblem(
                    requests, 1, includeArcCost: false, randomizedPricing: false,
                    originalNetworkHeuristic: true);
                subSw.Stop();

                LogP1($"  {iter,4} | {master.Model.ObjVal,14:F4} | {incStr1,14} | {"-",14} | {lpSw.Elapsed.TotalSeconds,10:F1} | {intSw.Elapsed.TotalSeconds,10:F1} | {subSw.Elapsed.TotalSeconds,10:F1}");
                phase1Csv.AppendLine($"{iter},{master.Model.ObjVal:F4},{incStr1},,{lpSw.Elapsed.TotalSeconds:F1},{intSw.Elapsed.TotalSeconds:F1},{subSw.Elapsed.TotalSeconds:F1}");

                // 7) Add improving columns, reusing an existing column id when the
                //    same path (same arc signature) was created before.
                var toAdd = new List<string>();
                foreach (var (reqId, paths) in priced)
                {
                    foreach (var sp in paths)
                    {
                        if (!sigToId.TryGetValue(reqId, out var map))
                        {
                            map = new Dictionary<string, string>(StringComparer.Ordinal);
                            sigToId[reqId] = map;
                        }

                        var sig = Signature(sp.Arcs);
                        if (map.TryGetValue(sig, out var existingPid))
                        {
                            // Path already exists. Re-add only if it is not currently
                            // in the model (e.g. it was cleaned out in a prior iteration).
                            if (!master.X.ContainsKey((reqId, existingPid)))
                                toAdd.Add(existingPid);
                            continue;
                        }

                        var req = requests[reqId];
                        var newId = $"p[{reqId}]#{columnCounter++}";
                        var col = new RoutePath(newId, req, sp.Arcs);
                        col.CalculateCost();   // sets RawCost (true cost)
                        col.Cost = 0d;         // Phase-1: tone down real-path cost to 0

                        pathdict[newId] = col;
                        req.Paths.Add(newId);
                        map[sig] = newId;
                        toAdd.Add(newId);
                    }
                }

                // No improving column was priced. If the incumbent still routes over
                // dummy arcs (its objective counts selected dummy columns, so it is
                // strictly positive), restart the master from the incumbent: drop every
                // current column, keep only the integer solution's selected paths, and
                // artificially inflate the cost of the dummy columns it uses so the next
                // LP/pricing round is pushed to replace them with real paths. Otherwise,
                // with nothing to add or clean, Phase 1 cannot progress.
                if (toAdd.Count == 0)
                {
                    var incumbentUsesDummy = incumbent is not null && incumbent.Objective > RcTol;
                    if (incumbentUsesDummy)
                    {
                        // Bring the incumbent's dummy columns up to a single shared
                        // penalty level. A NEW violator (still below the level, e.g. a
                        // freshly-selected dummy at its seed cost) is only raised UP TO
                        // the current level - matching the previously-penalized dummies
                        // instead of being pushed past them. The level itself escalates
                        // only once EVERY violating dummy is already sitting at it, i.e.
                        // the current penalty was not enough to dislodge them.
                        var incumbentDummies = new List<string>();
                        foreach (var (_, pathIds) in incumbent!.SelectedPaths)
                            foreach (var pid in pathIds)
                                if (pid.StartsWith("dummy[", StringComparison.Ordinal))
                                    incumbentDummies.Add(pid);

                        var allAtLevel = true;
                        foreach (var pid in incumbentDummies)
                        {
                            if (pathdict[pid].Cost < dummyPenalty)
                            {
                                allAtLevel = false;
                                break;
                            }
                        }

                        // Only escalate when the current level already failed to move
                        // the violators; otherwise just lift the new ones up to it.
                        if (allAtLevel)
                            dummyPenalty *= DummyCostGrowth;

                        var addIncumbent = new List<string>();
                        foreach (var (_, pathIds) in incumbent.SelectedPaths)
                        {
                            foreach (var pid in pathIds)
                            {
                                if (pid.StartsWith("dummy[", StringComparison.Ordinal))
                                    pathdict[pid].Cost = Math.Max(pathdict[pid].Cost, dummyPenalty);
                                addIncumbent.Add(pid);
                            }
                        }

                        // Reset the LP master to exactly the incumbent's columns: remove
                        // every current column, then re-add the incumbent's selected
                        // paths (the dummies among them now carry the inflated cost).
                        var removeAll = new List<string>();
                        foreach (var ((_, pathId), _) in master.X)
                            removeAll.Add(pathId);

                        master.UpdatePaths(addIncumbent, removeAll, pathdict);
                        continue;
                    }

                    if (toRemove.Count == 0)
                        break;
                }

                master.UpdatePaths(toAdd, toRemove, pathdict);
            }
            phase1Sw.Stop();
            LogP1($"  Phase 1 run time: {phase1Sw.Elapsed.TotalSeconds:F1} s.");

            if (rootRelaxationOnly && !phase1LpFeasible)
            {
                LogP1("  Phase 1 did not reach a zero LP objective; skipping Phase 2.");
                return null;
            }

            // Phase 1 -> Phase 2 handoff runs once. Each policy below restarts from the
            // same snapshot of the column set, so every policy sees identical columns.
            var phase2PrepLog = new StringBuilder();
            var phase2PrepSw = Stopwatch.StartNew();
            var phase2Feasible = PreparePhase2(
                network, requests, pathdict, sigToId, env, feasible, ref columnCounter,
                mode, phase2PrepLog, rootRelaxationOnly, phase1Basis);
            phase2PrepSw.Stop();
            var phase2Start = new Phase2StartState(requests, pathdict, sigToId, columnCounter);

            var phase2Results = new List<(int Policy, IntegerMasterSolution? Best, double TimeToBestMs, double DualBound, Dictionary<string, double> ArcValues, BranchingArc? Candidate)>();
            foreach (var policy in policies)
            {
                if (phase2Results.Count > 0)
                    phase2Start.Restore(pathdict, sigToId, ref columnCounter);

                // A single policy keeps the original output names; several policies each
                // write their own files.
                var suffix = policies.Count == 1 ? "" : $"_dual_lb{policy}";
                var runTag = policies.Count == 1 ? "" : $" (dual-lb {policy})";
                var runLog = new StringBuilder(phase2PrepLog.ToString());
                var runCsv = new StringBuilder();
                runCsv.AppendLine(CsvHeader);
                phase2Runs.Add((suffix, runLog, runCsv));

                var runSw = Stopwatch.StartNew();
                var best = RunPhase2(
                    network, requests, pathdict, sigToId, env, phase2Feasible, ref columnCounter,
                    mode, maxIterations, numPaths, randomizedPricing, policy, integerMasterInterval, phase2OptimalityTol,
                    runLog, runCsv,
                    out var phase2TimeToBestSeconds, out var bestDualBound,
                    out var finalArcValues, out var candidate, out phase2Master, rootRelaxationOnly, legReqIdleLimit,
                    hybridSwitchTolerance, hybridSwitchStallIterations, columnIdleLimit);
                // Phase 2 time includes the shared handoff, as it did before policies were swept.
                var phase2Seconds = phase2PrepSw.Elapsed.TotalSeconds + runSw.Elapsed.TotalSeconds;
                runLog.AppendLine($"  Phase 2 run time{runTag}: {phase2Seconds:F1} s.");
                Console.WriteLine($"  Phase 2 run time{runTag}: {phase2Seconds:F1} s.");

                // Record per-phase run times so callers can report/compare them.
                if (!string.IsNullOrEmpty(outputDir))
                {
                    try
                    {
                        Directory.CreateDirectory(outputDir);
                        File.WriteAllText(
                            Path.Combine(outputDir, $"cg_phase_times{suffix}.csv"),
                            $"phase1_s,phase2_s,root_relaxation_only{Environment.NewLine}" +
                            $"{phase1Sw.Elapsed.TotalSeconds:F1},{phase2Seconds:F1},{(rootRelaxationOnly ? 1 : 0)}{Environment.NewLine}");
                    }
                    catch (IOException ex)
                    {
                        Console.Error.WriteLine($"Failed to write CG phase times to '{outputDir}': {ex.Message}");
                    }
                }

                // Depth-first branch-and-price over the root's fractional arc.
                if (branchAndPrice && !rootRelaxationOnly && phase2Master is not null && branchDepth > 0)
                {
                    best = RunBranchAndPriceDfs(
                        phase2Master, network, requests, pathdict, sigToId, env, best, ref columnCounter,
                        mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, phase2OptimalityTol,
                        runLog, runCsv,
                        candidate, new HashSet<string>(StringComparer.Ordinal), depth: 0, maxDepth: branchDepth);
                }

                if (best is not null)
                    runLog.AppendLine($"  Best objective{runTag}: {best.Objective:F4}");

                phase2Master?.Dispose();
                phase2Master = null;
                phase2Results.Add((policy, best, phase2TimeToBestSeconds, bestDualBound, finalArcValues, candidate));
                if (policies.Count > 1)
                    Console.WriteLine($"  dual-lb {policy}: objective {(best is null ? "none" : best.Objective.ToString("F4"))}, dual bound {bestDualBound:F4}");
            }

            // Report the lowest-objective policy's solution. With one policy this is its result.
            var chosen = phase2Results[0];
            foreach (var result in phase2Results)
            {
                if (result.Best is not null && (chosen.Best is null || result.Best.Objective < chosen.Best.Objective))
                    chosen = result;
            }
            if (policies.Count > 1 && chosen.Best is not null)
                Console.WriteLine($"  Best dual-lb policy: {chosen.Policy}");

            cgArcValues = chosen.ArcValues;
            branchingCandidate = chosen.Candidate;
            cgTimeToBestMs = chosen.TimeToBestMs;
            cgDualBound = chosen.DualBound;
            return chosen.Best;
            }
            finally
            {
                phase2Master?.Dispose();
                master?.Dispose();
                env.Dispose(); // close the shared environment when done

                // Flush the captured phase logs to <outputDir>/phase1.log and phase2.log.
                if (!string.IsNullOrEmpty(outputDir))
                {
                    try
                    {
                        Directory.CreateDirectory(outputDir);
                        File.WriteAllText(Path.Combine(outputDir, "phase1.log"), phase1Log.ToString());
                        foreach (var (suffix, log, csv) in phase2Runs)
                        {
                            File.WriteAllText(Path.Combine(outputDir, $"phase2{suffix}.log"), log.ToString());
                            File.WriteAllText(Path.Combine(outputDir, $"phase2{suffix}.csv"), csv.ToString());
                        }
                        File.WriteAllText(Path.Combine(outputDir, "phase1.csv"), phase1Csv.ToString());
                    }
                    catch (IOException ex)
                    {
                        Console.Error.WriteLine($"Failed to write CG logs to '{outputDir}': {ex.Message}");
                    }
                }
            }
        }

        // Phase 1 -> Phase 2 handoff, run once per CGoptimize call. Keeps the final
        // Phase 1 LP basis (plus the warm-start incumbent's paths), strips the dummy
        // arcs, restores true costs, and seeds a feasible solution when Phase 1 did not
        // produce one. Its messages go to prepLog so every Phase 2 run can report them.
        private static IntegerMasterSolution? PreparePhase2(
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            IntegerMasterSolution? feasible,
            ref int columnCounter,
            CGMode mode,
            StringBuilder prepLog,
            bool rootRelaxationOnly,
            IReadOnlySet<string>? phase1Basis)
        {
            // Normal mode warm-starts from a dummy-free integer solution, restores
            // true costs, and runs Phase 2 with the real economics. Root-relaxation-only
            // mode skips the integer-feasibility fallbacks and solves only the LP master.
            if (feasible is null && !rootRelaxationOnly)
            {
                var allColumns = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                foreach (var req in requests.Values)
                {
                    if (req.SubNetwork is null)
                        continue;
                    allColumns[req.Id] = new List<string>(req.Paths);
                }

                var finalAttempt = SolveIntegerMaster(allColumns, pathdict, network, env, includeArcFixedCost: false, mode: mode);
                if (finalAttempt is not null && UsesNoDummy(finalAttempt))
                    feasible = finalAttempt;
            }

            // Carry over only the final Phase 1 LP basis (plus the warm-start
            // incumbent's selected paths) instead of every column generated so far.
            static int RealColumnCount(Dictionary<string, Request> reqs)
                => reqs.Values.Sum(r => r.Paths.Count(p => !p.StartsWith("dummy[", StringComparison.Ordinal)));
            var realBefore = RealColumnCount(requests);
            if (phase1Basis is not null)
                PruneToPhase1Basis(requests, pathdict, sigToId, phase1Basis, feasible);

            // Strip the artificial dummy arcs / columns: pricing and the master now
            // operate on the real network only.
            RemoveDummies(network, requests, pathdict);

            if (phase1Basis is not null)
            {
                var uncovered = requests.Values.Count(r => r.SubNetwork is not null && r.Paths.Count == 0);
                var msg = $"  Phase 1 -> Phase 2: kept {RealColumnCount(requests)} of {realBefore} real columns (final LP basis + incumbent paths; dummies removed)"
                    + (uncovered > 0 ? $"; WARNING: {uncovered} request(s) have no column." : ".");
                Console.WriteLine(msg);
                prepLog.AppendLine(msg);
            }

            // Restore every surviving (non-dummy) column to its true cost.
            foreach (var rp in pathdict.Values)
                rp.Cost = rp.RawCost;

            // Phase 1 fallback: column generation never produced a dummy-free feasible
            // solution. Solve the arc-based MIP in feasibility-only mode on the real
            // (dummy-free) network, turn its per-request routing into seed columns, and
            // use the resulting integer solution to initialize Phase 2.
            if (feasible is null && !rootRelaxationOnly)
                feasible = SeedPhase2FromArcMip(
                    network, requests, pathdict, sigToId, env, ref columnCounter, mode);

            return feasible;
        }

        // Column-generation optimality phase for one dual-lb policy: builds a fresh
        // master on the column set left by PreparePhase2 and runs RunCGLoop on it.
        // Ownership of the master is handed to the caller so branch-and-price can keep
        // branching on it.
        // Start Phase 2 pricing with the SP-CSS heuristic before the MIP pricer.
        public static bool CssHeuristicEnabled { get; set; } = true;

        private static IntegerMasterSolution? RunPhase2(
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            IntegerMasterSolution? feasible,
            ref int columnCounter,
            CGMode mode,
            int maxIterations,
            int numPaths,
            bool randomizedPricing,
            int enable_dual_lb,
            int integerMasterInterval,
            double optimalityTol,
            StringBuilder phase2Log,
            StringBuilder phase2Csv,
            out double phase2TimeToBestMs,
            out double dualBoundResult,
            out Dictionary<string, double> finalArcValues,
            out BranchingArc? branchingCandidate,
            out CGMaster? finalMaster,
            bool rootRelaxationOnly = false,
            int legReqIdleLimit = 2,
            double hybridSwitchTolerance = 0.01,
            int hybridSwitchStallIterations = 5,
            int columnIdleLimit = 0)
        {
            phase2TimeToBestMs = double.NaN;
            dualBoundResult = double.NegativeInfinity;
            finalArcValues = new Dictionary<string, double>(StringComparer.Ordinal);
            branchingCandidate = null;
            finalMaster = null;

            if (feasible is null && !rootRelaxationOnly)
                return null; // never found a feasible solution without dummy arcs

            // Fresh master that charges the real arc fixed cost on y.
            var master2 = new CGMaster(env, requests, pathdict, network, includeArcFixedCost: true, enable_dual_lb: enable_dual_lb, mode: mode, legReqIdleLimit: legReqIdleLimit, hybridSwitchTolerance: hybridSwitchTolerance, hybridSwitchStallIterations: hybridSwitchStallIterations, columnIdleLimit: columnIdleLimit);
            finalMaster = master2;

            return RunCGLoop(
                master2, network, requests, pathdict, sigToId, env, feasible, ref columnCounter,
                mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                phase2Log, phase2Csv, "Phase 2 (optimality)",
                out phase2TimeToBestMs, out dualBoundResult, out finalArcValues, out branchingCandidate, isRootNode: true, rootRelaxationOnly: rootRelaxationOnly);
        }

        // Column-generation loop over an EXISTING master: alternates LP master,
        // integer master and pricing until the LP/dual-bound gap closes, unless
        // root-relaxation-only mode requests LP column generation without integer solves.
        //
        // forbiddenArcs is enforced in pricing only; the integer master stays
        // bound-free and is used purely to harvest globally feasible incumbents.
        //
        // isRootNode forces an integer master solve at iteration 0 (start of the
        // root node); other nodes only run it per integerMasterInterval as usual.
        private static IntegerMasterSolution? RunCGLoop(
            CGMaster master2,
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            IntegerMasterSolution? warmStart,
            ref int columnCounter,
            CGMode mode,
            int maxIterations,
            int numPaths,
            bool randomizedPricing,
            int integerMasterInterval,
            double optimalityTol,
            StringBuilder phase2Log,
            StringBuilder phase2Csv,
            string label,
            out double phase2TimeToBestMs,
            out double dualBoundResult,
            out Dictionary<string, double> finalArcValues,
            out BranchingArc? branchingCandidate,
            bool isRootNode = true,
            IReadOnlySet<string>? forbiddenArcs = null,
            bool rootRelaxationOnly = false)
        {
            const double RcTol = 1e-8;
            phase2TimeToBestMs = double.NaN;
            dualBoundResult = double.NegativeInfinity;
            finalArcValues = new Dictionary<string, double>(StringComparer.Ordinal);
            branchingCandidate = null;

            // Mirror a log line to the console and the phase 2 log buffer.
            void LogP2(string line)
            {
                Console.WriteLine(line);
                phase2Log.AppendLine(line);
            }

            // Seeded from warmStart so the log's incumbent column reflects the
            // carried-over incumbent immediately, not only once this node improves it.
            // warmStart.Objective may come from a different cost basis (e.g. Phase 1's
            // dummy-penalty objective), so recompute it under the real path/arc costs.
            IntegerMasterSolution? best = warmStart is null
                ? null
                : warmStart with { Objective = TrueObjective(warmStart.SelectedPaths, pathdict, network) };
            // Previous integer incumbent, seeded with the Phase 1 (dummy-free) solution.
            // Its selected paths plus the current LP support form the next integer
            // master's column pool instead` of every column generated so far.
            IntegerMasterSolution? lastIncumbent2 = warmStart;

            // Best (largest) Lagrangian dual bound found across Phase 2 iterations.
            var bestDualBound = double.NegativeInfinity;

            // Sliding window of the incumbent objective: holds the best objective
            // from the last StallWindow iterations so the current incumbent can be
            // compared against the one StallWindow iterations ago. Phase 2 stops once
            // that relative improvement drops below StallTolerance (0.1%).
            const int StallWindow = 5;
            const double StallTolerance = 0.001;
            var recentBest = new Queue<double>();

            // Hybrid dual-lb policy 2 (dual bounds) / policy 4 (master LP objectives) of the most recent
            // iterations, oldest first, capped at HybridSwitchStallIterations.
            var recentDualBounds = new Queue<double>();
            // Policy 3 keeps only observations from the current policy/pricer.
            var recentHybridBounds = new Queue<double>();

            var phase2Sw = Stopwatch.StartNew();
            // var subproblemAddedPaths = new Dictionary<string, List<string>>(StringComparer.Ordinal);

            var latestIntegerMasterPool = new Dictionary<string, List<string>>(StringComparer.Ordinal);
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is not null)
                    latestIntegerMasterPool[req.Id] = new List<string>(req.Paths);
            }
            var latestIntegerMasterXValues = new Dictionary<(string Request, string Path), double>();
            var lastIterationRanIntegerMaster = false;
            var phase2TimeToBestSeconds = double.NaN;

            // The integer master is a primal heuristic over the whole column pool: it
            // carries no branch bounds, so any solution it returns is a valid global
            // incumbent and a request can never be left without a selectable column.
            double SolvePhase2IntegerMaster(
                Dictionary<string, List<string>> pool,
                Dictionary<(string Request, string Path), double> xValues)
            {
                var intSw = Stopwatch.StartNew();
                var incumbent = SolveIntegerMaster(
                    pool,
                    pathdict,
                    network,
                    env,
                    includeArcFixedCost: true,
                    masterXValues: xValues,
                    sequentialAdd: true,
                    guaranteedPaths: lastIncumbent2?.SelectedPaths,
                    mode: mode);
                intSw.Stop();

                if (incumbent is not null)
                {
                    // The MIP charges fixed cost for every y[a] = 1, even on arcs no
                    // selected path uses; re-base so it matches the warm start's basis.
                    incumbent = incumbent with
                    {
                        Objective = TrueObjective(incumbent.SelectedPaths, pathdict, network),
                    };

                    lastIncumbent2 = incumbent;
                    if (best is null || incumbent.Objective < best.Objective)
                    {
                        best = incumbent;
                        phase2TimeToBestSeconds = phase2Sw.Elapsed.TotalSeconds;
                    }
                }

                return intSw.Elapsed.TotalSeconds;
            }

            double totalminvariablecost = findshortestpaths(requests);
            

            LogP2("");
            LogP2($"{label}:");
            LogP2($"  {"iter",4} | {"master LP",14} | {"incumbent",14} | {"dual bound",14} | {"LP s",10} | {"int s",10} | {"sub s",10}");

            var useHeuristicPricing = CssHeuristicEnabled;
            for (var iter = 0; iter < maxIterations; iter++)
            {
                lastIterationRanIntegerMaster = false;

                // 1) Solve the LP master and refresh the real duals.
                var lpSw = Stopwatch.StartNew();
                master2.Optimize();
                lpSw.Stop();
                if (master2.Status != GRB.Status.OPTIMAL)
                    break;

                var masterObj = master2.Model.ObjVal;

                // Snapshot this LP master's arc-activation (y[a]) values. Overwritten
                // each iteration so it holds the final LP solution once CG stops.
                finalArcValues.Clear();
                foreach (var (arcId, yVar) in master2.Y)
                    finalArcValues[arcId] = yVar.X;

                // 2) Columns with non-zero x in the current LP master solution.
                var lpSupport = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                var cgXValues = new Dictionary<(string Request, string Path), double>();
                foreach (var ((reqId, pathId), xVar) in master2.X)
                {
                    cgXValues[(reqId, pathId)] = xVar.X;

                    if (xVar.X >= 0.0)
                    {
                        if (!lpSupport.TryGetValue(reqId, out var list))
                        {
                            list = new List<string>();
                            lpSupport[reqId] = list;
                        }
                        list.Add(pathId);
                    }
                }

                // Integer incumbent over the previous incumbent's paths plus the
                // current LP support, solved on every iteration.
                var pool = new Dictionary<string, List<string>>(StringComparer.Ordinal);
                foreach (var req in requests.Values)
                {
                    if (req.SubNetwork is null)
                        continue;

                    var set = new HashSet<string>(StringComparer.Ordinal);
                    if (lpSupport.TryGetValue(req.Id, out var a))
                        set.UnionWith(a);
                    // if (subproblemAddedPaths.TryGetValue(req.Id, out var added))
                    //     set.UnionWith(added);
                    if (lastIncumbent2 is not null &&
                        lastIncumbent2.SelectedPaths.TryGetValue(req.Id, out var prev))
                        set.UnionWith(prev);
                    pool[req.Id] = new List<string>(set);
                }

                latestIntegerMasterPool = pool;
                latestIntegerMasterXValues = cgXValues;
                var intSeconds = 0d;
                if (!rootRelaxationOnly &&
                    ((iter + 1) % integerMasterInterval == 0 || iter == maxIterations - 1 || (iter == 0 && isRootNode)))
                {
                    intSeconds = SolvePhase2IntegerMaster(pool, cgXValues);
                    lastIterationRanIntegerMaster = true;
                }
                

                // 3) Price real columns (full fixed + variable arc cost) and add the
                //    ones with strictly negative reduced cost. The reduced cost of a
                //    priced path is TotalCost - sigma_r (convexity dual).
                var subSw = Stopwatch.StartNew();
                var priced = master2.SolveSubproblem(requests, numPaths, includeArcCost: true, randomizedPricing: randomizedPricing, forbiddenArcs: forbiddenArcs, heuristic: useHeuristicPricing, warmStartMip: CssHeuristicEnabled);
                subSw.Stop();

                // Lagrangian dual bound: the master LP objective plus, for every
                // request, the most negative reduced cost among its priced paths,
                // i.e. min(bestPathReducedCost, 0). A request's best-path reduced
                // cost is sp.TotalCost - sigma_r (its convexity dual). The bound is a
                // valid lower bound on the optimum; keep the largest one seen so far.
                // var dualBound = master2.Model.ObjVal;
                var dualBound = 0.0;
                // Branch bound rows contribute rhs * dual, so a node with y[a] fixed
                // to 1 is accounted for instead of assuming the root's y[a] <= 1.
                dualBound += master2.ArcBoundDualContribution();
                foreach(string key in master2.ConvexityDuals.Keys)
                {
                    dualBound = dualBound + master2.ConvexityDuals[key];
                }
                foreach (var (reqId, paths) in priced)
                {
                    if (paths.Count == 0)
                        continue;

                    // The dual bound must use the ACTUAL master duals, not the
                    // stabilized combination that priced the paths. Recompute each
                    // found path's reduced cost under the actual duals and take the
                    // most negative (best) one for this request.
                    var req = requests[reqId];
                    var reducedCost = double.PositiveInfinity;
                    foreach (var sp in paths)
                    {
                        // True reduced cost under the actual duals: each resource's
                        // mu_{r,s} is charged once even if several CSS labels of the
                        // same resource were concatenated on the path.
                        var rc = master2.ActualReducedCost(req, sp.Arcs, includeArcCost: true);
                        if (rc < reducedCost)
                            reducedCost = rc;
                    }
                    dualBound += Math.Min(reducedCost, 0d);
                }

                var hybridPolicySwitched = false;

                // Hybrid dual-lb policy 2: compare this iteration's dual bound with the
                // one from HybridSwitchStallIterations iterations ago; if it has not
                // improved by the relative HybridSwitchTolerance, switch from policy 0
                // to policy 1. With the SP-CSS heuristic enabled, the comparison runs
                // only while heuristic pricing is active; after the hand-off to MIP
                // pricing the current dual-lb setting is kept as is.
                if (master2.EnableDualLb == 2 && master2.HybridDualLbPending && (useHeuristicPricing || !CssHeuristicEnabled))
                {
                    if (recentDualBounds.Count == master2.HybridSwitchStallIterations)
                    {
                        var pastDualBound = recentDualBounds.Dequeue();
                        var dualImprovement = (dualBound - pastDualBound) / Math.Max(Math.Abs(pastDualBound), RcTol);
                        if (dualImprovement < master2.HybridSwitchTolerance && master2.SwitchHybridDualLb())
                            LogP2($"  >> {label}: dual-lb policy switched 0 -> 1 at iter {iter} " +
                                  $"(dual bound {pastDualBound:F4} -> {dualBound:F4} over the last {master2.HybridSwitchStallIterations} iterations, " +
                                  $"improvement {dualImprovement:P3} < {master2.HybridSwitchTolerance:P3}).");
                    }
                    recentDualBounds.Enqueue(dualBound);
                }

                // Reverse hybrid dual-lb policy 4: start with policy 1 and switch to
                // policy 0 once the master LP objective (upper bound) has not decreased by
                // the relative HybridSwitchTolerance over the last HybridSwitchStallIterations
                // iterations. Gated on heuristic pricing the same way as policy 2.
                if (master2.EnableDualLb == 4 && master2.HybridDualLbPending && (useHeuristicPricing || !CssHeuristicEnabled))
                {
                    if (recentDualBounds.Count == master2.HybridSwitchStallIterations)
                    {
                        var pastMasterObj = recentDualBounds.Dequeue();
                        var objDecrease = (pastMasterObj - masterObj) / Math.Max(Math.Abs(pastMasterObj), RcTol);
                        if (objDecrease < master2.HybridSwitchTolerance && master2.SwitchHybridDualLb())
                        {
                            hybridPolicySwitched = true;
                            LogP2($"  >> {label}: dual-lb policy switched 1 -> 0 at iter {iter} " +
                                  $"(master LP {pastMasterObj:F4} -> {masterObj:F4} over the last {master2.HybridSwitchStallIterations} iterations, " +
                                  $"decrease {objDecrease:P3} < {master2.HybridSwitchTolerance:P3}).");
                        }
                    }
                    recentDualBounds.Enqueue(masterObj);
                }

                if (master2.EnableDualLb == 3)
                {
                    var policy = master2.EffectiveDualLbPolicy;
                    var bound = policy == 0 ? dualBound : masterObj;
                    if (recentHybridBounds.Count == master2.HybridSwitchStallIterations)
                    {
                        var pastBound = recentHybridBounds.Dequeue();
                        var improvement = (policy == 0 ? bound - pastBound : pastBound - bound)
                            / Math.Max(Math.Abs(pastBound), RcTol);
                        if (improvement < master2.HybridSwitchTolerance && master2.SwitchHybridDualLb())
                        {
                            hybridPolicySwitched = true;
                            recentHybridBounds.Clear();
                            LogP2($"  >> {label}: dual-lb policy switched {policy} -> {master2.EffectiveDualLbPolicy} at iter {iter} " +
                                  $"({(policy == 0 ? "dual bound" : "master LP")} {pastBound:F4} -> {bound:F4} over the last {master2.HybridSwitchStallIterations} iterations, " +
                                  $"improvement {improvement:P3} < {master2.HybridSwitchTolerance:P3}).");
                        }
                    }
                    if (!hybridPolicySwitched)
                        recentHybridBounds.Enqueue(bound);
                }

                // Under SP-CSS heuristic pricing the bound is only an upper estimate of
                // the true Lagrangian bound: use it to decide when to switch to the
                // MIP pricer, but never record it as a valid bound.
                if (useHeuristicPricing)
                {
                    var hGap = Math.Abs(masterObj) > RcTol
                        ? Math.Abs(masterObj - dualBound) / Math.Abs(masterObj)
                        : 0d;
                    var hasColumn = priced.Values.Any(ps => ps.Count > 0);
                    LogP2($"  {iter,4} | {masterObj,14:F4} | {"(heur)",14} | {dualBound,14:F4} | {lpSw.Elapsed.TotalSeconds,10:F1} | {intSeconds,10:F1} | {subSw.Elapsed.TotalSeconds,10:F1}");
                    phase2Csv.AppendLine($"{iter},{masterObj:F4},heur,{dualBound:F4},{lpSw.Elapsed.TotalSeconds:F1},{intSeconds:F1},{subSw.Elapsed.TotalSeconds:F1}");
                    if (!hasColumn || hGap < optimalityTol)
                    {
                        useHeuristicPricing = false;
                        recentHybridBounds.Clear();
                        LogP2($"  >> {label}: SP-CSS heuristic converged at iter {iter} " +
                              (hasColumn ? $"(gap {hGap:P3} < {optimalityTol:P3})" : "(no negative column)") +
                              "; switching to MIP pricing.");
                        continue; // re-price the same master with the MIP
                    }
                }

                // A new best (largest) dual bound: record it and snapshot the actual
                // duals that produced it as the stable centre for stabilized pricing.
                if (!useHeuristicPricing && dualBound > bestDualBound)
                {
                    bestDualBound = dualBound;
                    master2.SaveStableDuals();
                }

                var bestStr = best is null ? "-" : best.Objective.ToString("F4");
                if (!useHeuristicPricing)
                {
                LogP2($"  {iter,4} | {master2.Model.ObjVal,14:F4} | {bestStr,14} | {bestDualBound,14:F4} | {lpSw.Elapsed.TotalSeconds,10:F1} | {intSeconds,10:F1} | {subSw.Elapsed.TotalSeconds,10:F1}");
                phase2Csv.AppendLine($"{iter},{master2.Model.ObjVal:F4},{bestStr},{bestDualBound:F4},{lpSw.Elapsed.TotalSeconds:F1},{intSeconds:F1},{subSw.Elapsed.TotalSeconds:F1}");
                }

                // Relative gap between the master LP objective and the best Lagrangian
                // dual bound. This criterion is purely LP-based, so it must be checked
                // independently of the integer incumbent: in root-relaxation-only mode
                // the integer master never runs and `best` stays null, which would
                // otherwise leave column generation running to maxIterations.
                if (!useHeuristicPricing && !double.IsNegativeInfinity(bestDualBound) && Math.Abs(master2.Model.ObjVal) > RcTol)
                if (!double.IsNegativeInfinity(bestDualBound) && Math.Abs(master2.Model.ObjVal) > RcTol)
                {
                    var lpgap = Math.Abs(master2.Model.ObjVal - bestDualBound) / Math.Abs(master2.Model.ObjVal);
                    if (lpgap < optimalityTol)
                    {
                        LogP2($" Stopping CG gap {lpgap:P3} < {optimalityTol:P3}.");
                        break;
                    }
                }

                // Stop if the incumbent improved by less than StallTolerance (0.1%)
                // over the last StallWindow iterations. The queue front is the best
                // objective from StallWindow iterations ago once it is full.
                if (best is not null)
                {
                    recentBest.Enqueue(best.Objective);
                    // if (recentBest.Count > StallWindow)
                    // {
                    //     var older = recentBest.Dequeue();
                    //     var improvement = Math.Abs(older) > RcTol
                    //         ? (older - best.Objective) / Math.Abs(older)
                    //         : double.PositiveInfinity;
                    //     if (improvement < StallTolerance)
                    //     {
                    //         LogP2(
                    //             $"  Stopping: incumbent improved < {StallTolerance:P1} over {StallWindow} iterations.");
                    //         break;
                    //     }
                    // }
                }

                var toAdd = new List<string>();
                
                foreach (var (reqId, paths) in priced)
                {
                    // sp.TotalCost is the stabilized path weight, so compare it
                    // against the stabilized convex dual for a consistent reduced cost.
                    var sigma = master2.CombinedConvexDual(reqId);

                    foreach (var sp in paths)
                    {
                        if (sp.TotalCost >= sigma - RcTol)
                            continue; // not improving

                        if (!sigToId.TryGetValue(reqId, out var map))
                        {
                            map = new Dictionary<string, string>(StringComparer.Ordinal);
                            sigToId[reqId] = map;
                        }

                        var sig = Signature(sp.Arcs);
                        if (map.TryGetValue(sig, out var existingPid))
                        {
                            if (!master2.X.ContainsKey((reqId, existingPid)))
                                toAdd.Add(existingPid);
                            continue;
                        }

                        var req = requests[reqId];
                        var newId = $"p[{reqId}]#{columnCounter++}";
                        var col = new RoutePath(newId, req, sp.Arcs);
                        col.CalculateCost(); // RawCost = Cost = true cost

                        pathdict[newId] = col;
                        req.Paths.Add(newId);
                        map[sig] = newId;
                        toAdd.Add(newId);
                    }
                }

                // subproblemAddedPaths.Clear();
                // foreach (var pid in toAdd)
                // {
                //     var reqId = pathdict[pid].Request.Id;
                //     if (!subproblemAddedPaths.TryGetValue(reqId, out var list))
                //     {
                //         list = new List<string>();
                //         subproblemAddedPaths[reqId] = list;
                //     }
                //     list.Add(pid);
                // }

                // 4) No improving column anywhere -> CG has converged to optimality.
                if (toAdd.Count == 0 && useHeuristicPricing)
                {
                    useHeuristicPricing = false;
                    recentHybridBounds.Clear();
                    LogP2($"  >> {label}: SP-CSS heuristic added no column at iter {iter}; switching to MIP pricing.");
                    continue;
                }
                if (toAdd.Count == 0)
                {
                    if (hybridPolicySwitched)
                        continue; // Price once under the new policy before declaring convergence.
                    Console.WriteLine("  No improving column found; Phase 2 has converged to optimality.");
                    break;
                }
                    
                // Remove columns unused for more than columnIdleLimit consecutive rounds.
                var toRemove = master2.CollectIdleColumns(1e-8, keep: toAdd.Contains);
                master2.UpdatePaths(toAdd, toRemove, pathdict);
            }

            if (!rootRelaxationOnly && !lastIterationRanIntegerMaster)
            {
                Console.WriteLine("  Running final integer master solve.");
                SolvePhase2IntegerMaster(latestIntegerMasterPool, latestIntegerMasterXValues);
            }

            // Pick the branching candidate from the final LP master's y[a] values:
            // the most fractional arc (closest to 0.5), tie-broken by highest cost.
            branchingCandidate = SelectBranchingArc(finalArcValues, network);
            if (branchingCandidate is not null)
                LogP2($"  Branching candidate: y[{branchingCandidate.ArcId}] = {branchingCandidate.Value:F4} " +
                      $"(|y-0.5| = {branchingCandidate.DistanceToHalf:F4}, cost = {branchingCandidate.Cost:F4}).");

            phase2TimeToBestMs = phase2TimeToBestSeconds;
            dualBoundResult = bestDualBound;
            return best;
        }

        // Creates one branch-and-price child by fixing the branching arc's
        // activation variable (direction 1 -> y[a] = 1, direction 0 -> y[a] = 0) in
        // the LP master and re-solving it. The re-solve yields the child's starting
        // basis for column generation and its LP lower bound. The parent's bounds
        // are restored only by the caller (not here), so the master is left sitting
        // at the child node.
        private static BranchNode BranchOnArc(
            CGMaster master, string arcId, int direction, StringBuilder log)
        {
            var fixedValue = direction == 1 ? 1d : 0d;
            master.SetResourceBounds(arcId, fixedValue, fixedValue);
            master.Optimize();

            var feasible = master.Status == GRB.Status.OPTIMAL;
            var bound = feasible ? master.Model.ObjVal : double.PositiveInfinity;

            var line = feasible
                ? $"  Branch y[{arcId}] = {fixedValue:F0}: master LP = {bound:F4}"
                : $"  Branch y[{arcId}] = {fixedValue:F0}: master LP infeasible (status {master.Status}).";
            Console.WriteLine(line);
            log.AppendLine(line);

            return new BranchNode(arcId, direction, bound, feasible);
        }

        // Depth-first branch-and-price. At each level both children of the current
        // fractional arc are solved, then the one with the SMALLER final dual lower
        // bound is re-entered and branched again on its own fractional arc, until
        // maxDepth levels have been branched. Only that one child is explored deeper
        // (no open-node pool), so the recursion is a single root-to-leaf dive.
        private static IntegerMasterSolution? RunBranchAndPriceDfs(
            CGMaster master,
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            IntegerMasterSolution? incumbent,
            ref int columnCounter,
            CGMode mode,
            int maxIterations,
            int numPaths,
            bool randomizedPricing,
            int integerMasterInterval,
            double optimalityTol,
            StringBuilder phase2Log,
            StringBuilder phase2Csv,
            BranchingArc? candidate,
            IReadOnlySet<string> forbiddenArcs,
            int depth,
            int maxDepth)
        {
            var best = incumbent;
            if (candidate is null || depth >= maxDepth)
                return best;

            void Log(string line)
            {
                Console.WriteLine(line);
                phase2Log.AppendLine(line);
            }

            var arcId = candidate.ArcId;
            Log("");
            Log($"[depth {depth}] Branching on y[{arcId}] = {candidate.Value:F4}.");

            var open = SolveBranchChild(
                master, network, requests, pathdict, sigToId, env, best, ref columnCounter,
                mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                phase2Log, phase2Csv, arcId, 1, forbiddenArcs, depth, maxDepth, explore: false);
            if (open.Best is not null && (best is null || open.Best.Objective < best.Objective))
                best = open.Best;

            var closed = SolveBranchChild(
                master, network, requests, pathdict, sigToId, env, best, ref columnCounter,
                mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                phase2Log, phase2Csv, arcId, 0, forbiddenArcs, depth, maxDepth, explore: false);
            if (closed.Best is not null && (best is null || closed.Best.Objective < best.Objective))
                best = closed.Best;

            if (depth + 1 >= maxDepth)
                return best;

            int direction;
            if (open.Pruned && closed.Pruned)
                return best;
            else if (open.Pruned)
                direction = 0;
            else if (closed.Pruned)
                direction = 1;
            else
                direction = open.DualBound <= closed.DualBound ? 1 : 0;

            var chosen = direction == 1 ? open : closed;
            if (chosen.Candidate is null)
            {
                Log($"[depth {depth}] y[{arcId}] = {direction} has no fractional arc left; stopping this dive.");
                return best;
            }

            Log($"[depth {depth}] Diving into y[{arcId}] = {direction} " +
                $"(dual bounds: open = {open.DualBound:F4}, closed = {closed.DualBound:F4}).");

            // The dive re-solves the chosen child so its bounds / purged columns are in
            // place while the deeper levels branch; column generation there restarts
            // from the columns it already produced, so the re-solve converges quickly.
            var deeper = SolveBranchChild(
                master, network, requests, pathdict, sigToId, env, best, ref columnCounter,
                mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                phase2Log, phase2Csv, arcId, direction, forbiddenArcs, depth, maxDepth, explore: true);
            if (deeper.Best is not null && (best is null || deeper.Best.Objective < best.Objective))
                best = deeper.Best;

            return best;
        }

        // One branch-and-price child. direction 1 just fixes y[a] = 1; direction 0
        // additionally prunes when a request is disconnected without arc a, purges the
        // master's columns through a, and runs a node Phase 1 when the purged LP is
        // infeasible. Pricing bans every arc closed on the path from the root. Bounds
        // and purged columns are restored on exit. With explore set, the DFS recurses
        // one level deeper while this node is still installed on the master.
        private static BranchChildResult SolveBranchChild(
            CGMaster master,
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            IntegerMasterSolution? incumbent,
            ref int columnCounter,
            CGMode mode,
            int maxIterations,
            int numPaths,
            bool randomizedPricing,
            int integerMasterInterval,
            double optimalityTol,
            StringBuilder phase2Log,
            StringBuilder phase2Csv,
            string arcId,
            int direction,
            IReadOnlySet<string> forbiddenArcs,
            int depth,
            int maxDepth,
            bool explore)
        {
            var fixedValue = direction == 1 ? 1d : 0d;
            var label = $"[depth {depth}] Branch y[{arcId}] = {direction}";

            var nodeForbidden = new HashSet<string>(forbiddenArcs, StringComparer.Ordinal);
            // arcId is a resource id: closing y[s] forbids every arc of s.
            if (direction == 0 && network.TryGetResource(arcId, out var closedRes))
                foreach (var resArc in closedRes.Arcs)
                    nodeForbidden.Add(resArc.Id);

            void Log(string line)
            {
                Console.WriteLine(line);
                phase2Log.AppendLine(line);
            }

            if (direction == 0)
            {
                // Genuine infeasibility: some request cannot reach its target without a.
                foreach (var req in requests.Values)
                {
                    if (req.SubNetwork is null)
                        continue;
                    if (!IsReachable(req.SubNetwork.Network, req.Source, req.Target, nodeForbidden))
                    {
                        Log($"  {label}: request {req.Id} is disconnected without the arc; node pruned.");
                        return new BranchChildResult(null, double.PositiveInfinity, null, true);
                    }
                }
            }

            var saved = master.GetResourceBounds(arcId);
            var purged = new List<string>();
            if (direction == 0)
            {
                foreach (var (_, pathId) in master.X.Keys)
                {
                    if (UsesAnyArc(pathdict[pathId], nodeForbidden))
                        purged.Add(pathId);
                }
                master.UpdatePaths(new List<string>(), purged, pathdict);
                master.ClearStableDuals();
            }

            try
            {
                // A request left with no column makes the convexity row unsatisfiable,
                // so skip the doomed LP solve and go straight to Phase 1.
                var covered = new HashSet<string>(StringComparer.Ordinal);
                foreach (var (reqId, _) in master.X.Keys)
                    covered.Add(reqId);
                var needsPhase1 = requests.Values.Any(r => r.SubNetwork is not null && !covered.Contains(r.Id));

                if (needsPhase1)
                    master.SetResourceBounds(arcId, fixedValue, fixedValue);
                else
                    needsPhase1 = !BranchOnArc(master, arcId, direction, phase2Log).Feasible;

                if (needsPhase1 &&
                    !RunNodePhase1(master, requests, pathdict, sigToId, nodeForbidden, ref columnCounter, numPaths, label, Log))
                {
                    Log($"  {label}: Phase 1 LP optimum > 0 with no improving column; node pruned.");
                    return new BranchChildResult(null, double.PositiveInfinity, null, true);
                }

                var best = RunCGLoop(
                    master, network, requests, pathdict, sigToId, env, incumbent, ref columnCounter,
                    mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                    phase2Log, phase2Csv, label,
                    out _, out var dualBound, out _, out var candidate,
                    isRootNode: false,
                    forbiddenArcs: nodeForbidden.Count > 0 ? nodeForbidden : null);

                if (explore)
                {
                    var deeper = RunBranchAndPriceDfs(
                        master, network, requests, pathdict, sigToId, env, best, ref columnCounter,
                        mode, maxIterations, numPaths, randomizedPricing, integerMasterInterval, optimalityTol,
                        phase2Log, phase2Csv, candidate, nodeForbidden, depth + 1, maxDepth);
                    if (deeper is not null && (best is null || deeper.Objective < best.Objective))
                        best = deeper;
                }

                return new BranchChildResult(best, dualBound, candidate, false);
            }
            finally
            {
                master.SetResourceBounds(arcId, saved.Lower, saved.Upper);
                if (purged.Count > 0)
                    master.UpdatePaths(purged, new List<string>(), pathdict);
            }
        }

        // Node-level Phase 1 on the branch master: one arc-free artificial column per
        // request (cost 1, convexity row only), real columns and y[a] at cost 0, and
        // pricing with the forbidden arcs banned. Returns true once the LP drives all
        // artificials to 0; false if pricing finds no improving column while the LP
        // objective is still positive, which proves the node LP infeasible.
        private static bool RunNodePhase1(
            CGMaster master,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            IReadOnlySet<string> forbidden,
            ref int columnCounter,
            int numPaths,
            string label,
            Action<string> log)
        {
            const double FeasTol = 1e-6;
            const double RcTol = 1e-8;
            const int MaxIterations = 200;

            var artificial = new List<string>();
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is null)
                    continue;
                var id = $"artificial[{req.Id}]";
                pathdict[id] = new RoutePath(id, req, Array.Empty<Arc>()) { Cost = 1d };
                artificial.Add(id);
            }

            master.SetPhase1Objective(true, pathdict);
            master.UpdatePaths(artificial, new List<string>(), pathdict);

            log("");
            log($"{label} - Phase 1 (feasibility):");
            log($"  {"iter",4} | {"artificial",14} | {"added",8}");

            var feasible = false;
            try
            {
                for (var iter = 0; iter < MaxIterations; iter++)
                {
                    master.Optimize();
                    if (master.Status != GRB.Status.OPTIMAL)
                        break;

                    var infeasibility = master.Model.ObjVal;
                    if (infeasibility <= FeasTol)
                    {
                        log($"  {iter,4} | {infeasibility,14:F6} | {"-",8}");
                        feasible = true;
                        break;
                    }

                    var priced = master.SolveSubproblem(
                        requests, numPaths, includeArcCost: false, randomizedPricing: false,
                        forbiddenArcs: forbidden, originalNetworkHeuristic: true);

                    var toAdd = new List<string>();
                    foreach (var (reqId, paths) in priced)
                    {
                        var sigma = master.ConvexityDuals[reqId];
                        foreach (var sp in paths)
                        {
                            if (sp.TotalCost >= sigma - RcTol)
                                continue;

                            if (!sigToId.TryGetValue(reqId, out var map))
                            {
                                map = new Dictionary<string, string>(StringComparer.Ordinal);
                                sigToId[reqId] = map;
                            }

                            var sig = Signature(sp.Arcs);
                            if (map.TryGetValue(sig, out var existingPid))
                            {
                                if (!master.X.ContainsKey((reqId, existingPid)))
                                    toAdd.Add(existingPid);
                                continue;
                            }

                            var req = requests[reqId];
                            var newId = $"p[{reqId}]#{columnCounter++}";
                            var col = new RoutePath(newId, req, sp.Arcs);
                            col.CalculateCost();

                            pathdict[newId] = col;
                            req.Paths.Add(newId);
                            map[sig] = newId;
                            toAdd.Add(newId);
                        }
                    }

                    log($"  {iter,4} | {infeasibility,14:F6} | {toAdd.Count,8}");

                    if (toAdd.Count == 0)
                        break;

                    master.UpdatePaths(toAdd, new List<string>(), pathdict);
                }
            }
            finally
            {
                master.UpdatePaths(new List<string>(), artificial, pathdict);
                foreach (var id in artificial)
                    pathdict.Remove(id);
                master.SetPhase1Objective(false, pathdict);
                master.ClearStableDuals();
            }

            return feasible;
        }

        private static bool UsesAnyArc(RoutePath path, IReadOnlySet<string> arcIds)
        {
            foreach (var arc in path.Arcs)
            {
                if (arcIds.Contains(arc.Id))
                    return true;
            }
            return false;
        }

        private static bool IsReachable(Network network, string source, string target, IReadOnlySet<string> forbiddenArcIds)
        {
            var visited = new HashSet<string>(StringComparer.Ordinal) { source };
            var queue = new Queue<string>();
            queue.Enqueue(source);
            while (queue.Count > 0)
            {
                var node = queue.Dequeue();
                if (string.Equals(node, target, StringComparison.Ordinal))
                    return true;
                foreach (var arc in network.OutgoingFrom(node))
                {
                    if (!forbiddenArcIds.Contains(arc.Id) && visited.Add(arc.Target))
                        queue.Enqueue(arc.Target);
                }
            }
            return false;
        }

        // Selects the branching arc from a set of arc-activation (y[a]) LP values:
        // among all fractional arcs, the one closest to 0.5. Ties (equal distance to
        // 0.5) are broken toward the arc with the highest fixed cost.
        // Returns null when no such fractional arc exists.
        private static BranchingArc? SelectBranchingArc(
            IReadOnlyDictionary<string, double> arcValues, Network network)
        {
            const double IntegralTol = 1e-6;

            var arcCosts = new Dictionary<string, double>(StringComparer.Ordinal);
            // arcValues are y[s] values keyed by resource id; tie-break on sigma_s.
            foreach (var res in network.Resources)
                arcCosts[res.Id] = res.FixedCost;

            BranchingArc? best = null;
            foreach (var (arcId, value) in arcValues)
            {
                // Skip integral y[a] (0 or 1): nothing to branch on.
                var fractional = value - Math.Floor(value);
                if (fractional <= IntegralTol || fractional >= 1d - IntegralTol)
                    continue;

                var distance = Math.Abs(value - 0.5);
                var cost = arcCosts.GetValueOrDefault(arcId);

                if (best is null ||
                    distance < best.DistanceToHalf - IntegralTol ||
                    (Math.Abs(distance - best.DistanceToHalf) <= IntegralTol && cost > best.Cost))
                {
                    best = new BranchingArc(arcId, value, distance, cost);
                }
            }

            return best;
        }

        // Phase 1 fallback. Solves the arc-based MIP in feasibility-only mode (no
        // objective, stop at the first feasible routing) on the real, dummy-free
        // network, then converts each request's routing into a column and runs the
        // integer master over those seed columns to produce an initial dummy-free
        // solution for Phase 2. Returns null when even the arc MIP is infeasible.
        private static IntegerMasterSolution? SeedPhase2FromArcMip(
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            GRBEnv env,
            ref int columnCounter,
            CGMode mode = CGMode.Leg)
        {
            var arcSolution = SolveArcBasedModel(network, requests, out _, out _, feasibilityOnly: true);
            if (arcSolution is null)
                return null;

            // Seed pool: start from every existing column, then add the arc MIP's
            // per-request path so the integer master can reproduce its feasible routing.
            var pool = new Dictionary<string, List<string>>(StringComparer.Ordinal);
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is null)
                    continue;
                pool[req.Id] = new List<string>(req.Paths);
            }

            foreach (var (reqId, arcs) in arcSolution.RequestPaths)
            {
                if (arcs.Count == 0)
                    continue;
                if (!requests.TryGetValue(reqId, out var req))
                    continue;

                if (!sigToId.TryGetValue(reqId, out var map))
                {
                    map = new Dictionary<string, string>(StringComparer.Ordinal);
                    sigToId[reqId] = map;
                }

                // Reuse the column id if this exact path already exists.
                var sig = Signature(arcs);
                if (!map.TryGetValue(sig, out var pid))
                {
                    pid = $"p[{reqId}]#{columnCounter++}";
                    var col = new RoutePath(pid, req, arcs);
                    col.CalculateCost(); // RawCost = Cost = true cost
                    pathdict[pid] = col;
                    req.Paths.Add(pid);
                    map[sig] = pid;
                }

                if (!pool.TryGetValue(reqId, out var list))
                {
                    list = new List<string>();
                    pool[reqId] = list;
                }
                if (!list.Contains(pid))
                    list.Add(pid);
            }

            return SolveIntegerMaster(pool, pathdict, network, env, includeArcFixedCost: true, mode: mode);
        }

        // Path ids that are part of the master's current LP basis: basic columns plus
        // any column with x > 0 (e.g. nonbasic at its upper bound x = 1, which
        // carries a whole request and must not be dropped). Falls back to the LP
        // support (x > 0) if no simplex basis is available (e.g. barrier without
        // crossover).
        private static HashSet<string> CaptureBasicColumns(CGMaster master)
        {
            var basic = new HashSet<string>(StringComparer.Ordinal);
            try
            {
                foreach (var ((_, pathId), xVar) in master.X)
                {
                    if (xVar.VBasis == GRB.BASIC || xVar.X > 0)
                        basic.Add(pathId);
                }
            }
            catch (GRBException)
            {
                basic.Clear();
                foreach (var ((_, pathId), xVar) in master.X)
                {
                    if (xVar.X > 0)
                        basic.Add(pathId);
                }
            }
            return basic;
        }

        // Column set, costs and column counter at the Phase 1 -> Phase 2 handoff. Restoring
        // it lets each Phase 2 policy start from identical columns.
        private sealed class Phase2StartState
        {
            private readonly (Request Request, string[] PathIds)[] _requestPaths;
            private readonly Dictionary<string, RoutePath> _pathdict;
            private readonly (RoutePath Path, double Cost)[] _costs;
            private readonly Dictionary<string, Dictionary<string, string>> _sigToId;
            private readonly int _columnCounter;

            public Phase2StartState(
                Dictionary<string, Request> requests,
                Dictionary<string, RoutePath> pathdict,
                Dictionary<string, Dictionary<string, string>> sigToId,
                int columnCounter)
            {
                _requestPaths = requests.Values.Select(r => (Request: r, PathIds: r.Paths.ToArray())).ToArray();
                _pathdict = new Dictionary<string, RoutePath>(pathdict, StringComparer.Ordinal);
                _costs = pathdict.Values.Select(rp => (Path: rp, Cost: rp.Cost)).ToArray();
                _sigToId = sigToId.ToDictionary(
                    kv => kv.Key,
                    kv => new Dictionary<string, string>(kv.Value, StringComparer.Ordinal),
                    StringComparer.Ordinal);
                _columnCounter = columnCounter;
            }

            public void Restore(
                Dictionary<string, RoutePath> pathdict,
                Dictionary<string, Dictionary<string, string>> sigToId,
                ref int columnCounter)
            {
                foreach (var (request, pathIds) in _requestPaths)
                {
                    request.Paths.Clear();
                    request.Paths.AddRange(pathIds);
                }

                pathdict.Clear();
                foreach (var (id, rp) in _pathdict)
                    pathdict[id] = rp;

                foreach (var (rp, cost) in _costs)
                    rp.Cost = cost;

                sigToId.Clear();
                foreach (var (reqId, map) in _sigToId)
                    sigToId[reqId] = new Dictionary<string, string>(map, StringComparer.Ordinal);

                columnCounter = _columnCounter;
            }
        }

        // Drops every non-dummy column that is neither basic in the final Phase 1
        // LP nor selected by the warm-start incumbent from req.Paths, pathdict and
        // sigToId, so Phase 2 starts from that reduced column set. Dummy columns
        // are left to RemoveDummies.
        private static void PruneToPhase1Basis(
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict,
            Dictionary<string, Dictionary<string, string>> sigToId,
            IReadOnlySet<string> phase1Basis,
            IntegerMasterSolution? incumbent)
        {
            var keep = new HashSet<string>(phase1Basis, StringComparer.Ordinal);
            if (incumbent is not null)
            {
                foreach (var (_, pathIds) in incumbent.SelectedPaths)
                    keep.UnionWith(pathIds);
            }

            var removed = new HashSet<string>(StringComparer.Ordinal);
            foreach (var req in requests.Values)
            {
                req.Paths.RemoveAll(pid =>
                {
                    if (pid.StartsWith("dummy[", StringComparison.Ordinal) || keep.Contains(pid))
                        return false;
                    removed.Add(pid);
                    return true;
                });
            }

            foreach (var pid in removed)
                pathdict.Remove(pid);

            foreach (var map in sigToId.Values)
            {
                var staleSigs = new List<string>();
                foreach (var (sig, pid) in map)
                {
                    if (removed.Contains(pid))
                        staleSigs.Add(sig);
                }
                foreach (var sig in staleSigs)
                    map.Remove(sig);
            }
        }

        // Removes the per-request dummy arcs from the main network and each
        // sub-network, and drops the dummy columns from pathdict and req.Paths.
        private static void RemoveDummies(
            Network network,
            Dictionary<string, Request> requests,
            Dictionary<string, RoutePath> pathdict)
        {
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is null)
                    continue;

                var dummyNode = $"__dummy__[{req.Id}]";

                // The dummy node is unique to this request, so removing it also
                // strips its two incident legs from each network. This drops both
                // the dummy arcs AND the dummy node itself.
                network.RemoveNode(dummyNode);
                req.SubNetwork.Network.RemoveNode(dummyNode);

                var pathId = $"dummy[{req.Id}]";
                pathdict.Remove(pathId);
                req.Paths.Remove(pathId);
            }
        }

        // True iff the integer solution selects no dummy column.
        private static bool UsesNoDummy(IntegerMasterSolution solution)
        {
            foreach (var (_, pathIds) in solution.SelectedPaths)
            {
                foreach (var pid in pathIds)
                {
                    if (pid.StartsWith("dummy[", StringComparison.Ordinal))
                        return false;
                }
            }
            return true;
        }

        // Recomputes a solution's objective under the real path costs and arc fixed
        // costs (sum of pathdict[p].Cost over selected paths, plus FixedCost of every
        // arc touched by at least one selected path). Used to re-base an incumbent
        // carried over from a different cost basis (e.g. Phase 1's dummy penalty).
        private static double TrueObjective(
            Dictionary<string, List<string>> selectedPaths,
            Dictionary<string, RoutePath> pathdict,
            Network network)
        {
            var total = 0d;
            var usedArcIds = new HashSet<string>(StringComparer.Ordinal);
            foreach (var (_, pathIds) in selectedPaths)
            {
                foreach (var pid in pathIds)
                {
                    if (!pathdict.TryGetValue(pid, out var rp))
                        continue;
                    total += rp.Cost;
                    foreach (var arc in rp.Arcs)
                        if (arc.ResourceId is not null)
                            usedArcIds.Add(arc.ResourceId);
                }
            }

            // sigma_s of every resource touched by at least one selected path.
            foreach (var res in network.Resources)
            {
                if (usedArcIds.Contains(res.Id))
                    total += res.FixedCost;
            }

            return total;
        }


        // Gurobi callback: records the wall-clock time of the first incumbent whose
        // objective is at least as good (<=, minimization) as the target value.
        private sealed class ArcMipTimeToTargetCallback : GRBCallback
        {
            private readonly double _target;
            public double TimeToTargetSeconds { get; private set; } = double.NaN;

            public ArcMipTimeToTargetCallback(double target) => _target = target;

            protected override void Callback()
            {
                if (where != GRB.Callback.MIPSOL || !double.IsNaN(TimeToTargetSeconds))
                    return;
                var obj = GetDoubleInfo(GRB.Callback.MIPSOL_OBJ);
                if (obj <= _target)
                    TimeToTargetSeconds = GetDoubleInfo(GRB.Callback.RUNTIME);
            }
        }

        // When a model is infeasible, compute and report its IIS (irreducible
        // inconsistent subsystem) so the conflicting constraints/bounds are visible.
        internal static void ReportIfInfeasible(GRBModel model, string label)
        {
            if (model.Status != GRB.Status.INFEASIBLE)
                return;

            Console.Error.WriteLine($"{label} is INFEASIBLE; computing IIS...");
            model.ComputeIIS();

            foreach (var c in model.GetConstrs())
            {
                if (c.IISConstr > 0)
                    Console.Error.WriteLine($"  IIS constraint: {c.ConstrName}");
            }

            foreach (var v in model.GetVars())
            {
                if (v.IISLB > 0)
                    Console.Error.WriteLine($"  IIS var lower bound: {v.VarName}");
                if (v.IISUB > 0)
                    Console.Error.WriteLine($"  IIS var upper bound: {v.VarName}");
            }

            model.Write($"{label}_iis.ilp");
        }

        // Arc-id signature of a path, used to detect duplicate columns.
        private static string Signature(IReadOnlyList<Arc> arcs)
        {
            var ids = new string[arcs.Count];
            for (var i = 0; i < arcs.Count; i++)
                ids[i] = arcs[i].Id;
            return string.Join("|", ids);
        }
    }
}
