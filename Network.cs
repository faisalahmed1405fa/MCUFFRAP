namespace MCFStatic
{
    internal enum NetworkKind
    {
        Static,
        Tele,
    }

    internal readonly record struct Arc(
        string LinkId,
        string Source,
        string Target,
        double LengthKm,
        double DelayMs,
        double CapacityGbps,
        double FixedCost,
        double UsageCostPerGbps,
        double Weight,
        string? ResourceId = null)
    {
        // s(a): the resource this arc belongs to. Arcs without a resource
        // (e.g. artificial dummy arcs) carry no fixed cost and need no y_s.
        public bool HasResource => ResourceId is not null;

        // Directional identifier, unique per arc within a network
        // (a network forbids duplicate Source -> Target pairs).
        public string Id => $"{Source}->{Target}";
    }

    internal sealed class Network
    {
        private readonly List<Arc> _arcs;
        private readonly HashSet<string> _nodes;
        private readonly Dictionary<(string Source, string Target), Arc> _arcLookup;
        private readonly Dictionary<string, List<Arc>> _outgoing;
        private readonly Dictionary<string, List<Arc>> _incoming;
        private readonly Dictionary<string, Resource> _resources;

        public Network(IEnumerable<Arc> arcs)
        {
            ArgumentNullException.ThrowIfNull(arcs);

            _arcs = new List<Arc>();
            _nodes = new HashSet<string>(StringComparer.Ordinal);
            _arcLookup = new Dictionary<(string, string), Arc>();
            _outgoing = new Dictionary<string, List<Arc>>(StringComparer.Ordinal);
            _incoming = new Dictionary<string, List<Arc>>(StringComparer.Ordinal);
            _resources = new Dictionary<string, Resource>(StringComparer.Ordinal);

            foreach (var arc in arcs)
                Add(arc);

            foreach (var resource in _resources.Values)
                resource.OrderAsChain();
        }

        public IReadOnlyCollection<Resource> Resources => _resources.Values;

        public int ResourceCount => _resources.Count;

        public bool TryGetResource(string resourceId, out Resource resource)
            => _resources.TryGetValue(resourceId, out resource!);

        public Resource GetResource(string resourceId)
            => _resources.TryGetValue(resourceId, out var resource)
                ? resource
                : throw new KeyNotFoundException($"No resource '{resourceId}'.");

        // s(a): the resource of an arc, or null if the arc has none.
        public Resource? ResourceOf(Arc arc)
            => arc.ResourceId is not null && _resources.TryGetValue(arc.ResourceId, out var resource)
                ? resource
                : null;

        public IReadOnlyList<Arc> Arcs => _arcs;

        public IReadOnlyCollection<string> Nodes => _nodes;

        public int ArcCount => _arcs.Count;

        public int NodeCount => _nodes.Count;

        public bool TryGetArc(string source, string target, out Arc arc)
            => _arcLookup.TryGetValue((source, target), out arc);

        public double GetCapacity(string source, string target)
            => TryGetArc(source, target, out var arc)
                ? arc.CapacityGbps
                : throw new KeyNotFoundException($"No arc from '{source}' to '{target}'.");

        public IReadOnlyList<Arc> OutgoingFrom(string node)
            => _outgoing.TryGetValue(node, out var list) ? list : Array.Empty<Arc>();

        public IReadOnlyList<Arc> IncomingTo(string node)
            => _incoming.TryGetValue(node, out var list) ? list : Array.Empty<Arc>();

        // Adds an arc to the network. Returns false (without modifying the
        // network) if an arc with the same Source -> Target already exists.
        public bool AddArc(Arc arc)
        {
            if (_arcLookup.ContainsKey((arc.Source, arc.Target)))
                return false;
            Add(arc);
            if (arc.ResourceId is not null)
                _resources[arc.ResourceId].OrderAsChain();
            return true;
        }

        // Removes the arc Source -> Target if present. Returns false if no such
        // arc exists. Nodes left without arcs are kept in the node set.
        public bool RemoveArc(string source, string target)
        {
            if (!_arcLookup.Remove((source, target), out var arc))
                return false;

            _arcs.Remove(arc);

            if (arc.ResourceId is not null && _resources.TryGetValue(arc.ResourceId, out var resource))
            {
                resource.RemoveArc(arc);
                if (resource.ArcCount == 0)
                    _resources.Remove(arc.ResourceId);
                else
                    resource.OrderAsChain();
            }

            if (_outgoing.TryGetValue(source, out var outList))
            {
                outList.Remove(arc);
                if (outList.Count == 0)
                    _outgoing.Remove(source);
            }

            if (_incoming.TryGetValue(target, out var inList))
            {
                inList.Remove(arc);
                if (inList.Count == 0)
                    _incoming.Remove(target);
            }

            return true;
        }

        // Removes a node and every arc incident to it (incoming and outgoing).
        // Returns false if the node is not present.
        public bool RemoveNode(string node)
        {
            if (!_nodes.Contains(node))
                return false;

            // Copy first: RemoveArc mutates the adjacency lists we'd be iterating.
            var incident = new List<Arc>();
            if (_outgoing.TryGetValue(node, out var outList))
                incident.AddRange(outList);
            if (_incoming.TryGetValue(node, out var inList))
                incident.AddRange(inList);

            foreach (var arc in incident)
                RemoveArc(arc.Source, arc.Target);

            _nodes.Remove(node);
            return true;
        }

        private void Add(Arc arc)
        {
            if (string.IsNullOrWhiteSpace(arc.Source))
                throw new ArgumentException("Arc source must be provided.", nameof(arc));
            if (string.IsNullOrWhiteSpace(arc.Target))
                throw new ArgumentException("Arc target must be provided.", nameof(arc));

            var key = (arc.Source, arc.Target);
            if (!_arcLookup.TryAdd(key, arc))
                throw new InvalidOperationException($"Duplicate arc '{arc.Source}' -> '{arc.Target}'.");

            _arcs.Add(arc);
            _nodes.Add(arc.Source);
            _nodes.Add(arc.Target);

            if (!_outgoing.TryGetValue(arc.Source, out var outList))
            {
                outList = new List<Arc>();
                _outgoing[arc.Source] = outList;
            }
            outList.Add(arc);

            if (!_incoming.TryGetValue(arc.Target, out var inList))
            {
                inList = new List<Arc>();
                _incoming[arc.Target] = inList;
            }
            inList.Add(arc);

            if (arc.ResourceId is not null)
            {
                if (!_resources.TryGetValue(arc.ResourceId, out var resource))
                {
                    resource = new Resource(arc.ResourceId);
                    _resources[arc.ResourceId] = resource;
                }
                resource.AddArc(arc);
            }
        }
    }
}
