namespace MCFStatic
{
    // A resource s is a contiguous sequence of arcs (no breakage) that is
    // activated as a whole: y_s pays sigma_s once and opens capacity c_a on
    // every arc a with s(a) = s. sigma_s is the sum of the fixed costs of the
    // arcs assigned to the resource. When every arc is its own resource the
    // model reduces to the classic FCMCF.
    internal sealed class Resource
    {
        private readonly List<Arc> _arcs = new();

        public Resource(string id)
        {
            if (string.IsNullOrWhiteSpace(id))
                throw new ArgumentException("Resource id must be provided.", nameof(id));
            Id = id;
        }

        public string Id { get; }

        // Arcs of the resource, ordered along the sequence when it forms a chain.
        public IReadOnlyList<Arc> Arcs => _arcs;

        public int ArcCount => _arcs.Count;

        // True when the arcs form a single unbroken directed chain.
        public bool IsChain { get; private set; } = true;

        // sigma_s: sum of fixed costs of the assigned arcs. Arcs sharing a LinkId
        // (the two directions of a bidirectional link) are charged only once.
        public double FixedCost
        {
            get
            {
                var seen = new HashSet<string>(StringComparer.Ordinal);
                var total = 0d;
                foreach (var arc in _arcs)
                {
                    if (seen.Add(arc.LinkId))
                        total += arc.FixedCost;
                }
                return total;
            }
        }

        public bool Contains(Arc arc) => _arcs.Contains(arc);

        internal void AddArc(Arc arc)
        {
            if (!string.Equals(arc.ResourceId, Id, StringComparison.Ordinal))
                throw new ArgumentException($"Arc '{arc.Id}' belongs to resource '{arc.ResourceId}', not '{Id}'.", nameof(arc));
            _arcs.Add(arc);
        }

        internal bool RemoveArc(Arc arc) => _arcs.Remove(arc);

        // Reorders the arcs into a directed chain (head -> ... -> tail). If the
        // arcs do not form a single unbroken chain the original order is kept and
        // IsChain is set to false.
        internal void OrderAsChain()
        {
            IsChain = TryBuildChain(out var ordered);
            if (IsChain)
            {
                _arcs.Clear();
                _arcs.AddRange(ordered);
            }
        }

        private bool TryBuildChain(out List<Arc> ordered)
        {
            ordered = new List<Arc>(_arcs.Count);
            if (_arcs.Count <= 1)
            {
                ordered.AddRange(_arcs);
                return true;
            }

            // A resource must be a simple directed path: each node is left at most
            // once and entered at most once, exactly one head, and no loops.
            var bySource = new Dictionary<string, Arc>(StringComparer.Ordinal);
            var targets = new HashSet<string>(StringComparer.Ordinal);
            foreach (var arc in _arcs)
            {
                if (!bySource.TryAdd(arc.Source, arc) || !targets.Add(arc.Target))
                    return false; // branching or merging
            }

            var heads = _arcs.Where(a => !targets.Contains(a.Source)).ToList();
            if (heads.Count != 1)
                return false; // loop or multiple pieces

            var visited = new HashSet<string>(StringComparer.Ordinal) { heads[0].Source };
            var current = heads[0];
            while (true)
            {
                if (!visited.Add(current.Target))
                    return false; // loop
                ordered.Add(current);
                if (!bySource.TryGetValue(current.Target, out var next))
                    break;
                current = next;
            }

            return ordered.Count == _arcs.Count;
        }

        public override string ToString()
            => $"{Id} [{string.Join(", ", _arcs.Select(a => a.Id))}] sigma={FixedCost}";
    }
}
