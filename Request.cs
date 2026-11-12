namespace MCFStatic
{
    internal sealed class Request
    {
        private readonly string _id;
        private readonly string _source;
        private readonly string _target;
        private readonly double _demandGbps;
        private readonly double _maxPingMs;
        private readonly double _revenue;
        private FeasibleSubNetwork? _subNetwork;
        private readonly List<string> _paths;

        public Request(
            string id,
            string source,
            string target,
            double demandGbps,
            double maxPingMs,
            double revenue)
        {
            _id = id;
            _source = source;
            _target = target;
            _demandGbps = demandGbps;
            _maxPingMs = maxPingMs;
            _revenue = revenue;
            _subNetwork = null;
            _paths = new();
        }

        public string Id => _id;
        public string Source => _source;
        public string Target => _target;
        public double DemandGbps => _demandGbps;
        public double MaxPingMs => _maxPingMs;
        public double Revenue => _revenue;

        public FeasibleSubNetwork? SubNetwork
        {
            get => _subNetwork;
            set => _subNetwork = value;
        }

        public List<string> Paths => _paths;
    }
}
