namespace MCFStatic
{
    internal sealed class RoutePath
    {
        public RoutePath(string id, Request request, IReadOnlyList<Arc> arcs)
        {
            Id = id;
            Request = request;
            Arcs = arcs;
        }

        // Unique identifier of this path.
        public string Id { get; }

        // The request this path routes.
        public Request Request { get; }

        // Ordered list of Arc objects that make up the path.
        public IReadOnlyList<Arc> Arcs { get; }

        // Raw (original) cost of the path: sum of usage cost * demand over arcs.
        public double RawCost { get; private set; }

        // Objective coefficient actually used by the master. Defaults to RawCost
        // but can be overridden per phase (e.g. 0 for real paths in the Phase-1
        // feasibility model so the objective only penalizes dummy arcs).
        public double Cost { get; set; }

        // Calculates the raw cost of this path and resets Cost to match it.
        public double CalculateCost()
        {
            double totalCost = 0d;
            foreach (var arc in Arcs)
            {
                totalCost += arc.UsageCostPerGbps*Request.DemandGbps;
            }

            RawCost = totalCost;
            Cost = totalCost;
            return totalCost;
        }
    }
}
