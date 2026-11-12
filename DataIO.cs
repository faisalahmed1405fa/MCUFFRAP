using System.Globalization;

namespace MCFStatic
{
    internal static class DataIO
    {
        public static Dictionary<string, Request> LoadRequests(string path)
        {
            var list = ReadRequests(path);
            var dict = new Dictionary<string, Request>(list.Count, StringComparer.Ordinal);
            foreach (var r in list)
            {
                if (!dict.TryAdd(r.Id, r))
                    throw new InvalidOperationException($"Duplicate request id '{r.Id}'.");
            }
            return dict;
        }

        public static Network LoadNetwork(string path, NetworkKind kind)
            => new(ReadArcs(path, kind));

        public static IReadOnlyList<Request> ReadRequests(string path)
        {
            var required = new[] { "request_id", "source", "target", "demand_gbps" };
            return ReadCsv(path, "requests", required, get =>
                new Request(
                    get("request_id")!,
                    get("source")!,
                    get("target")!,
                    ParseDouble(get("demand_gbps")!, "demand_gbps", path),
                    ParseDoubleOrNaN(get("max_ping_ms"), "max_ping_ms", path),
                    ParseDoubleOrNaN(get("revenue"), "revenue", path)));
        }

        public static IReadOnlyList<Arc> ReadArcs(string path, NetworkKind kind)
        {
            var required = new[] { "link_id", "source", "target", "capacity_gbps", "fixed_cost", "usage_cost_per_gbps" };
            var rows = ReadCsv(path, "arcs", required, get =>
                (LinkId: get("link_id")!,
                 Source: get("source")!,
                 Target: get("target")!,
                 LengthKm: ParseDoubleOrNaN(get("length_km"), "length_km", path),
                 DelayMs: ParseDoubleOrNaN(get("delay_ms"), "delay_ms", path),
                 CapacityGbps: ParseDouble(get("capacity_gbps")!, "capacity_gbps", path),
                 FixedCost: ParseDouble(get("fixed_cost")!, "fixed_cost", path),
                 UsageCostPerGbps: ParseDouble(get("usage_cost_per_gbps")!, "usage_cost_per_gbps", path),
                 Weight: ParseDoubleOrNaN(get("weight"), "weight", path),
                 Resource: get("resource")));

            var bidirectional = kind == NetworkKind.Tele;
            var arcs = new List<Arc>(bidirectional ? rows.Count * 2 : rows.Count);
            foreach (var r in rows)
            {
                var weight = double.IsNaN(r.Weight) ? r.UsageCostPerGbps : r.Weight;

                // s(a): when no resource is given, each directed arc is its own
                // resource, which recovers the classic FCMCF.
                var forwardResource = r.Resource ?? $"{r.Source}->{r.Target}";
                var reverseResource = r.Resource ?? $"{r.Target}->{r.Source}";

                arcs.Add(new Arc(
                    r.LinkId,
                    r.Source,
                    r.Target,
                    r.LengthKm,
                    r.DelayMs,
                    r.CapacityGbps,
                    r.FixedCost,
                    r.UsageCostPerGbps,
                    weight,
                    forwardResource));

                if (bidirectional)
                {
                    arcs.Add(new Arc(
                        r.LinkId,
                        r.Target,
                        r.Source,
                        r.LengthKm,
                        r.DelayMs,
                        r.CapacityGbps,
                        r.FixedCost,
                        r.UsageCostPerGbps,
                        weight,
                        reverseResource));
                }
            }
            return arcs;
        }

        private static List<T> ReadCsv<T>(
            string path,
            string label,
            IReadOnlyCollection<string> requiredColumns,
            Func<Func<string, string?>, T> factory)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("Path must be provided.", nameof(path));
            if (!File.Exists(path))
                throw new FileNotFoundException($"{label} file not found.", path);

            using var reader = new StreamReader(path);

            string? headerLine;
            var lineNumber = 0;
            do
            {
                headerLine = reader.ReadLine();
                lineNumber++;
            }
            while (headerLine != null && string.IsNullOrWhiteSpace(headerLine));

            if (headerLine == null)
                throw new FormatException($"{label} file '{path}' is empty.");

            var headerFields = headerLine.Split(',');
            for (var i = 0; i < headerFields.Length; i++)
                headerFields[i] = headerFields[i].Trim();

            var headerIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            for (var i = 0; i < headerFields.Length; i++)
            {
                if (headerFields[i].Length == 0)
                    continue;
                headerIndex[headerFields[i]] = i;
            }

            foreach (var required in requiredColumns)
            {
                if (!headerIndex.ContainsKey(required))
                    throw new FormatException($"{label} file '{path}' is missing required column '{required}'.");
            }

            var rows = new List<T>();
            string[] currentFields = Array.Empty<string>();

            string? Get(string column)
            {
                if (!headerIndex.TryGetValue(column, out var idx) || idx >= currentFields.Length)
                    return null;
                var value = currentFields[idx].Trim();
                return value.Length == 0 ? null : value;
            }

            string? rawLine;
            while ((rawLine = reader.ReadLine()) != null)
            {
                lineNumber++;
                var line = rawLine.Trim();
                if (line.Length == 0)
                    continue;

                currentFields = line.Split(',');
                rows.Add(factory(Get));
            }

            return rows;
        }

        private static double ParseDouble(string value, string column, string path)
        {
            if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
                throw new FormatException($"Invalid {column} value '{value}' in '{path}'.");
            return parsed;
        }

        private static double ParseDoubleOrNaN(string? value, string column, string path)
        {
            if (value is null)
                return double.NaN;
            if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
                throw new FormatException($"Invalid {column} value '{value}' in '{path}'.");
            return parsed;
        }
    }
}
