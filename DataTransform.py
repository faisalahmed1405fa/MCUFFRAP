"""Convert CPlus and R MULTIGEN .dow instances to the Canad_C_out CSV format.

Run `python MCFStatic/DataTransform.py` to create CPlus_out and R_out beside
this script, with one subdirectory per input filename (without .dow).
Each instance contains the three CSVs and an output folder for solver results.
Optional positional arguments select input directories; --output-root selects
the parent of the <input-directory-name>_out folders. Use --overwrite to
replace previously generated CSVs.

Arc fields are source, target, usage cost, capacity, fixed cost, cost count,
and original arc ID. The supported inputs have one cost per arc. CSV link
and request IDs follow Canad_C_out's zero-based row numbering; node names
are n1, n2, etc. No units are rescaled and directed arcs are not duplicated.
Unavailable geography, delay, and revenue fields are left empty.

Canadian-format .dat files (e.g. LargeMulgen; see the Format file) are also
supported: `python MCFStatic/DataTransform.py MCFStatic/LargeMulgen`.
Each arc is "end start fixed_cost mutual_capacity pk" followed by pk lines
"commodity cost individual_capacity"; trailing lines are "commodity node
supply". Following the SMS++ MMCFBlock reader, the origin of a commodity is
its node with negative supply. Fixed costs are written as absolute values,
the per-commodity arc costs are averaged into usage_cost_per_gbps, and
individual capacities are dropped (they must be at least the demand).
"""

import argparse
import csv
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


LINK_HEADER = (
    "link_id", "source", "target", "length_km", "delay_ms",
    "capacity_gbps", "fixed_cost", "usage_cost_per_gbps",
)
NODE_HEADER = ("node_id", "name", "lat", "lng", "population")
REQUEST_HEADER = (
    "request_id", "source", "target", "demand_gbps", "max_ping_ms",
    "revenue", "shortest_path_delay_ms",
)


def read_instance(path: Path) -> Tuple[int, List[List[str]], List[List[str]]]:
    """Validate and parse one MULTIGEN instance before writing any output."""
    records = [
        (number, line.split())
        for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1)
        if line.strip()
    ]

    def fail(line: int, message: str) -> ValueError:
        return ValueError(f"{path}:{line}: {message}")

    if len(records) < 2 or records[0][1] != ["MULTIGEN.DAT:"]:
        raise fail(1, "expected MULTIGEN.DAT: followed by instance counts")
    count_line, counts = records[1]
    if len(counts) != 3:
        raise fail(count_line, "expected node, arc, and request counts")
    try:
        node_count, arc_count, request_count = map(int, counts)
    except ValueError as exc:
        raise fail(count_line, "instance counts must be integers") from exc
    if node_count <= 0 or arc_count < 0 or request_count < 0:
        raise fail(count_line, "node count must be positive; other counts nonnegative")
    if len(records) != 2 + arc_count + request_count:
        raise fail(
            count_line,
            f"expected {arc_count} arcs and {request_count} requests, "
            f"found {len(records) - 2} data rows",
        )

    def endpoint(value: str, line: int) -> str:
        try:
            node = int(value)
        except ValueError as exc:
            raise fail(line, f"invalid node ID {value!r}") from exc
        if not 1 <= node <= node_count:
            raise fail(line, f"node ID {node} outside 1..{node_count}")
        return f"n{node}"

    def number(value: str, line: int) -> str:
        try:
            parsed = float(value)
        except ValueError as exc:
            raise fail(line, f"invalid numeric value {value!r}") from exc
        if not math.isfinite(parsed) or parsed < 0:
            raise fail(line, f"expected a finite nonnegative value, got {value!r}")
        return str(parsed)

    links: List[List[str]] = []
    original_ids = set()
    for link_id, (line, fields) in enumerate(records[2:2 + arc_count]):
        if len(fields) != 7:
            raise fail(line, f"expected 7 arc fields, got {len(fields)}")
        source, target, usage, capacity, fixed, cost_count, original_id = fields
        if cost_count != "1":
            raise fail(line, "only arcs with one cost are supported")
        try:
            arc_id = int(original_id)
        except ValueError as exc:
            raise fail(line, f"invalid arc ID {original_id!r}") from exc
        if arc_id in original_ids:
            raise fail(line, f"duplicate original arc ID {arc_id}")
        original_ids.add(arc_id)
        links.append([
            str(link_id), endpoint(source, line), endpoint(target, line), "", "",
            number(capacity, line), number(fixed, line), number(usage, line),
        ])

    requests: List[List[str]] = []
    for request_id, (line, fields) in enumerate(records[2 + arc_count:]):
        if len(fields) != 3:
            raise fail(line, f"expected 3 request fields, got {len(fields)}")
        source, target, demand = fields
        requests.append([
            str(request_id), endpoint(source, line), endpoint(target, line),
            number(demand, line), "", "", "",
        ])
    return node_count, links, requests


def read_canad_instance(path: Path) -> Tuple[int, List[List[str]], List[List[str]]]:
    """Validate and parse one Canadian-format (.dat) instance, streaming the file."""
    with path.open(encoding="utf-8-sig") as stream:
        records = ((number, line.split()) for number, line in enumerate(stream, 1))
        records = ((number, fields) for number, fields in records if fields)

        def fail(line: int, message: str) -> ValueError:
            return ValueError(f"{path}:{line}: {message}")

        def take(expected: int, what: str) -> Tuple[int, List[float]]:
            try:
                line, fields = next(records)
            except StopIteration:
                raise fail(-1, f"unexpected end of file while reading {what}") from None
            if len(fields) != expected:
                raise fail(line, f"expected {expected} {what} fields, got {len(fields)}")
            try:
                values = [float(value) for value in fields]
            except ValueError as exc:
                raise fail(line, f"invalid numeric value in {what}") from exc
            if not all(math.isfinite(value) for value in values):
                raise fail(line, f"non-finite value in {what}")
            return line, values

        def integer(value: float, line: int, low: int, high: int, what: str) -> int:
            if value != int(value) or not low <= value <= high:
                raise fail(line, f"{what} {value:g} outside {low}..{high}")
            return int(value)

        count_line, counts = take(3, "instance count")
        node_count, arc_count, commodity_count = (
            integer(value, count_line, 0, 10**9, "count") for value in counts
        )
        if node_count <= 0:
            raise fail(count_line, "node count must be positive")

        arcs = []
        for _ in range(arc_count):
            line, (end, start, fixed, capacity, pk) = take(5, "arc")
            head = integer(end, line, 1, node_count, "node ID")
            tail = integer(start, line, 1, node_count, "node ID")
            if capacity < 0:
                raise fail(line, "uncapacitated arcs are not supported")
            pk = integer(pk, line, 0, commodity_count, "commodity count")
            if pk != commodity_count:
                raise fail(line, "arcs restricted to a subset of commodities are not supported")
            costs = []
            bounds = {}
            for _ in range(pk):
                cline, (commodity, cost, bound) = take(3, "arc-commodity")
                commodity = integer(commodity, cline, 1, commodity_count, "commodity")
                if commodity in bounds:
                    raise fail(cline, f"duplicate commodity {commodity} on arc")
                if cost < 0:
                    raise fail(cline, f"negative arc cost {cost:g}")
                costs.append(cost)
                bounds[commodity] = (cline, bound)
            arcs.append((line, tail, head, capacity, abs(fixed), sum(costs) / pk, bounds))

        supplies = {}
        for line, fields in records:
            if len(fields) != 3:
                raise fail(line, f"expected 3 supply fields, got {len(fields)}")
            try:
                commodity, node, supply = (float(value) for value in fields)
            except ValueError as exc:
                raise fail(line, "invalid numeric value in supply") from exc
            commodity = integer(commodity, line, 1, commodity_count, "commodity")
            node = integer(node, line, 1, node_count, "node ID")
            if supply != 0:
                supplies.setdefault(commodity, []).append((line, node, supply))

    requests: List[List[str]] = []
    demands = {}
    for commodity in range(1, commodity_count + 1):
        entries = supplies.get(commodity, [])
        origins = [entry for entry in entries if entry[2] < 0]
        destinations = [entry for entry in entries if entry[2] > 0]
        if len(origins) != 1 or len(destinations) != 1:
            raise fail(
                entries[0][0] if entries else -1,
                f"commodity {commodity} must have exactly one origin and one destination",
            )
        (line, origin, out), (_, target, into) = origins[0], destinations[0]
        if -out != into:
            raise fail(line, f"commodity {commodity} supplies do not balance")
        demands[commodity] = into
        requests.append([
            str(commodity - 1), f"n{origin}", f"n{target}", str(into), "", "", "",
        ])

    links: List[List[str]] = []
    for link_id, (line, tail, head, capacity, fixed, usage, bounds) in enumerate(arcs):
        for commodity, (cline, bound) in bounds.items():
            if 0 <= bound < demands[commodity]:
                raise fail(cline, "individual capacity below demand is not supported")
        links.append([
            str(link_id), f"n{tail}", f"n{head}", "", "",
            str(capacity), str(fixed), str(usage),
        ])
    return node_count, links, requests


READERS = {".dow": read_instance, ".dat": read_canad_instance}


def convert_instance(source: Path, destination: Path, overwrite: bool = False) -> None:
    """Write the three CSVs and create an output folder for solver results."""
    node_count, links, requests = READERS[source.suffix.lower()](source)
    outputs = (
        (destination / "links.csv", LINK_HEADER, links),
        (destination / "nodes.csv", NODE_HEADER, [
            [str(node), f"n{node}", "", "", ""] for node in range(1, node_count + 1)
        ]),
        (destination / "requests.csv", REQUEST_HEADER, requests),
    )
    for path, _, _ in outputs:
        if path.exists() and not overwrite:
            raise FileExistsError(f"{path} already exists; use --overwrite to replace it")
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "output").mkdir(exist_ok=True)
    for path, header, rows in outputs:
        with path.open("w" if overwrite else "x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)


def main(argv: Optional[Sequence[str]] = None) -> None:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_dirs", nargs="*", type=Path, default=[base / "CPlus", base / "R"],
        help="directories containing .dow or .dat files (default: CPlus and R beside this script)",
    )
    parser.add_argument(
        "--output-root", type=Path,
        help="parent of output folders (default: each input directory's parent)",
    )
    parser.add_argument("--overwrite", action="store_true", help="replace existing output CSVs")
    args = parser.parse_args(argv)
    batches = []
    destinations = set()
    for directory in args.input_dirs:
        if not directory.is_dir():
            parser.error(f"input directory does not exist: {directory}")
        sources = sorted(
            path for path in directory.iterdir()
            if path.is_file() and path.suffix.lower() in READERS
        )
        if not sources:
            parser.error(f"no .dow or .dat files found in {directory}")
        root = (args.output_root or directory.parent) / f"{directory.name}_out"
        for source in sources:
            destination = root / source.stem
            resolved = destination.resolve()
            if resolved in destinations:
                parser.error(f"multiple inputs map to the same output: {destination}")
            destinations.add(resolved)
            batches.append((source, destination))
    for source, destination in batches:
        convert_instance(source, destination, overwrite=args.overwrite)
        print(f"{source.name} -> {destination}")
    print(f"Converted {len(batches)} instances.")


if __name__ == "__main__":
    main()