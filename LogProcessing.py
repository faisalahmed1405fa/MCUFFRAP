"""Extract one CSV row per ``=== Running: ... ===`` block in a solver log.

Usage: python MCFStatic/LogProcessing.py [log.out] [-o results.csv]
The default input is CPlus_out/CplusInstance.out beside this script. The
default output is <log-stem>_metrics.csv beside the input log.

All runtimes are seconds, using the application's reported wall-clock times
(not Gurobi's internal solve time). The Phase 2 final LP objective is the last
master LP table value, even if no integer feasible solution was found.
Phase 2 iterations count the logged master LP rows, including iteration 0;
the count is blank if Phase 2 was not started.
The total run time is the sum of Phase 1 and Phase 2 runtimes. The percentage
gap is 100 * (Phase 2 final LP objective - weak LP objective) / weak LP
objective. The weak LP objective uses Gurobi's higher-precision "Optimal
objective" when available, otherwise the application's "Objective:" summary.
Missing or undefined metrics are blank in the CSV and reported to stderr.
Repeated instance runs remain separate rows in log order.
"""

import argparse
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path
from typing import Dict, List, Optional, Sequence


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
HEADER = (
    "instance",
    "phase1_run_time_s",
    "phase2_run_time_s",
    "total_run_time_s",
    "phase2_iterations",
    "phase2_final_lp_objective",
    "lp_relax_objective",
    "gap_percent",
    "lp_relax_run_time_s",
)
INSTANCE = re.compile(r"^\s*=== Running:\s*(.+?)\s*===\s*$")
PHASE_TIME = re.compile(rf"^\s*Phase ([12]) run time:\s*({NUMBER})\s*s\.?\s*$")
MASTER_ROW = re.compile(rf"^\s*\d+\s*\|\s*({NUMBER})\s*\|")
RELAX_TIME = re.compile(
    rf"^\s*Weak LP relaxation solved in\s+({NUMBER})\s*s\.?\s*$"
)
OPTIMAL_OBJECTIVE = re.compile(rf"^\s*Optimal objective\s+({NUMBER})\s*$")
SUMMARY_OBJECTIVE = re.compile(rf"^\s*Objective:\s*({NUMBER})\s*$")


def parse_log(path: Path) -> List[Dict[str, str]]:
    """Parse instance metrics without mixing values from different runs."""
    rows: List[Dict[str, str]] = []
    current: Optional[Dict[str, str]] = None
    section = ""
    phase2_iterations = 0
    with path.open(encoding="utf-8-sig") as log:
        for line in log:
            match = INSTANCE.match(line)
            if match:
                current = dict.fromkeys(HEADER, "")
                current["instance"] = Path(match.group(1)).name
                rows.append(current)
                section = ""
                phase2_iterations = 0
                continue
            if current is None:
                continue
            text = line.strip()
            if text.startswith("Phase 1 ("):
                section = "phase1"
            elif text.startswith("Phase 2 ("):
                section = "phase2"
                current["phase2_iterations"] = str(phase2_iterations)
            elif text == "Running weak LP relaxation...":
                section = "relax"

            match = PHASE_TIME.match(line)
            if match:
                current[f"phase{match.group(1)}_run_time_s"] = match.group(2)
                section = ""
                continue
            if section == "phase2":
                match = MASTER_ROW.match(line)
                if match:
                    phase2_iterations += 1
                    current["phase2_iterations"] = str(phase2_iterations)
                    current["phase2_final_lp_objective"] = match.group(1)
            elif section == "relax":
                match = OPTIMAL_OBJECTIVE.match(line)
                if match:
                    current["lp_relax_objective"] = match.group(1)
                match = SUMMARY_OBJECTIVE.match(line)
                if match and not current["lp_relax_objective"]:
                    current["lp_relax_objective"] = match.group(1)
                match = RELAX_TIME.match(line)
                if match:
                    current["lp_relax_run_time_s"] = match.group(1)
    if not rows:
        raise ValueError(f"{path}: no '=== Running: ... ===' instance blocks found")
    for row in rows:
        if row["phase1_run_time_s"] and row["phase2_run_time_s"]:
            total_time = Decimal(row["phase1_run_time_s"]) + Decimal(
                row["phase2_run_time_s"]
            )
            row["total_run_time_s"] = format(total_time.normalize(), "f")
        if row["phase2_final_lp_objective"] and row["lp_relax_objective"]:
            relax_objective = Decimal(row["lp_relax_objective"])
            if relax_objective != 0:
                gap = (
                    Decimal(row["phase2_final_lp_objective"]) - relax_objective
                ) / relax_objective * 100
                row["gap_percent"] = f"{gap:.2f}"
    return rows


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log", nargs="?", type=Path,
        default=Path(__file__).parent / "CPlus_out" / "CplusInstance.out",
        help="solver output log",
    )
    parser.add_argument("-o", "--output", type=Path, help="destination CSV")
    args = parser.parse_args(argv)
    output = args.output or args.log.with_name(f"{args.log.stem}_metrics.csv")
    if output.resolve() == args.log.resolve():
        parser.error("output must not overwrite the input log")
    try:
        rows = parse_log(args.log)
        for number, row in enumerate(rows, 1):
            missing = [key for key in HEADER[1:] if not row[key]]
            if missing:
                print(
                    f"Warning: run {number} ({row['instance']}): missing "
                    + ", ".join(missing),
                    file=sys.stderr,
                )
        with output.open("w", newline="", encoding="utf-8") as destination:
            writer = csv.DictWriter(destination, fieldnames=HEADER)
            writer.writeheader()
            writer.writerows(rows)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Wrote {len(rows)} instance rows to {output}")


if __name__ == "__main__":
    main()