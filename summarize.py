#!/usr/bin/env python3
"""Summarize the --enable-dual-lb sweep (duallb_sweep.sbatch) into one CSV.

Layout read:  <results>/<dataset>/<instance>/dual_lb<k>/{phase1,phase2,weaklp,weaklp_si}.*, run.out
One row per instance with:
  * instance size: nodes, arcs, commodities (requests), as loaded by the solver
  * objectives: weak LP, weak LP + SI cuts, CG root LP (final phase-2 master LP) for each dual-lb value
  * run times (s): weak LP, weak LP + SI cuts, total column-generation time for each dual-lb value
  * iterations: phase-1 iterations (identical across dual-lb values) and phase-2 iterations per value

Usage: python3 summarize.py [results_dir] [-o summary.csv]
"""
import argparse
import csv
import re
import sys
from pathlib import Path

DUAL_LB_VALUES = (0, 1, 2, 3)
CG_TIME_RE = re.compile(r"^Column generation .*\(([\d.]+) s\)\.?\s*$", re.MULTILINE)
GRB_INFINITY = 1e99
NETWORK_RE = re.compile(r"^Loaded \w+ network: (\d+) nodes, (\d+) arcs\.", re.MULTILINE)
REQUESTS_RE = re.compile(r"^Loaded (\d+) requests", re.MULTILINE)
PHASE_TIME_RE = re.compile(r"Phase [12] run time: ([\d.]+) s\.")


def natural_key(name):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", name)]


def read_rows(path):
    if not path.is_file():
        return None
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def weak_lp(run_dir, stem):
    """Return (objective, solve_s, issue); objective is blank if the LP was not solved to optimality."""
    rows = read_rows(run_dir / f"{stem}.csv")
    if not rows:
        return "", "", "missing"
    obj, secs = rows[0]["objective"], rows[0]["solve_s"]
    log = run_dir / f"{stem}.log"
    if log.is_file() and "Time limit reached" in log.read_text(errors="replace"):
        return "", secs, "time limit"
    try:
        # An interrupted Gurobi solve reports GRB.INFINITY (1e100) as the objective.
        if abs(float(obj)) >= GRB_INFINITY:
            return "", secs, "not solved"
    except ValueError:
        return "", secs, "unparsable objective"
    return obj, secs, ""


def cg_time(run_dir):
    run_out = run_dir / "run.out"
    if run_out.is_file():
        m = CG_TIME_RE.search(run_out.read_text(errors="replace"))
        if m:
            return m.group(1)
    # Fallback: phase 1 + phase 2 run times from the phase logs.
    times = []
    for log in ("phase1.log", "phase2.log"):
        p = run_dir / log
        if p.is_file():
            times += [float(t) for t in PHASE_TIME_RE.findall(p.read_text(errors="replace"))]
    return f"{sum(times):.1f}" if times else ""


def instance_size(run_dir):
    """(nodes, arcs, commodities) as loaded by the solver; each request is one commodity."""
    run_out = run_dir / "run.out"
    if not run_out.is_file():
        return "", "", ""
    text = run_out.read_text(errors="replace")
    net, req = NETWORK_RE.search(text), REQUESTS_RE.search(text)
    return (net.group(1) if net else "", net.group(2) if net else "", req.group(1) if req else "")


def summarize_instance(dataset, inst_dir):
    row = {"dataset": dataset, "instance": inst_dir.name}
    notes = []

    lb0_dir = inst_dir / f"dual_lb{DUAL_LB_VALUES[0]}"
    row["nodes"], row["arcs"], row["commodities"] = instance_size(lb0_dir)
    if not all((row["nodes"], row["arcs"], row["commodities"])):
        notes.append("instance size missing")
    for stem in ("weaklp", "weaklp_si"):
        row[f"{stem}_obj"], row[f"{stem}_time_s"], issue = weak_lp(lb0_dir, stem)
        if issue:
            notes.append(f"{stem} {issue}")

    phase1_iters = set()
    for lb in DUAL_LB_VALUES:
        run_dir = inst_dir / f"dual_lb{lb}"
        p1 = read_rows(run_dir / "phase1.csv")
        p2 = read_rows(run_dir / "phase2.csv")
        if p1 is not None:
            phase1_iters.add(len(p1))
        row[f"dual_lb{lb}_obj"] = p2[-1]["master_lp"] if p2 else ""
        row[f"dual_lb{lb}_iters"] = len(p2) if p2 else ""
        row[f"dual_lb{lb}_time_s"] = cg_time(run_dir) if run_dir.is_dir() else ""

        exit_file = run_dir / "exit_code"
        rc = exit_file.read_text().strip() if exit_file.is_file() else "missing"
        if rc != "0":
            notes.append(f"dual_lb{lb} exit {rc}")
        elif not p2:
            log = run_dir / "phase1.log"
            if log.is_file() and "skipping Phase 2" in log.read_text(errors="replace"):
                notes.append(f"dual_lb{lb} phase 1 infeasible")
            else:
                notes.append(f"dual_lb{lb} phase 2 missing")

    row["phase1_iters"] = phase1_iters.pop() if len(phase1_iters) == 1 else "/".join(map(str, sorted(phase1_iters)))
    # Collapse identical per-lb notes (e.g. phase-1 infeasibility affects every run).
    infeasible = [n for n in notes if n.endswith("phase 1 infeasible")]
    if len(infeasible) == len(DUAL_LB_VALUES):
        notes = [n for n in notes if n not in infeasible] + ["phase 1 infeasible (no phase 2)"]
    row["note"] = "; ".join(notes)
    return row


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results_dir", nargs="?", default=here, type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None, help="default: <results_dir>/summary.csv")
    args = ap.parse_args()

    results = args.results_dir.resolve()
    out = args.output or results / "summary.csv"

    columns = (
        ["dataset", "instance", "nodes", "arcs", "commodities", "weaklp_obj", "weaklp_si_obj"]
        + [f"dual_lb{lb}_obj" for lb in DUAL_LB_VALUES]
        + ["weaklp_time_s", "weaklp_si_time_s"]
        + [f"dual_lb{lb}_time_s" for lb in DUAL_LB_VALUES]
        + ["phase1_iters"]
        + [f"dual_lb{lb}_iters" for lb in DUAL_LB_VALUES]
        + ["note"]
    )

    rows = []
    for ds_dir in sorted((d for d in results.iterdir() if d.is_dir()), key=lambda d: natural_key(d.name)):
        instances = sorted((d for d in ds_dir.iterdir() if d.is_dir()), key=lambda d: natural_key(d.name))
        ds_rows = [summarize_instance(ds_dir.name, inst) for inst in instances]
        flagged = sum(1 for r in ds_rows if r["note"])
        print(f"{ds_dir.name}: {len(ds_rows)} instances, {flagged} with notes")
        rows += ds_rows

    if not rows:
        sys.exit(f"No instances found under {results}")

    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} rows to {out}")


if __name__ == "__main__":
    main()
