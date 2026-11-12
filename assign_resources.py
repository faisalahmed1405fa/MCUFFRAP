"""
Assign arcs of FCMCF instances to resources via random walks.

For every links.csv under the data root, repeatedly:
  1. pick a random node that still has unassigned outgoing arcs,
  2. walk along unassigned outgoing arcs for up to L steps, where
     L = round(Normal(mean, std)) clipped to [1, max_len],
  3. assign all traversed arcs to a new resource and remove them from the pool.
The walk stops early if the current node has no unassigned outgoing arcs.
Continues until every arc belongs to exactly one resource.

A 'resource' column is added (or overwritten) in each links.csv.

Usage:
  python assign_resources.py [folder] [--mean 3] [--std 1] [--max-len 5] [--seed 42]
"""
import argparse
import csv
import os
import random
from collections import defaultdict


def assign_resources(rows, rng, mean, std, max_len):
	out_arcs = defaultdict(list)  # node -> list of row indices (unassigned)
	for i, r in enumerate(rows):
		out_arcs[r["source"]].append(i)

	resource = [None] * len(rows)
	remaining = len(rows)
	rid = 0

	while remaining > 0:
		start_nodes = [n for n, arcs in out_arcs.items() if arcs]
		node = rng.choice(start_nodes)
		walk_len = min(max_len, max(1, round(rng.gauss(mean, std))))

		steps = 0
		visited = {node}
		while steps < walk_len:
			arcs = out_arcs[node]
			# only extend to nodes not yet on this path (keeps it simple/open)
			cand = [k for k, a in enumerate(arcs) if rows[a]["target"] not in visited]
			if not cand:
				if steps == 0 and arcs:
					# only self-loops remain at this node; each becomes its own resource
					cand = [0]
				else:
					break
			idx = arcs.pop(rng.choice(cand))
			resource[idx] = rid
			remaining -= 1
			steps += 1
			node = rows[idx]["target"]
			if node in visited:
				break
			visited.add(node)

		rid += 1

	return resource, rid


def process_file(path, rng, mean, std, max_len):
	with open(path, newline="") as f:
		reader = csv.DictReader(f)
		fieldnames = list(reader.fieldnames)
		rows = list(reader)

	resource, n_res = assign_resources(rows, rng, mean, std, max_len)

	if "resource" not in fieldnames:
		fieldnames.append("resource")
	for r, res in zip(rows, resource):
		r["resource"] = res

	with open(path, "w", newline="") as f:
		writer = csv.DictWriter(f, fieldnames=fieldnames)
		writer.writeheader()
		writer.writerows(rows)

	print(f"{path}: {len(rows)} arcs -> {n_res} resources")


def main():
	here = os.path.dirname(os.path.abspath(__file__))
	p = argparse.ArgumentParser()
	p.add_argument("folder", nargs="?", help="data folder containing links.csv files (searched recursively)")
	p.add_argument("--root", default=None, help="same as positional folder")
	p.add_argument("--mean", type=float, default=3.0, help="mean arcs per walk (normal dist)")
	p.add_argument("--std", type=float, default=1.0, help="std dev of arcs per walk")
	p.add_argument("--max-len", type=int, default=5, help="max arcs per walk")
	p.add_argument("--seed", type=int, default=42)
	args = p.parse_args()

	if args.max_len < 1:
		p.error("require max-len >= 1")
	if args.std < 0:
		p.error("require std >= 0")

	root = args.folder or args.root or os.path.join(here, "C_out")
	if not os.path.isdir(root):
		p.error(f"folder not found: {root}")

	rng = random.Random(args.seed)
	for dirpath, _, files in sorted(os.walk(root)):
		if "links.csv" in files:
			process_file(os.path.join(dirpath, "links.csv"), rng, args.mean, args.std, args.max_len)


if __name__ == "__main__":
	main()
