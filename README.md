# MCUFFRAP
Multi-commodity unsplittable flow fixed resource activation problem

## Requirements

- Python 3.8+ (standard library only) for the data scripts.
- .NET 10 SDK and a valid [Gurobi](https://www.gurobi.com/) 13 license for the optimizer (`Gurobi.Optimizer` NuGet package).

## 1. Generating the data with `DataTransform.py`

The base fixed-charge multi-commodity flow (FCMCF) instances are the Canad instances (`C`, `C+`, `R`)
from the [Commalab MMCF datasets page](https://commalab.di.unipi.it/datasets/mmcf/#Linear).

1. Download and extract the instance archive(s) from the website. Each family should end up in its own
   directory of MULTIGEN `.dow` files (e.g. `C/`, `CPlus/`, `R/`). Canadian-format `.dat` files
   (e.g. `LargeMulgen`) are also supported.
2. Convert them to the CSV format used by the optimizer:

   ```bash
   # One or more input directories containing .dow / .dat files
   python DataTransform.py path/to/C path/to/CPlus path/to/R

   # Default (no arguments): converts CPlus/ and R/ located beside the script
   python DataTransform.py
   ```

   Options:

   | Option | Description |
   |---|---|
   | `input_dirs` (positional) | Directories with `.dow`/`.dat` files. Default: `CPlus` and `R` beside the script. |
   | `--output-root <dir>` | Parent folder of the generated `<input-dir-name>_out` folders. Default: the parent of each input directory. |
   | `--overwrite` | Replace previously generated CSVs. |

For every input file `X.dow`, a folder `<input-dir-name>_out/X/` is created containing:

- `links.csv` – `link_id, source, target, length_km, delay_ms, capacity_gbps, fixed_cost, usage_cost_per_gbps`
- `nodes.csv` – `node_id, name, lat, lng, population`
- `requests.csv` – `request_id, source, target, demand_gbps, max_ping_ms, revenue, shortest_path_delay_ms`
- `output/` – empty folder where the optimizer writes its results

Fields not available in the original instances (geography, delay, revenue) are left empty; no units are rescaled.

## 2. Transforming FCMCF instances to MCUFFRAP instances with `assign_resources.py`

MCUFFRAP groups arcs into *resources* that are activated together. `assign_resources.py` partitions the arcs of
every `links.csv` found (recursively) under a folder into resources using random walks:

1. pick a random node that still has unassigned outgoing arcs;
2. walk along unassigned outgoing arcs for `L = round(Normal(mean, std))` steps, clipped to `[1, max-len]`
   (the walk stops early if no unassigned outgoing arc leads to an unvisited node);
3. assign all traversed arcs to a new resource.

This repeats until every arc belongs to exactly one resource. A `resource` column is added to (or overwritten in)
each `links.csv` in place.

For the resource generation used in the paper, set `mean=8`, `std=1`, and
`max-len=11`:

```bash
python assign_resources.py C_out --mean 8 --std 1 --max-len 11 --seed 42
```

The table below gives the script defaults; use the paper values above when
generating instances to reproduce the paper results.

| Option | Default | Description |
|---|---|---|
| `folder` (positional) / `--root` | `C_out` beside the script | Folder searched recursively for `links.csv` files |
| `--mean` | `3.0` | Mean number of arcs per walk (resource) |
| `--std` | `1.0` | Standard deviation of arcs per walk |
| `--max-len` | `5` | Maximum number of arcs per walk |
| `--seed` | `42` | Random seed (for reproducibility) |

The instance in [c33/](c33) is an example of a resulting MCUFFRAP instance.

## 3. Running the optimizer

Build the project:

```bash
dotnet build -c Release
```

Run it on an instance folder (the folder containing `links.csv` and `requests.csv`):

```bash
dotnet bin/Release/net10.0/MCFStatic.dll --data-directory=c33 --enable-dual-lb=0
# or
dotnet run -c Release -- --data-directory c33 --enable-dual-lb 0
```

All logs and result CSVs (`phase1.*`, `phase2.*`, `cg_phase_times.csv`, `weaklp.*`, `weaklp_si.*`, ...) are written
to `<data-directory>/output/`. See [run.sh](run.sh) for an example. Use `--help` to print the usage.

Options can be given as named arguments (`--name value` or `--name=value`) or positionally in the order of the
table below (do not mix the two styles). Boolean options accept `0/1` or `false/true`.

| # | Option | Default | Description |
|---|---|---|---|
| 0 | `--data-directory <path>` | *(required)* | Instance folder containing `links.csv` and `requests.csv` |
| 1 | `--network-kind <static\|tele>` | `static` | Network type |
| 2 | `--cg-mode <leg\|legrequest>` | `legrequest` | Column generation master formulation |
| 3 | `--max-runtime-seconds <s>` | `600` | Time limit (arc MIP and weak LP) |
| 4 | `--run-arc-mip <bool>` | `false` | Solve the arc-based MIP |
| 5 | `--run-cg <bool>` | `true` | Run column generation |
| 6 | `--enable-dual-lb <0..4>` | `1` | Policy for estimating duals of missing (request, arc) linking constraints: `0` off; `1` equal split over all absent pairs on the arc; `2` hybrid: `0` until the Lagrangian dual bound stalls, then `1`; `3` alternate `0→1` on dual-bound stalls and `1→0` on master-LP stalls; `4` `1` until the master LP stalls, then `0` |
| 7 | `--cg-integer-mip-interval <int>` | `50` | CG iterations between integer master MIP solves |
| 8 | `--cg-optimality-tol <x>` | `0.0001` | CG (phase 2) optimality tolerance |
| 9 | `--branch-and-price <bool>` | `false` | Run the branch-and-price dive instead of plain CG |
| 10 | `--branch-depth <int>` | `5` | Maximum branch-and-price dive depth |
| 11 | `--cg-num-paths <int>` | `1` | Paths priced per request in each pricing round |
| 12 | `--run-weak-lp <bool>` | `true` | Solve the weak LP relaxation |
| 13 | `--root-relaxation-only <bool>` | `true` | Only solve the CG phase 1/phase 2 root LP relaxations (no integer solution) |
| 14 | `--single-threaded <bool>` | `false` | Run single-threaded |
| 15 | `--legreq-idle-limit <int>` | `0` | Rounds an unused (request, arc) linking constraint stays in the LegRequest master before removal (negative: never remove) |
| 16 | `--cg-max-iterations <int>` | `200` | Maximum phase 2 CG iterations |
| 17 | `--hybrid-switch-tolerance <x>` | `0.01` | Relative improvement (1%) below which a bound is considered stalled (policies 2, 3, 4) |
| 18 | `--hybrid-switch-stall-iters <int>` | `3` | Look-back window (iterations) for the stall test (policies 2, 3, 4) |
| 19 | `--weak-lp-si-cuts <0\|1\|2>` | `2` | Weak LP without (`0`), with (`1`) or both without and with (`2`/`both`) the SI cuts `x[a,r] <= y_s(a)` |
| 20 | `--cg-css-heuristic <bool>` | `true` | Start pricing with the SP-CSS heuristic before switching to the pricing MIP |
| 21 | `--column-idle-limit <int>` | `2` | Extra rounds an unused column stays in the master before removal (negative: never remove) |
| 22 | `--lp-method <-1..5>` | `0` | Gurobi `Method` for the CG restricted master LP |
| 23 | `--weak-lp-method <-1..5>` | `-1` | Gurobi `Method` for the weak LP relaxations |
| 24 | `--pricing-mip-workers <int>` | `4` | Parallel pricing MIP workers (one Gurobi env each; `0` = sequential) |

### Reproducing the paper results

Use the default value for every parameter and only vary `--enable-dual-lb` over `0`, `1`, `2` and `3`:

```bash
for policy in 0 1 2 3; do
  dotnet bin/Release/net10.0/MCFStatic.dll --data-directory=<instance-dir> --enable-dual-lb=$policy
done
```

Note that each run writes to `<instance-dir>/output/`, so copy/rename that folder between runs to keep the results
of every policy.
