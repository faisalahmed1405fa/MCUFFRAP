#!/usr/bin/env bash
# Run the MCFStatic solver on the c33 instance with all default parameters.
#
# Usage: ./run_c33.sh
set -euo pipefail

DLL="/storage/home/hcoda1/1/mahmed91/r-phentenryck3-1/MCUFFRAP/bin/Debug/net10.0/MCFStatic.dll"
DATA_DIR="/storage/home/hcoda1/1/mahmed91/r-phentenryck3-1/MCUFFRAP/c33"
LOG="$DATA_DIR/run_c33.log"

dotnet "$DLL" --data-directory="$DATA_DIR" 2>&1 --enable-dual-lb=0