#!/usr/bin/env bash
# Detached driver: pass 1 = 02-22 bag (session D1 files complete), pass 2 = the two 02-23 bags (polls for D2 files).
cd "$(dirname "$0")"
./run_matrix.py --bags 2026-02-22-11-25-57
./run_matrix.py --bags 2026-02-23-11-46-32 2026-02-23-11-57-46
