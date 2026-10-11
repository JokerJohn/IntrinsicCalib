#!/usr/bin/env python3
"""Write a .intrinsics_session.json session next to a folder of chessboard
images so the calibration tool opens it with the board already set.

The schema is the one camera_intrinsics.py::_save_session writes, so the GUI
restores it verbatim on `Add Folder`:

    board  : pattern / cols / rows / square_mm / tag_spacing / detector
    model  : plumb_bob | rational | fisheye
    fix_k3 : bool  (safe baseline; the tool auto-upgrades if holdout allows)
    images : [{name, checked, detected}]
    result : null   (nothing calibrated yet)

Usage:
    make_gui_session.py <folder> --cols 17 --rows 15 --square-mm 50
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

SESSION_FILE = ".intrinsics_session.json"
# camera_intrinsics.py::load_folder accepts exactly these suffixes
GUI_EXT = (".png", ".jpg", ".jpeg", ".bmp")
DETECTORS = {"auto": 0, "sb": 1, "classic": 2}


def write_session(folder: Path, cols: int, rows: int, square_mm: float,
                  model: str = "plumb_bob", fix_k3: bool = True,
                  detector: str = "auto", pattern: str = "chessboard") -> Path:
    images = sorted(p.name for p in folder.iterdir()
                    if p.suffix.lower() in GUI_EXT)
    if not images:
        raise SystemExit(f"{folder}: no GUI-readable images "
                         f"(need one of {', '.join(GUI_EXT)})")
    data = {
        "board": {
            "pattern": pattern,
            "cols": int(cols),
            "rows": int(rows),
            "square_mm": float(square_mm),
            "tag_spacing": 0.3,
            "detector": DETECTORS[detector],
        },
        "model": model,
        "fix_k3": bool(fix_k3),
        # every image starts checked; Stage 1/Stage 2 decide what to drop
        "images": [{"name": n, "checked": True, "detected": False} for n in images],
        "result": None,
    }
    out = folder / SESSION_FILE
    out.write_text(json.dumps(data, indent=1))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", type=Path)
    ap.add_argument("--cols", type=int, required=True, help="inner corners across")
    ap.add_argument("--rows", type=int, required=True, help="inner corners down")
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--model", default="plumb_bob",
                    choices=["plumb_bob", "rational", "fisheye"])
    ap.add_argument("--free-k3", action="store_true",
                    help="start with k3 free instead of the fix-k3 baseline")
    ap.add_argument("--detector", default="auto", choices=list(DETECTORS))
    args = ap.parse_args()

    out = write_session(args.folder.expanduser().resolve(), args.cols, args.rows,
                        args.square_mm, args.model, not args.free_k3, args.detector)
    n = len(json.loads(out.read_text())["images"])
    print(f"{out}  ({n} images, {args.cols}x{args.rows} inner corners, "
          f"{args.square_mm} mm)")


if __name__ == "__main__":
    main()
