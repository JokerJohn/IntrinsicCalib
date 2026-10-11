#!/usr/bin/env python3
"""Open the camera-intrinsics calibration window on a folder of images.

    python tool/run_intrinsics_gui.py [folder]

The folder holds the chessboard (or AprilGrid) images and, optionally, a
``.intrinsics_session.json`` session file (experiments/public/make_gui_session.py
writes one).  Without a folder argument the window opens empty; use
``Add Folder``.  Detection, the two-stage image selection, the distortion-order
selection and the export of ``camera_intrinsics.yaml`` / ``ost.yaml`` are
driven from the window.
"""
import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PySide6.QtWidgets import QApplication  # noqa: E402
from intrinsic_calib.camera_intrinsics import IntrinsicsDialog  # noqa: E402


def main() -> None:
    folder = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else None
    app = QApplication(sys.argv)
    dlg = IntrinsicsDialog(current_dataset_dir=str(folder.parent) if folder else None)
    dlg.show()
    if folder is not None:
        dlg.load_folder(folder)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
