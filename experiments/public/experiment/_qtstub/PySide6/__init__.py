"""Minimal PySide6 stand-in so camera_intrinsics.py imports without Qt.

Only the names the module references at import time are provided, as plain
subclassable objects.  Nothing here is exercised by the headless run: the
comparison calls detect_corners_ex() / calibrate(), which are pure functions.
"""
