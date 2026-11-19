# Third-Party Notices

This project includes the following third-party software components.
Each component retains its own license. The MIT license of this project
(`LICENSE`) covers only the original code written by the project author.

## 1. PySide6 (Qt for Python)

- **Purpose**: GUI framework (MainWindow, widgets, Qt signals).
- **License**: GNU Lesser General Public License v3 (LGPL v3); GPL v3 and
  commercial options available from the Qt Company.
- **Source**: The Qt Company (pypi.org/project/PySide6).
- **Key terms**: this project imports PySide6 dynamically (normal Python
  `import` = dynamic linking), so LGPL v3 compliance is satisfied by
  allowing users to replace the PySide6 version (e.g. `pip install -U
  PySide6`). If the GUI is ever distributed as a bundled executable
  (PyInstaller etc.), additional LGPL obligations apply (relinking
  information). Not vendored — installed via `requirements.txt`.

## 2. numpy

- **Purpose**: numeric arrays used by the SDK camera module.
- **License**: BSD-3-Clause.
- **Source**: numpy.org. Not vendored — installed via `requirements.txt`.

## 3. opencv-python

- **Purpose**: video decoding and teammate vision algorithms
  (`peo.cam.read()` returns BGR ndarrays).
- **License**: Apache License 2.0.
- **Source**: opencv.org. Not vendored — installed via `requirements.txt`.

---

Compliance note: no third-party code is vendored in this project; all
dependencies are declared in `requirements.txt` and `pyproject.toml` and
installed by the user's package manager.
