"""Build and run the real esp_link.cpp regression tests with host adapters."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = os.environ.get('CXX') or shutil.which('g++')
if not compiler:
    raise SystemExit('A host C++17 compiler is required (set CXX or put g++ on PATH).')
with tempfile.TemporaryDirectory(prefix='peo-link-', dir=os.environ.get('TMPDIR')) as tmp:
    exe = Path(tmp) / ('link_test.exe' if os.name == 'nt' else 'link_test')
    command = [compiler, '-std=c++17', '-Wall', '-Wextra',
               '-Wno-unused-parameter', '-Wno-unused-variable',
               '-I', str(root/'tests/stubs_link'), '-I', str(root/'components/esp_link/include'),
               '-I', str(root/'components/macro_engine/include'),
               str(root/'tests/link_regression.cpp'), '-o', str(exe)]
    if os.name == 'nt':
        command.insert(1, '-static')
    subprocess.run(command, check=True)
    subprocess.run([str(exe)], check=True)
