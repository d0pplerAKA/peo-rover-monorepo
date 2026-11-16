"""Validate the built firmware, not just a successful build command.
Run with the ESP-IDF Python environment.
"""
from pathlib import Path
import hashlib
import sys

ROOT = Path(__file__).resolve().parents[1]
failures = []

def check(ok, label):
    print(('PASS ' if ok else 'FAIL ') + label)
    if not ok:
        failures.append(label)

config = {}
for line in (ROOT / 'sdkconfig').read_text(encoding='utf-8').splitlines():
    if line.startswith('CONFIG_') and '=' in line:
        k, v = line.split('=', 1)
        config[k] = v
for key, value in {
    'CONFIG_IDF_TARGET': '"esp32s3"',
    'CONFIG_ESPTOOLPY_FLASHSIZE': '"8MB"',
    'CONFIG_ESPTOOLPY_FLASHMODE': '"dio"',
    'CONFIG_SPIRAM': 'y',
    'CONFIG_SPIRAM_MODE_OCT': 'y',
    'CONFIG_SPIRAM_SPEED_80M': 'y',
    'CONFIG_SPIRAM_BOOT_INIT': 'y',
    'CONFIG_SPIRAM_MEMTEST': 'y',
}.items():
    check(config.get(key) == value, key + '=' + value)
# Kconfig output, not merely the requested source configuration.
import json
effective = json.loads((ROOT/'build/config/sdkconfig.json').read_text(encoding='utf-8'))
for key, value in {'SPIRAM': True, 'SPIRAM_MODE_OCT': True, 'SPIRAM_SPEED': 80,
                   'ESPTOOLPY_FLASHSIZE': '8MB', 'ESPTOOLPY_FLASHMODE': 'dio',
                   'CJSON_NESTING_LIMIT': 16}.items():
    check(effective.get(key) == value, 'effective ' + key + '=' + str(value))
check(config.get('CONFIG_ESPTOOLPY_OCT_FLASH') != 'y', 'N8R8 Quad flash is NOT configured as Octal flash')
print('ELF_SHA256=' + hashlib.sha256((ROOT/'build/peo_rover_esp32s3.elf').read_bytes()).hexdigest())
raise SystemExit(1 if failures else 0)
