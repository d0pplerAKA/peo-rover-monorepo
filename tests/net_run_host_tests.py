"""Build/run real production net/parser/macro/telemetry logic, no IDF build."""
import os
import pathlib
import subprocess
import tempfile
import sys
import shutil
ROOT = pathlib.Path(__file__).resolve().parents[1]
CXX = os.environ.get("CXX") or shutil.which("g++")
if not CXX:
    raise SystemExit("A host C++17 g++ compiler is required (set CXX or PATH).")
CJSON = pathlib.Path(os.environ.get("CJSON_DIR", str(ROOT / "managed_components/espressif__cjson/cJSON")))
scratch = pathlib.Path(os.environ.get("HERMES_NET_TEST_TMP") or os.environ.get("TMPDIR") or tempfile.gettempdir())
scratch.mkdir(parents=True, exist_ok=True)
os.environ["TMP"] = os.environ["TEMP"] = os.environ["TMPDIR"] = str(scratch)
with tempfile.TemporaryDirectory(prefix="net-host-", dir=scratch) as tmp:
    exe = pathlib.Path(tmp) / "net_host_test.exe"
    args = [CXX,"-std=c++17","-DCJSON_NESTING_LIMIT=16","-O0","-g","-Wall","-Wextra","-Werror","-Wno-unused-parameter","-Wno-unused-variable","-Wno-unused-but-set-variable", "-I"+str(ROOT/"tests/stubs_net"),"-I"+str(CJSON)]
    for component in ["esp_net","esp_link","macro_engine","cmd_parser","telemetry","c3_link","camera_service"]:
        args.append("-I"+str(ROOT/"components"/component/"include"))
    # Each production TU must compile on its own, not just via the combined harness.
    for component in ["esp_net", "cmd_parser", "macro_engine", "telemetry"]:
        subprocess.run(args + ["-fsyntax-only", str(ROOT/"components"/component/(component+".cpp"))], check=True)
    args += [str(ROOT/"tests/net_host_test.cpp"),str(CJSON/"cJSON.c"),"-static-libgcc","-static-libstdc++","-o",str(exe)]
    subprocess.run(args,check=True)
    env=os.environ.copy();env["PATH"]=str(pathlib.Path(CXX).parent)+os.pathsep+env.get("PATH","")
    discovered = subprocess.check_output([str(exe),"--list"], env=env, text=True).splitlines()
    assert discovered and len(discovered) == len(set(discovered))
    tests = sys.argv[1:] or discovered
    failed=0
    for test in tests:
        failed += subprocess.run([str(exe),test],env=env).returncode != 0
    print(f"{len(tests)-failed}/{len(tests)} host regressions passed",flush=True)
    sys.exit(bool(failed))
