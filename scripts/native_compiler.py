import os
import shutil
from pathlib import Path

Import("env")

pio_home = Path(os.environ.get("PLATFORMIO_CORE_DIR",
                              Path(os.path.expanduser("~")) / ".platformio"))
mingw_bin = pio_home / "packages" / "toolchain-gccmingw32" / "bin"

if mingw_bin.is_dir():
    os.environ["PATH"] = str(mingw_bin) + os.pathsep + os.environ.get("PATH", "")
    env.PrependENVPath("PATH", str(mingw_bin))

    build_dir = Path(env.subst("$BUILD_DIR"))
    build_dir.mkdir(parents=True, exist_ok=True)
    for dll in mingw_bin.glob("*.dll"):
        shutil.copy2(dll, build_dir / dll.name)
