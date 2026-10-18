"""在瀏覽器裡 360° 預覽 STEP 檔（OCP CAD Viewer 獨立版）。

    python tools/view.py STEP/l_bracket.step                 # 預覽一個零件
    python tools/view.py STEP/a.step STEP/b.step             # 一起預覽
    python tools/view.py --stop                              # 關掉預覽器

第一次執行會在背景啟動預覽器，網址：http://127.0.0.1:3939/viewer
"""

from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

PORT = 3939
URL = f"http://127.0.0.1:{PORT}/viewer"
LOG = Path.home() / ".cad3d" / "viewer.log"


def running() -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", PORT)) == 0


def start() -> None:
    if running():
        return
    LOG.parent.mkdir(parents=True, exist_ok=True)
    flags = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
    with open(LOG, "w", encoding="utf-8") as log:
        subprocess.Popen(
            [sys.executable, "-m", "ocp_viewer", "--port", str(PORT), "--axes0", "--grid_xy",
             "--theme", "light", "--reset_camera", "reset"],
            creationflags=flags, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
        )
    for _ in range(120):
        if running():
            return
        time.sleep(0.5)
    raise RuntimeError(f"預覽器 60 秒內沒有啟動，請看 {LOG}")


def stop() -> None:
    subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*-m ocp_viewer*' } "
         "| ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"],
        check=False,
    )


def show(paths: list[Path]) -> None:
    from build123d import import_step
    from ocp_viewer import set_port, show as ocp_show

    shapes, names = [], []
    for p in paths:
        shape = import_step(str(p))
        shapes.append(shape)
        names.append(p.stem)
    set_port(PORT)
    ocp_show(*shapes, names=names)


if __name__ == "__main__":
    args = sys.argv[1:]
    if args == ["--stop"]:
        stop()
        print("預覽器已關閉")
    elif not args:
        sys.exit(__doc__)
    else:
        start()
        show([Path(a) for a in args])
        print(f"已送到預覽器：{URL}")
