"""把零件存成 STEP / STL / 3MF。

build123d 的 STL 匯出遇到含中文的路徑（例如「桌面」）會失敗，
所以 STL 先存到純英文路徑的暫存資料夾，再搬回專案資料夾。
"""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from build123d import Mesher, export_step, export_stl

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ASCII_TMP = Path.home() / ".cad3d" / "tmp"


def _export_stl(shape, target: Path) -> None:
    ASCII_TMP.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ASCII_TMP) as tmp:
        staging = Path(tmp) / "part.stl"
        if not export_stl(shape, str(staging)):
            raise RuntimeError(f"STL 匯出失敗：{target.name}")
        shutil.move(staging, target)


def save(shape, name: str, formats=("step", "stl", "3mf")) -> dict[str, Path]:
    """存到 STEP/、STL/、3MF/，檔名為 name，並印出尺寸摘要。"""
    if shape is None or shape.volume <= 0:
        raise ValueError(f"{name}：形狀是空的或體積不是正的，無法存檔")

    written = {}
    for fmt in formats:
        folder = PROJECT_ROOT / fmt.upper()
        folder.mkdir(exist_ok=True)
        target = folder / f"{name}.{fmt}"
        if fmt == "step":
            if not export_step(shape, str(target)):
                raise RuntimeError(f"STEP 匯出失敗：{target.name}")
        elif fmt == "stl":
            _export_stl(shape, target)
        elif fmt == "3mf":
            mesher = Mesher()
            mesher.add_shape(shape)
            mesher.write(str(target))
        else:
            raise ValueError(f"不支援的格式：{fmt}")
        written[fmt] = target

    size = shape.bounding_box().size
    print(f"{name}: {size.X:.1f} x {size.Y:.1f} x {size.Z:.1f} mm, "
          f"體積 {shape.volume / 1000:.2f} cm³, 實體數 {len(shape.solids())}")
    for fmt, path in written.items():
        print(f"  {fmt.upper():4} {path.relative_to(PROJECT_ROOT)}")
    return written
