"""產生列印資料夾：每個零件轉成列印方向、排成一盤，再附上零件清單。

做法參考 openscad-skill 的 asm.py --print / --bom：零件標好列印方向（或自動挑支撐最少的），
一次輸出可以直接丟進切片軟體的檔案，以及要印、要買的東西清單。

    from cadkit.printprep import save_print
    save_print("parallel_gripper",
               [("機身", body(), 1, Rot(0, 0, 0)), ("夾爪", jaw(), 2, "auto")],
               bom=[("M4×12 內六角螺絲", 4, "手腕舵盤"), ...])

輸出到 PRINT/<名稱>/：
    <零件>.stl            已轉好方向、放在平台上（Z=0）
    <名稱>_plate.3mf      所有零件（含數量）排在同一盤
    <名稱>_零件清單.csv    要印的和要買的
"""

from __future__ import annotations

import csv

import numpy as np
import trimesh
from build123d import Mesher, Pos, Rot

from cadkit.export import PROJECT_ROOT, _export_stl
from cadkit.printer import PROFILE

ORIENTATIONS = {
    "原本方向": Rot(0, 0, 0),
    "繞 X 轉 90°": Rot(90, 0, 0),
    "繞 X 轉 -90°": Rot(-90, 0, 0),
    "繞 X 轉 180°": Rot(180, 0, 0),
    "繞 Y 轉 90°": Rot(0, 90, 0),
    "繞 Y 轉 -90°": Rot(0, -90, 0),
}
PLATE_GAP = 8.0          # 盤上零件之間的距離（mm）
DEFAULT_BED = (220.0, 220.0)   # 還沒設定印表機時，排盤用的寬度


def _mesh(shape) -> trimesh.Trimesh:
    verts, tris = shape.tessellate(0.05, 0.2)
    return trimesh.Trimesh([(v.X, v.Y, v.Z) for v in verts], tris)


def support_area(shape) -> float:
    """需要支撐的面積（mm²）：朝下、超過 PROFILE 懸空角度、又不貼在平台上的面。"""
    m = _mesh(shape)
    nz = m.face_normals[:, 2]
    angle = 90.0 - np.degrees(np.arcsin(np.clip(-nz, 0, 1)))
    on_plate = m.triangles_center[:, 2] < m.bounds[0][2] + 0.1
    needs = (nz < -1e-6) & ~on_plate & (angle < PROFILE["overhang_deg"])
    return float(m.area_faces[needs].sum())


def on_bed(shape):
    bb = shape.bounding_box()
    return Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * shape


def orient(shape, rot) -> tuple:
    """回傳 (轉好放在平台上的零件, 方向名稱, 支撐面積)。rot="auto" 會比較 6 個方向。"""
    if rot == "auto":
        rows = []
        for name, r in ORIENTATIONS.items():
            s = on_bed(r * shape)
            rows.append((support_area(s), s.bounding_box().size.Z, name, s))
        area, _, name, s = min(rows, key=lambda r: (round(r[0], 1), r[1]))
        return s, f"自動：{name}", area
    s = on_bed(rot * shape)
    return s, "指定", support_area(s)


def _arrange(shapes: list, bed_w: float) -> list:
    """由左到右排，超過平台寬度就換下一排。"""
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    for s in shapes:
        size = s.bounding_box().size
        if x > 0 and x + size.X > bed_w:
            x, y, row_h = 0.0, y + row_h + PLATE_GAP, 0.0
        placed.append(Pos(x, y, 0) * s)
        x += size.X + PLATE_GAP
        row_h = max(row_h, size.Y)
    return placed


def save_print(name: str, items: list, bom: list | None = None) -> dict:
    """items：[(零件名, 零件, 數量, 方向)]，方向是 Rot(...) 或 "auto"。
    bom：要買的東西 [(名稱, 數量, 用途)]。"""
    folder = PROJECT_ROOT / "PRINT" / name
    folder.mkdir(parents=True, exist_ok=True)
    plate, rows = [], []
    print(f"列印資料夾 PRINT/{name}/")
    for part_name, shape, qty, rot in items:
        s, how, area = orient(shape, rot)
        _export_stl(s, folder / f"{part_name}.stl")
        size = s.bounding_box().size
        support = "不用支撐" if area < 1 else f"支撐約 {area:.0f} mm²"
        print(f"  {part_name} ×{qty}：{size.X:.0f}×{size.Y:.0f}×{size.Z:.0f} mm，{how}，{support}")
        rows.append(("列印", part_name, qty, f"{size.X:.0f}×{size.Y:.0f}×{size.Z:.0f} mm，{support}"))
        plate += [s] * qty

    bed = PROFILE["bed_mm"] or DEFAULT_BED
    placed = _arrange(plate, bed[0])
    mesher = Mesher()
    for s in placed:
        mesher.add_shape(s)
    mesher.write(str(folder / f"{name}_plate.3mf"))
    total = placed[0].bounding_box()
    for s in placed[1:]:
        total = total.add(s.bounding_box())
    fits = "" if total.size.X <= bed[0] and total.size.Y <= bed[1] else "（超過平台，要分兩次印）"
    note = "" if PROFILE["bed_mm"] else f"，還沒設定印表機，先用 {bed[0]:.0f}×{bed[1]:.0f} 排"
    print(f"  整盤 {name}_plate.3mf：{total.size.X:.0f}×{total.size.Y:.0f} mm{fits}{note}")

    for item, qty, use in bom or []:
        rows.append(("購買", item, qty, use))
    with open(folder / f"{name}_零件清單.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["類別", "名稱", "數量", "說明"])
        w.writerows(rows)
    if bom:
        print("  要買的：" + "、".join(f"{i} ×{q}" for i, q, _ in bom))
    return {"folder": folder, "rows": rows}
