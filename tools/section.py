"""剖面圖：在指定位置把組合圖切一刀，看內部配合。

    python -X utf8 tools/section.py STEP/parallel_gripper_assembly_closed.step --y 6
    python -X utf8 tools/section.py STEP/parallel_gripper_body.step --x 0 --z 20

每個零件一個顏色、重疊的地方標紅，圖存到 tmp/<檔名>_section_<軸><位置>.png。
同時印出每個零件在剖面上的範圍、零件之間的重疊面積，以及小於 --clear 的間隙。
做法參考 openscad-skill 的 asm.py --section：內部配合先看剖面數字，比看立體圖可靠也省時間。
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from build123d import Face, Plane, import_step  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
COLORS = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#9D755D", "#72B7B2", "#EECA3B", "#FF9DA6"]
OVERLAP = "#E45756"

# 軸 → (切面, 圖上的橫軸/縱軸取哪兩個座標)
PLANES = {
    "x": (lambda v: Plane((v, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)), ("Y", "Z")),
    "y": (lambda v: Plane((0, v, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0)), ("X", "Z")),
    "z": (lambda v: Plane((0, 0, v), x_dir=(1, 0, 0), z_dir=(0, 0, 1)), ("X", "Y")),
}


def load_parts(path: str) -> dict:
    shape = import_step(path)
    kids = [c for c in getattr(shape, "children", []) if c.solids()]
    if not kids:
        return {Path(path).stem: shape}
    parts = {}
    for i, c in enumerate(kids):
        name = c.label or f"part{i}"
        parts[name if name not in parts else f"{name}_{i}"] = c
    return parts


def cut(parts: dict, axis: str, value: float) -> dict:
    plane = PLANES[axis][0](value)
    knife = Face.make_rect(10000, 10000, plane)
    out = {}
    for name, p in parts.items():
        s = p & knife
        faces = s.faces() if s is not None else []
        if faces:
            out[name] = faces
    return out


def triangles(faces, uv):
    tris = []
    for f in faces:
        verts, idx = f.tessellate(0.02)
        pts = [(getattr(v, uv[0]), getattr(v, uv[1])) for v in verts]
        tris += [[pts[a], pts[b], pts[c]] for a, b, c in idx]
    return tris


def report(sections: dict, uv, clear: float) -> list[str]:
    lines = []
    for name, faces in sections.items():
        bb = faces[0].bounding_box()
        for f in faces[1:]:
            bb = bb.add(f.bounding_box())
        lo, hi = bb.min, bb.max
        lines.append(f"  {name:12} {uv[0]} {getattr(lo, uv[0]):7.2f} ~ {getattr(hi, uv[0]):7.2f}   "
                     f"{uv[1]} {getattr(lo, uv[1]):7.2f} ~ {getattr(hi, uv[1]):7.2f}   面積 {sum(f.area for f in faces):7.1f} mm²")
    for (na, fa), (nb, fb) in itertools.combinations(sections.items(), 2):
        area = sum((a & b).area for a in fa for b in fb if (a & b) is not None)
        if area > 0.01:
            lines.append(f"  ⚠ {na}×{nb} 重疊 {area:.2f} mm²")
            continue
        gap = min(a.distance_to(b) for a in fa for b in fb)
        if gap < clear:
            lines.append(f"  · {na}↔{nb} 間隙 {gap:.2f} mm（小於 {clear} mm）")
    return lines


def draw(sections: dict, uv, title: str, out: Path) -> None:
    plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(9, 7), dpi=110)
    names = list(sections)
    for i, name in enumerate(names):
        tris = triangles(sections[name], uv)
        ax.add_collection(PolyCollection(tris, facecolor=COLORS[i % len(COLORS)], edgecolor=COLORS[i % len(COLORS)], linewidth=0.4,
                                         alpha=0.85, label=name))
    for (na, fa), (nb, fb) in itertools.combinations(sections.items(), 2):
        common = [a & b for a in fa for b in fb]
        faces = [f for c in common if c is not None for f in c.faces()]
        if faces:
            ax.add_collection(PolyCollection(triangles(faces, uv), facecolor=OVERLAP, edgecolor=OVERLAP, linewidth=0.4))
    ax.autoscale()
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_xlabel(f"{uv[0]} (mm)")
    ax.set_ylabel(f"{uv[1]} (mm)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("step")
    for a in "xyz":
        ap.add_argument(f"--{a}", type=float, action="append", default=[], help=f"在 {a.upper()}=值 的位置切（可重複）")
    ap.add_argument("--clear", type=float, default=0.5, help="小於這個值的間隙要列出來（mm）")
    args = ap.parse_args()
    cuts = [(a, v) for a in "xyz" for v in getattr(args, a)]
    if not cuts:
        ap.error("至少要給一個切面，例如 --y 6")
    try:
        parts = load_parts(args.step)
    except Exception as e:  # noqa: BLE001
        print(f"讀不到 {args.step}：{e}")
        return 2
    (ROOT / "tmp").mkdir(exist_ok=True)
    for axis, value in cuts:
        uv = PLANES[axis][1]
        sections = cut(parts, axis, value)
        print(f"剖面 {axis.upper()}={value:g}：")
        if not sections:
            print("  這個位置沒有切到任何零件")
            continue
        for line in report(sections, uv, args.clear):
            print(line)
        out = ROOT / "tmp" / f"{Path(args.step).stem}_section_{axis}{value:g}.png"
        draw(sections, uv, f"{Path(args.step).stem}  剖面 {axis.upper()}={value:g}", out)
        print(f"  圖：{out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
