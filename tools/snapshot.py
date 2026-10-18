"""把 STL 畫成四視圖 PNG（等角、上、前、右），給建模後自我檢查用。

    python tools/snapshot.py STL/l_bracket.stl            # 存到 tmp/l_bracket_views.png
    python tools/snapshot.py STL/l_bracket.stl out.png

正交投影 + 背面剔除 + z-buffer，避免 matplotlib 3D 的排序錯誤。
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

from cadkit import PROJECT_ROOT  # noqa: E402

# (標題, 看過去的方向：從物體指向觀看者, 畫面的上方, 橫軸名稱, 縱軸名稱)
VIEWS = [
    ("等角 (iso)", (1, -1, 0.8), (0, 0, 1), "", ""),
    ("上 (top, 從 +Z 看)", (0, 0, 1), (0, 1, 0), "X (mm)", "Y (mm)"),
    ("前 (front, 從 -Y 看)", (0, -1, 0), (0, 0, 1), "X (mm)", "Z (mm)"),
    ("右 (right, 從 +X 看)", (1, 0, 0), (0, 0, 1), "Y (mm)", "Z (mm)"),
]
LIGHT = np.array([0.5, -0.7, 0.9]) / np.linalg.norm([0.5, -0.7, 0.9])
FEATURE_ANGLE = np.radians(25)
EDGE_SEGMENTS = 8

plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def _basis(view, up):
    d = np.asarray(view, float) / np.linalg.norm(view)
    right = np.cross(up, d)
    right /= np.linalg.norm(right)
    return d, right, np.cross(d, right)


def _rasterize(tris2d, depth, shade, lo, hi, px):
    """簡易 z-buffer：每個像素只留最靠近觀看者的三角形。"""
    scale = px / (hi - lo).max()
    w, h = (np.ceil((hi - lo) * scale).astype(int) + 1)
    zbuf = np.full((h, w), -np.inf)
    img = np.ones((h, w, 4))
    img[..., 3] = 0
    base = np.array([0.91, 0.69, 0.14])
    for tri, z, s in zip((tris2d - lo) * scale, depth, shade):
        x0, y0 = np.floor(tri.min(axis=0)).astype(int)
        x1, y1 = np.ceil(tri.max(axis=0)).astype(int)
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w - 1), min(y1, h - 1)
        if x1 < x0 or y1 < y0:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        (ax_, ay), (bx, by), (cx, cy) = tri
        den = (by - cy) * (ax_ - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-12:
            continue
        l1 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / den
        l2 = ((cy - ay) * (xs - cx) + (ax_ - cx) * (ys - cy)) / den
        l3 = 1 - l1 - l2
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
        zz = l1 * z[0] + l2 * z[1] + l3 * z[2]
        region = zbuf[y0:y1 + 1, x0:x1 + 1]
        win = inside & (zz > region)
        region[win] = zz[win]
        img[y0:y1 + 1, x0:x1 + 1][win] = (*(base * s), 1.0)
    return img, zbuf, scale


def _draw(ax, mesh, view, up, px=520):
    d, right, upv = _basis(view, up)
    front = mesh.face_normals @ d > 1e-9
    tris = mesh.triangles[front]
    tris2d = np.stack([tris @ right, tris @ upv], axis=-1)
    shade = 0.35 + 0.65 * np.clip(mesh.face_normals[front] @ LIGHT, 0, 1)

    pts = mesh.vertices @ np.column_stack([right, upv])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    img, zbuf, scale = _rasterize(tris2d, tris @ d, shade, lo, hi, px)
    span = (hi - lo).max()
    ax.imshow(img, origin="lower", interpolation="antialiased", zorder=2,
              extent=(lo[0], lo[0] + img.shape[1] * span / px, lo[1], lo[1] + img.shape[0] * span / px))

    # 特徵線：相鄰兩面夾角大、至少一面朝向觀看者；切成小段，用 z-buffer 判斷被擋住的段不畫
    sharp = mesh.face_adjacency_angles > FEATURE_ANGLE
    facing = front[mesh.face_adjacency[sharp]].any(axis=1)
    edges = mesh.vertices[mesh.face_adjacency_edges[sharp][facing]]
    if len(edges):
        t = np.linspace(0, 1, EDGE_SEGMENTS + 1)[:, None, None]
        pts3 = edges[:, 0] + t * (edges[:, 1] - edges[:, 0])          # (段數+1, 邊數, 3)
        segs = np.stack([pts3[:-1], pts3[1:]], axis=2).reshape(-1, 2, 3)
        mids = segs.mean(axis=1)
        uv = np.floor((np.column_stack([mids @ right, mids @ upv]) - lo) * scale).astype(int)
        h, w = zbuf.shape
        near = np.full(len(mids), -np.inf)
        for du in (-1, 0, 1):                       # 邊線剛好在輪廓上，取附近 3×3 像素最前面的深度
            for dv in (-1, 0, 1):
                u = np.clip(uv[:, 0] + du, 0, w - 1)
                v = np.clip(uv[:, 1] + dv, 0, h - 1)
                near = np.maximum(near, zbuf[v, u])
        visible = mids @ d >= near - 2.5 / scale
        segs = segs[visible]
        ax.add_collection(LineCollection(np.stack([segs @ right, segs @ upv], axis=-1),
                                         colors="#3a3a3a", linewidths=0.6, zorder=3))

    pad = span * 0.08
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal")


def render(mesh_path: Path, out_path: Path) -> Path:
    mesh = trimesh.load(mesh_path, force="mesh")
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for ax, (title, view, up, xl, yl) in zip(axes.flat, VIEWS):
        _draw(ax, mesh, view, up)
        ax.set_title(title)
        if xl:
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.grid(True, color="#dddddd", linewidth=0.5)
            ax.set_axisbelow(True)
        else:
            ax.set_xticks([])
            ax.set_yticks([])
    size = mesh.extents
    fig.suptitle(f"{mesh_path.name}   {size[0]:.1f} × {size[1]:.1f} × {size[2]:.1f} mm")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=90)
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else PROJECT_ROOT / "tmp" / f"{src.stem}_views.png"
    print(render(src, out))
