"""3D 列印檢查（DFM）：量測 STL/3MF，跟 cadkit.printer.PROFILE 的限制比較。

    python tools/dfm_check.py STL/l_bracket.stl
    python tools/dfm_check.py STL/l_bracket.stl --json

做法參考 text-to-cad 的 dfam-check（MIT）：工具只量測事實，判斷依據寫在 PROFILE。
量不到的項目標 ❓，不會當成通過。

結束代碼：0 全部通過、1 有不通過的項目、2 檔案讀不到。
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np
import trimesh

from cadkit.printer import PROFILE

PLATE_TOL_MM = 0.1


def load_mesh(path: str) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)
    return mesh


def support_area(mesh: trimesh.Trimesh, angle_limit: float) -> tuple[float, np.ndarray]:
    """朝下、不貼平台、且角度小於 angle_limit 的面積（需要支撐）。"""
    nz = mesh.face_normals[:, 2]
    down = nz < -1e-6
    angle = 90.0 - np.degrees(np.arcsin(np.clip(-nz, 0, 1)))  # 0 = 天花板，90 = 垂直牆
    on_plate = mesh.triangles_center[:, 2] < mesh.bounds[0][2] + PLATE_TOL_MM
    needs = down & ~on_plate & (angle < angle_limit)
    return float(mesh.area_faces[needs].sum()), needs


def wall_thickness(mesh: trimesh.Trimesh, samples: int) -> dict:
    """在表面隨機取點，沿法線往內打射線，量到對面的距離。"""
    points, faces = trimesh.sample.sample_surface(mesh, samples, seed=0)
    normals = mesh.face_normals[faces]
    origins = points - normals * 1e-3
    hits, ray_idx, _ = mesh.ray.intersects_location(origins, -normals, multiple_hits=False)
    if len(hits) == 0:
        return {"error": "射線沒有打到任何面"}
    dist = np.linalg.norm(hits - origins[ray_idx], axis=1)
    order = np.argsort(dist)
    return {
        "samples": int(len(dist)),
        "min_mm": round(float(dist[order[0]]), 2),
        "p05_mm": round(float(np.percentile(dist, 5)), 2),
        "median_mm": round(float(np.median(dist)), 2),
        "thinnest_at": [[round(float(v), 1) for v in points[ray_idx[i]]] for i in order[:3]],
    }


ORIENTATIONS = {
    "原本方向": np.eye(4),
    "繞 X 轉 90°": trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]),
    "繞 X 轉 180°": trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]),
    "繞 X 轉 -90°": trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]),
    "繞 Y 轉 90°": trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]),
    "繞 Y 轉 -90°": trimesh.transformations.rotation_matrix(-np.pi / 2, [0, 1, 0]),
}


def orientations(mesh: trimesh.Trimesh, angle_limit: float) -> list[dict]:
    rows = []
    for name, matrix in ORIENTATIONS.items():
        m = mesh.copy()
        m.apply_transform(matrix)
        area, _ = support_area(m, angle_limit)
        rows.append({"name": name, "support_mm2": round(area, 1), "height_mm": round(float(m.extents[2]), 1)})
    return sorted(rows, key=lambda r: (r["support_mm2"], r["height_mm"]))


def measure(path: str, samples: int) -> dict:
    mesh = load_mesh(path)
    limit = PROFILE["overhang_deg"]
    area, _ = support_area(mesh, limit)
    return {
        "file": path,
        "bbox_mm": [round(float(v), 2) for v in mesh.extents],
        "volume_cm3": round(float(abs(mesh.volume)) / 1000, 2) if mesh.is_volume else None,
        "watertight": bool(mesh.is_watertight),
        "body_count": int(mesh.body_count),
        "support_area_mm2": round(area, 1),
        "wall": wall_thickness(mesh, samples) if mesh.is_watertight else {"error": "模型有破洞，無法量牆厚"},
        "orientations": orientations(mesh, limit),
    }


def judge(facts: dict) -> list[tuple[str, str, str]]:
    """回傳 (狀態, 項目, 說明)。狀態：✅ 通過、❌ 不通過、❓ 需要更多資訊。"""
    out = []
    if facts["watertight"]:
        out.append(("✅", "模型封閉", "沒有破洞，可以切片"))
    else:
        out.append(("❌", "模型封閉", "模型有破洞，切片軟體會出錯，要先修模型"))

    if facts["body_count"] > 1:
        out.append(("❓", "實體數", f"有 {facts['body_count']} 個分開的實體，確認是不是故意的"))

    wall = facts["wall"]
    lim = PROFILE["min_wall_mm"]
    if "error" in wall:
        out.append(("❓", "牆厚", wall["error"]))
    elif wall["p05_mm"] < lim:
        out.append(("❌", "牆厚", f"最薄 {wall['min_mm']} mm、5% 分位 {wall['p05_mm']} mm，低於 {lim} mm；"
                                  f"最薄處約在 {wall['thinnest_at'][0]}"))
    elif wall["min_mm"] < lim:
        out.append(("❓", "牆厚", f"5% 分位 {wall['p05_mm']} mm 合格，但有取樣點只有 {wall['min_mm']} mm"
                                  f"（約在 {wall['thinnest_at'][0]}），可能是倒角或邊緣"))
    else:
        out.append(("✅", "牆厚", f"最薄 {wall['min_mm']} mm（標準 ≥ {lim} mm）"))

    area = facts["support_area_mm2"]
    best = facts["orientations"][0]
    if area == 0:
        out.append(("✅", "懸空", f"沒有超過 {PROFILE['overhang_deg']:.0f}° 的懸空，不需要支撐"))
    else:
        note = f"需要支撐的面積約 {area} mm²"
        if best["support_mm2"] < area * 0.5:
            note += f"；改成「{best['name']}」可降到 {best['support_mm2']} mm²（高度 {best['height_mm']} mm）"
        out.append(("❓", "懸空", note))

    bed = PROFILE["bed_mm"]
    if bed is None:
        out.append(("❓", "平台尺寸", f"還沒設定印表機；零件大小 {facts['bbox_mm']} mm"))
    elif any(s > b for s, b in zip(sorted(facts["bbox_mm"]), sorted(bed))):
        out.append(("❌", "平台尺寸", f"零件 {facts['bbox_mm']} mm 超過平台 {bed} mm"))
    else:
        out.append(("✅", "平台尺寸", f"零件 {facts['bbox_mm']} mm 放得下"))

    out.append(("❓", "孔徑 / 細小特徵 / 橋接", "這個工具量不到，請看建模程式裡的尺寸"
                f"（標準：孔 ≥ {PROFILE['min_hole_mm']} mm、特徵 ≥ {PROFILE['min_feature_mm']} mm、"
                f"橋接 ≤ {PROFILE['max_bridge_mm']} mm）"))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mesh")
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--json", action="store_true", help="輸出原始量測資料")
    args = parser.parse_args()

    try:
        facts = measure(args.mesh, args.samples)
    except Exception as exc:
        print(f"讀不到檔案：{exc}")
        return 2

    if args.json:
        print(json.dumps(facts, ensure_ascii=False, indent=2))
        return 0

    v = facts["volume_cm3"]
    print(f"檔案：{facts['file']}")
    print(f"尺寸：{facts['bbox_mm']} mm，體積：{v if v is not None else '?'} cm³")
    print(f"標準：一般 FDM（{PROFILE['nozzle_mm']} mm 噴頭），見 lib/cadkit/printer.py\n")
    results = judge(facts)
    for status, item, note in results:
        print(f"{status} {item}：{note}")
    return 1 if any(s == "❌" for s, _, _ in results) else 0


if __name__ == "__main__":
    sys.exit(main())
