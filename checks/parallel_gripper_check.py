"""平行夾爪檢查：各開合位置的零件干涉、夾片間距、齒輪咬合。

    python -X utf8 checks/parallel_gripper_check.py
"""
import itertools, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import parallel_gripper as pg
from build123d import Rot, Pos

body, cover, pin0, jaw0 = pg.body(), pg.cover(), pg.pinion(), pg.jaw()
ok = True
for opening in [i / 8 * 2 * pg.STROKE for i in range(9)]:
    shift = opening / 2
    angle = math.degrees(shift / pg.PITCH_R)
    parts = {
        "body": body, "cover": cover,
        "pinion": Rot(0, angle, 0) * pin0,
        "jaw_top": Pos(shift, 0, 0) * jaw0,
        "jaw_bottom": Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw0,
    }
    clashes = []
    for (a, pa), (b, pb) in itertools.combinations(parts.items(), 2):
        v = (pa & pb).volume
        if v > 0.5:
            clashes.append(f"{a}×{b} {v:.1f}mm³")
    # 夾片內側面距離
    gap = (parts["jaw_top"].bounding_box().min.X - parts["jaw_bottom"].bounding_box().max.X)
    fa = [f for f in parts["jaw_top"].faces() if abs(f.normal_at().X + 1) < 1e-6 and f.area > 1000]
    fb = [f for f in parts["jaw_bottom"].faces() if abs(f.normal_at().X - 1) < 1e-6 and f.area > 1000]
    face_gap = min(f.center().X for f in fa) - max(f.center().X for f in fb)
    status = "OK" if not clashes else "干涉"
    ok &= not clashes
    print(f"開口 {opening:6.1f} mm | 夾片間距 {face_gap:6.1f} mm | {status} {' '.join(clashes)}")
sys.exit(0 if ok else 1)
