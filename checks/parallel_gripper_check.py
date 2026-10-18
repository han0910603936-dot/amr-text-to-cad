"""平行夾爪檢查：各開合位置的零件干涉、夾片間距，以及整體重心（手腕舵盤要在重心下方）。

    python -X utf8 checks/parallel_gripper_check.py

伺服馬達和舵盤用簡化方塊/圓盤代替，一起檢查會不會撞到。
重心估算：列印件 PLA 1.24 g/cm³ × 有效填充 45%，伺服馬達 55 g，MATRIX 舵盤 8 g。
"""
import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import parallel_gripper as pg  # noqa: E402
from build123d import Align, Cylinder, Pos, Rot  # noqa: E402

box_, cover_, pin0, jaw0 = pg.body_box(), pg.cover(), pg.pinion(), pg.jaw()

# 夾爪伺服馬達假人：固定耳貼背板背面，本體往後、往上
y_back = pg.BACK_WALL_BACK_Y
servo = (pg.box(-pg.SERVO_W / 2, pg.SERVO_W / 2, y_back - 2.5, y_back, pg.SERVO_CZ - 27.25, pg.SERVO_CZ + 27.25)
         + pg.box(-pg.SERVO_W / 2, pg.SERVO_W / 2, y_back - 30, y_back - 2.5,
                  pg.SERVO_CZ - pg.SERVO_L / 2, pg.SERVO_CZ + pg.SERVO_L / 2))
horn = Pos(0, -0.01, 0) * Rot(90, 0, 0) * Cylinder(pg.HORN_D / 2, pg.HORN_T, align=(Align.CENTER, Align.CENTER, Align.MIN))

ok = True
for i in range(9):
    opening = i / 8 * 2 * pg.STROKE
    shift = opening / 2
    angle = math.degrees(shift / pg.PITCH_R)
    parts = {
        "box": box_, "cover": cover_, "servo": servo,
        "pinion": Rot(0, angle, 0) * pin0, "horn": Rot(0, angle, 0) * horn,
        "jaw_top": Pos(shift, 0, 0) * jaw0,
        "jaw_bottom": Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw0,
    }
    allowed = {("pinion", "horn"), ("servo", "box")}   # 鎖在一起 / 伺服穿過背板（另外檢查）
    clashes = []
    for (a, pa), (b, pb) in itertools.combinations(parts.items(), 2):
        if (a, b) in allowed:
            continue
        v = (pa & pb).volume
        if v > 0.5:
            clashes.append(f"{a}×{b} {v:.1f}mm³")
    fa = [f for f in parts["jaw_top"].faces() if abs(f.normal_at().X + 1) < 1e-6 and f.area > 1000]
    fb = [f for f in parts["jaw_bottom"].faces() if abs(f.normal_at().X - 1) < 1e-6 and f.area > 1000]
    gap = min(f.center().X for f in fa) - max(f.center().X for f in fb)
    ok &= not clashes
    print(f"開口 {opening:6.1f} mm | 夾片間距 {gap:6.1f} mm | {'OK' if not clashes else '干涉'} {' '.join(clashes)}")

servo_clash = (servo & box_).volume
print(f"伺服馬達與長盒重疊 {servo_clash:.2f} mm³（應為 0：只穿過背板的開孔）")
ok &= servo_clash < 0.5

# 重心（全關、全開取平均）
k = 1.24e-3 * 0.45   # g/mm³
items = [(box_, None), (cover_, None), (pin0, None), (jaw0, None), (Rot(0, 180, 0) * jaw0, None),
         (servo, 55.0), (horn, 8.0)]
m_tot, my, mx = 0.0, 0.0, 0.0
for shape, mass in items:
    m = mass if mass is not None else shape.volume * k
    c = shape.center()
    m_tot += m
    my += m * c.Y
    mx += m * c.X
cg_x, cg_y = mx / m_tot, my / m_tot
print(f"估計總重 {m_tot:.0f} g，重心 X={cg_x:.1f} Y={cg_y:.1f} mm；手腕舵盤中心 Y={pg.FOOT_Y:.1f} mm，"
      f"偏差 {abs(cg_y - pg.FOOT_Y):.1f} mm")
sys.exit(0 if ok else 1)
