"""平行夾爪檢查：各開合位置的零件干涉、夾片間距、齒條在機身裡的導引長度、整體重心。

    python -X utf8 checks/parallel_gripper_check.py

伺服馬達和舵盤用簡化方塊/圓盤代替，一起檢查會不會撞到。
重心估算：列印件 PLA 1.24 g/cm³ × 有效填充 45%，伺服馬達 55 g，MATRIX 小舵盤 4 g。
"""
import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import parallel_gripper as pg  # noqa: E402
from build123d import Align, Cylinder, Pos, Rot  # noqa: E402

body_, pin0, jaw0 = pg.body(), pg.pinion(), pg.jaw()

# 伺服假人：固定耳前表面貼座面，上蓋往前到 -11，本體往後
ey = pg.EAR_FRONT_Y
sw = pg.SERVO_W / 2
servo = (pg.box(-sw, sw, ey - pg.SERVO_EAR_T, ey, pg.SERVO_CZ - pg.SERVO_EAR_L / 2, pg.SERVO_CZ + pg.SERVO_EAR_L / 2)
         + pg.box(-sw, sw, ey - pg.SERVO_EAR_T - pg.SERVO_BELOW_EAR, -11.0,
                  pg.SERVO_CZ - pg.SERVO_L / 2, pg.SERVO_CZ + pg.SERVO_L / 2))
horn = Pos(0, -0.01, 0) * Rot(90, 0, 0) * Cylinder(pg.MICRO_HORN_D / 2, pg.MICRO_HORN_T,
                                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
ok = True
for i in range(9):
    opening = i / 8 * pg.OPENING
    shift = opening / 2
    angle = math.degrees(shift / pg.PITCH_R)
    parts = {
        "body": body_, "servo": servo,
        "pinion": Rot(0, angle, 0) * pin0, "horn": Rot(0, angle, 0) * horn,
        "jaw_top": Pos(shift, 0, 0) * jaw0,
        "jaw_bottom": Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw0,
    }
    clashes = []
    for (a, pa), (b, pb) in itertools.combinations(parts.items(), 2):
        if (a, b) == ("pinion", "horn"):
            continue
        v = (pa & pb).volume
        if v > 0.5:
            clashes.append(f"{a}×{b} {v:.1f}mm³")
    fa = [f for f in parts["jaw_top"].faces() if abs(f.normal_at().X + 1) < 1e-6 and f.area > 1000]
    fb = [f for f in parts["jaw_bottom"].faces() if abs(f.normal_at().X - 1) < 1e-6 and f.area > 1000]
    gap = min(f.center().X for f in fa) - max(f.center().X for f in fb)
    inner = pg.JAW_END + shift - pg.RACK_LEN                 # 齒條內端
    outer = pg.JAW_END + shift
    guide = min(outer, pg.SEAT_HALF) - max(inner, -pg.SEAT_HALF)   # 齒條在齒條座裡的長度
    ok &= not clashes
    print(f"開口 {opening:6.1f} | 夾片間距 {gap:6.1f} | 齒條內端 {inner:6.1f}（越過中心 {-inner:5.1f}）"
          f" | 導引長度 {guide:5.1f} | {'OK' if not clashes else '干涉'} {' '.join(clashes)}")

rack_h = pg.RACK_Z1 - pg.RACK_Z0
g_min = min(min(pg.JAW_END + s, pg.SEAT_HALF) - max(pg.JAW_END + s - pg.RACK_LEN, -pg.SEAT_HALF) for s in (0, pg.STROKE))
print(f"齒條斷面 {pg.RACK_W:.1f} × {rack_h:.1f} mm；最短導引長度 {g_min:.1f} / 斷面最大邊 ≈ "
      f"{g_min / max(pg.RACK_W, rack_h):.1f}（建議 ≥ 2）")

k = 1.24e-3 * 0.45
items = [(body_, None), (pin0, None), (jaw0, None), (Rot(0, 180, 0) * jaw0, None), (servo, 55.0), (horn, 4.0)]
m_tot = sum(m if m is not None else s.volume * k for s, m in items)
cg_y = sum((m if m is not None else s.volume * k) * s.center().Y for s, m in items) / m_tot
cg_x = sum((m if m is not None else s.volume * k) * s.center().X for s, m in items) / m_tot
print(f"估計總重 {m_tot:.0f} g，重心 X={cg_x:.1f} Y={cg_y:.1f}；手腕舵盤中心 Y={pg.WRIST_Y:.1f}，偏差 {abs(cg_y - pg.WRIST_Y):.1f} mm")
sys.exit(0 if ok else 1)
