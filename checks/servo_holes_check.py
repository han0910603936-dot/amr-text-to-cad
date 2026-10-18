"""確認伺服固定耳的 4 個 M3 孔真的打在機身材料上（孔中心是空的、孔旁邊是實心）。"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import parallel_gripper as pg
from build123d import Vector

body = pg.body()
ok = True
for dz in (-pg.SERVO_HOLE_DL / 2, pg.SERVO_HOLE_DL / 2):
    for dx in (-pg.SERVO_HOLE_DW / 2, pg.SERVO_HOLE_DW / 2):
        x, z = dx, pg.SERVO_CZ + dz
        y = pg.EAR_FRONT_Y + 4.0                     # 孔裡面 4 mm
        center_empty = not body.is_inside(Vector(x, y, z))
        wall_solid = body.is_inside(Vector(x + 2.5, y, z))   # 孔壁外 1.2 mm
        seat_solid = body.is_inside(Vector(x + 2.5, pg.EAR_FRONT_Y + 0.5, z))
        good = center_empty and wall_solid and seat_solid
        ok &= good
        print(f"孔 (x={x:+.1f}, z={z:+.1f})：孔中心空={center_empty} 孔壁實心={wall_solid} 座面實心={seat_solid} → {'OK' if good else '有問題'}")

# 手腕 MATRIX 舵盤的 4 個 M4 孔：從機身底面往上
ok2 = True
for hx, hy in pg.WRIST_HOLES:
    x, y = hx, pg.WRIST_Y + hy
    center_empty = not body.is_inside(Vector(x, y, pg.BODY_BOTTOM + 5.0))
    wall_solid = body.is_inside(Vector(x + 3.0, y, pg.BODY_BOTTOM + 5.0))
    good = center_empty and wall_solid
    ok2 &= good
    print(f"手腕孔 (x={x:+.1f}, y={y:+.1f})：孔中心空={center_empty} 孔壁實心={wall_solid} → {'OK' if good else '有問題'}")
# 小齒輪的 4 個 M4 孔（同 MATRIX 小舵盤孔位）：從背面（Y=0）往前
pin = pg.pinion()
ok3 = True
for hx, hz in pg.PINION_HOLES:
    center_empty = not pin.is_inside(Vector(hx, pg.GEAR_W / 2, hz))
    wall_solid = pin.is_inside(Vector(hx + 2.4, pg.GEAR_W / 2, hz)) if abs(hx) < 1 else pin.is_inside(Vector(hx, pg.GEAR_W / 2, hz + 2.4))
    good = center_empty and wall_solid
    ok3 &= good
    print(f"小齒輪孔 ({hx:+.0f}, {hz:+.0f})：孔中心空={center_empty} 孔壁實心={wall_solid} → {'OK' if good else '有問題'}")
sys.exit(0 if (ok and ok2 and ok3) else 1)
