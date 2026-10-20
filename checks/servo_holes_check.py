"""確認伺服固定耳的 4 個 M3 孔真的打在機身材料上（孔中心是空的、孔旁邊是實心）。"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import parallel_gripper as pg
from build123d import Face, Plane, Vector, Vertex
from cadkit import fasteners as fx

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

# 手腕 MATRIX 舵盤的 4 個 M4 孔：底板穿孔 + 側面窗口（朝外打開）+ 上方細孔通到頂面
ok2 = True
web_top = pg.BODY_BOTTOM + pg.WRIST_WEB
for hx, hy in pg.WRIST_HOLES:
    x, y = hx, pg.WRIST_Y + hy
    sx = 1 if hx > 0 else -1
    hole_empty = not body.is_inside(Vector(x, y, pg.BODY_BOTTOM + pg.WRIST_WEB / 2))
    web_solid = body.is_inside(Vector(x + 3.2, y, pg.BODY_BOTTOM + pg.WRIST_WEB / 2))
    window_open = all(not body.is_inside(Vector(x + sx * d, y, web_top + 7)) for d in range(0, 12, 2))
    driver_open = all(not body.is_inside(Vector(x, y, z))
                      for z in [web_top + pg.WRIST_WINDOW_H + 1 + k * 4 for k in range(int((pg.BODY_TOP - web_top - pg.WRIST_WINDOW_H) / 4))])
    good = hole_empty and web_solid and window_open and driver_open
    ok2 &= good
    print(f"手腕孔 (x={x:+.1f}, y={y:+.1f})：穿孔空={hole_empty} 底板實心={web_solid} 側窗通到外面={window_open} 上方細孔通到頂={driver_open} → {'OK' if good else '有問題'}")
# 小齒輪的 4 個孔（同 MATRIX 小舵盤孔位）：從背面（Y=0）往前；孔邊到齒輪外形要 ≥ 1.2 mm（M4 時只剩 1.14 會裂）
pin = pg.pinion()
gear_blank = pin
ok3 = True
for hx, hz in pg.PINION_HOLES:
    center_empty = not pin.is_inside(Vector(hx, pg.GEAR_W / 2, hz))
    wall_solid = pin.is_inside(Vector(hx + 2.4, pg.GEAR_W / 2, hz)) if abs(hx) < 1 else pin.is_inside(Vector(hx, pg.GEAR_W / 2, hz + 2.4))
    outline = (gear_blank & Face.make_rect(100, 100, Plane.XZ.offset(-pg.GEAR_W / 2))).faces()[0].outer_wire()
    rim = outline.distance_to(Vertex(hx, pg.GEAR_W / 2, hz)) - fx.METRIC[pg.PINION_SCREW]["tap"] / 2
    good = center_empty and wall_solid and rim >= 1.2
    ok3 &= good
    print(f"小齒輪孔 ({hx:+.0f}, {hz:+.0f})：孔中心空={center_empty} 孔壁實心={wall_solid} 孔邊到齒形 {rim:.2f} mm → {'OK' if good else '有問題'}")
sys.exit(0 if (ok and ok2 and ok3) else 1)
