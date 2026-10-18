"""L 型支架（測試零件）：底板 2 個 M3 穿孔鎖在底盤上，立板 2 個 M3 穿孔裝感測器。

座標：底板底面在 Z=0，立板在 -Y 側，內角有圓角加強。
假設：板厚 4 mm（≥ 1.6 mm 無支撐牆標準的 2 倍以上），孔距 20 mm。
"""

from build123d import Align, Axis, Box, Pos, Rot, fillet

from cadkit import fasteners as fx
from cadkit import save

# 參數（mm）
WIDTH = 40.0          # 沿 X
BASE_DEPTH = 30.0     # 底板沿 Y
HEIGHT = 30.0         # 立板沿 Z
THICKNESS = 4.0
INNER_FILLET = 3.0
HOLE_SIZE = "M3"
HOLE_SPACING = 20.0   # 兩孔中心距


def l_bracket():
    base = Box(WIDTH, BASE_DEPTH, THICKNESS, align=(Align.CENTER, Align.MIN, Align.MIN))
    wall = Box(WIDTH, THICKNESS, HEIGHT, align=(Align.CENTER, Align.MIN, Align.MIN))
    part = base + wall

    # 內角圓角：底板上表面和立板內側面的交線
    inner = part.edges().filter_by(Axis.X).filter_by_position(Axis.Z, THICKNESS, THICKNESS) \
        .filter_by_position(Axis.Y, THICKNESS, THICKNESS)
    part = fillet(inner, INNER_FILLET)

    # 底板孔：從上表面往下穿
    base_hole_y = THICKNESS + (BASE_DEPTH - THICKNESS) / 2
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        part -= Pos(x, base_hole_y, THICKNESS) * fx.clearance_hole(HOLE_SIZE, THICKNESS)

    # 立板孔：從 +Y 面往 -Y 穿（把往 -Z 的孔轉成往 -Y）
    wall_hole_z = THICKNESS + (HEIGHT - THICKNESS) / 2
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        part -= Pos(x, THICKNESS, wall_hole_z) * Rot(-90, 0, 0) * fx.clearance_hole(HOLE_SIZE, THICKNESS)

    part.label = "l_bracket"
    return part


if __name__ == "__main__":
    save(l_bracket(), "l_bracket")
