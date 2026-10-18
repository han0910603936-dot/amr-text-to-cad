"""平行夾爪：伺服馬達帶動小齒輪，上下兩條齒條反向移動，兩片夾爪平行開合。

結構參考使用者提供的照片（齒輪齒條式平行夾爪）。
零件：
  body   主體：固定夾爪伺服馬達、齒條滑槽，底部延伸板裝在手腕伺服馬達的舵盤上
  cover  前蓋：擋住齒條和齒輪，用 4 顆 M3 鎖在主體上
  pinion 小齒輪：鎖在夾爪伺服馬達的圓形舵盤上
  jaw    齒條夾爪：上下兩支是同一個零件（下面那支轉 180° 安裝），要印 2 個

座標：X = 開合方向，Y = 夾爪伸出方向（往前），Z = 上下。
原點 = 小齒輪中心、小齒輪背面（貼舵盤那一面）。

舵盤：MATRIX RC Servo Horn Wheel（13-0002），尺寸取自官方 Studio 2.0 零件檔 MX13-0002.dat
（1 LDU = 0.4 mm）：直徑 47、厚 3.2、12 個 Ø4.2 孔排在 8 mm 格線上
（(±16,0)、(0,±16)、(±16,±8)、(±8,±16)），頂面平整。MATRIX 結構螺絲是 M4。
手腕和小齒輪都用這款舵盤，各用 (±16,0)、(0,±16) 四個孔、M4×12 螺絲。

⚠ 假設（要量實物確認，見「夾爪伺服馬達」那段參數）：
  - 夾爪伺服馬達是標準尺寸（MG996R 一類，40.7 × 19.7 mm），可轉 180°
  - 裝上舵盤後，固定耳上表面到舵盤上表面 15.5 mm
"""

import math

from bd_warehouse.gear import RackGear, SpurGear
from build123d import Align, Box, Compound, Cylinder, Plane, Pos, Rot, mirror

from cadkit import fasteners as fx
from cadkit import save

# ---------- 齒輪（模數 1.5、壓力角 20°）----------
MODULE = 1.5
PINION_TEETH = 32                     # 節圓直徑 48 mm（齒根夠大，M4 沉頭孔外還有約 2 mm）
PRESSURE_ANGLE = 20.0
GEAR_W = 8.0                          # 齒寬（沿 Y）
PITCH_R = MODULE * PINION_TEETH / 2   # 22.5
PITCH = math.pi * MODULE              # 齒距 4.712

# ---------- 行程 ----------
SERVO_RANGE_DEG = 180.0               # 伺服馬達可轉角度
STROKE = math.radians(SERVO_RANGE_DEG) * PITCH_R   # 每邊行程 75.4 mm，全開 151 mm

# ---------- 齒條夾爪 ----------
RACK_TEETH = 23                       # 奇數：齒條中心剛好是一顆齒，方便對相位
RACK_LEN = RACK_TEETH * PITCH         # 108.4 mm
RACK_BASE = 10.0                      # 齒根到齒條背面的厚度
RACK_TOP = PITCH_R + 1.25 * MODULE + RACK_BASE      # 齒條背面高度 34.4
GROOVE_Z = PITCH_R + 1.25 * MODULE + RACK_BASE / 2  # 導槽中心高度 29.4
GROOVE_H, GROOVE_D = 3.0, 1.5         # 齒條兩側導槽
FIT = 0.3                             # 滑動間隙
# 齒條 +X 端的位置（夾爪全關時）：讓齒條和小齒輪正上方的齒/齒間對上。
# 小齒輪齒數是 4 的倍數時正上方是齒，齒條在 X=0 要是齒間（多偏半個齒距）
_PHASE = 0.5 if PINION_TEETH % 4 == 0 else 0.0
JAW_END_CLOSED = (RACK_TEETH / 2 - 8 + _PHASE) * PITCH      # 18.85

BEAM_Z0, BEAM_H = 40.0, 6.0           # 橫樑底面高度、厚度
FINGER_T = 5.0                        # 夾片厚度
FINGER_LEN = 55.0                     # 夾片往前伸出的長度
FINGER_Z_REACH = 38.0                 # 夾片往另一側伸到的高度

# ---------- 主體 ----------
BODY_X = (-40.0, 44.0)
BODY_Z = 38.0                         # 主體上下範圍 ±38
PLATE_T = 4.0                         # 伺服馬達安裝板厚
BLOCK_Z0 = 19.5                       # 齒條滑槽背塊的內側高度
COVER_T = 3.0
COVER_Y0 = GEAR_W + FIT               # 前蓋內面
PILLAR = 7.0                          # 前蓋支柱邊長
PILLAR_POS = [(-34.0, 14.5), (-34.0, -14.5), (38.0, 14.5), (38.0, -14.5)]

# ---------- 夾爪伺服馬達（假設：標準尺寸）----------
SERVO_L, SERVO_W = 40.7, 19.7         # 本體長（沿 X）、寬（沿 Z）
SERVO_SHAFT_FROM_END = 10.2           # 輸出軸中心到本體近端的距離
SERVO_HOLE_DX, SERVO_HOLE_DZ = 49.5, 10.0   # 固定耳孔距
SERVO_FLANGE_T = 2.5                  # 固定耳厚度
FLANGE_TO_HORN_TOP = 15.5             # 固定耳上表面到舵盤上表面

# ---------- MATRIX 舵盤（13-0002）----------
HORN_D, HORN_T = 47.0, 3.2
HORN_HOLES = [(16.0, 0.0), (-16.0, 0.0), (0.0, 16.0), (0.0, -16.0)]   # 用到的 4 個孔
HORN_CENTER_HEAD_D = 5.8              # 舵盤中心螺絲頭（沉在舵盤裡）
BUTTON_HEAD_D, BUTTON_HEAD_H = 7.6, 2.2   # M4 半圓頭螺絲頭（ISO 7380）

# ---------- 手腕連接 ----------
SHELF_T = 8.0                         # 底部延伸板厚：M4×12 = 舵盤 3.2 + 延伸板 8，螺帽卡在上方
WRIST_EDGE = 5.0                      # 孔到延伸板邊緣至少留的距離

# 由上面推算
PLATE_FRONT_Y = -FLANGE_TO_HORN_TOP - SERVO_FLANGE_T                # 安裝板前表面
PLATE_BACK_Y = PLATE_FRONT_Y - PLATE_T
SERVO_CX = SERVO_L / 2 - SERVO_SHAFT_FROM_END                       # 伺服本體中心 X
# 手腕舵盤中心：4 個孔都落在安裝板後方的延伸板上
WRIST_CENTER = (SERVO_CX, PLATE_BACK_Y - WRIST_EDGE - 16.0)
SHELF_Y0 = WRIST_CENTER[1] - HORN_D / 2 - 3.0                       # 底部延伸板後端


def box(x0, x1, y0, y1, z0, z1):
    """用座標範圍建立方塊。"""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def pinion():
    gear = SpurGear(MODULE, PINION_TEETH, PRESSURE_ANGLE, root_fillet=0.3, thickness=GEAR_W,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 中心孔：讓舵盤中心螺絲頭和起子穿過（先把小齒輪鎖上舵盤，再整組裝上伺服馬達）
    gear -= Cylinder((HORN_CENTER_HEAD_D + 1.2) / 2, GEAR_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 4 顆 M4 半圓頭螺絲，頭沉進齒輪正面，螺帽在舵盤背面
    for x, y in HORN_HOLES:
        gear -= Pos(x, y, GEAR_W) * fx.clearance_hole("M4", GEAR_W)
        gear -= Pos(x, y, GEAR_W) * Cylinder((BUTTON_HEAD_D + 0.6) / 2, BUTTON_HEAD_H + 0.4,
                                             align=(Align.CENTER, Align.CENTER, Align.MAX))
    # 齒輪軸沿 Z 建立，轉成沿 +Y；背面（貼舵盤）在 Y=0
    return Rot(-90, 0, 0) * gear


def jaw():
    """上方那支齒條夾爪（夾爪全關的位置）。下方那支是同一個零件繞 Y 軸轉 180°。"""
    rack = RackGear(MODULE, RACK_TEETH, PRESSURE_ANGLE, thickness=GEAR_W, base_height=RACK_BASE)
    # 齒朝下、節線在 Z=PITCH_R、Y 範圍 0~GEAR_W、+X 端在 JAW_END_CLOSED
    rack = Pos(JAW_END_CLOSED - RACK_LEN / 2, GEAR_W / 2, PITCH_R) * Rot(180, 0, 0) * rack
    x1 = JAW_END_CLOSED
    x0 = x1 - RACK_LEN
    for y0, y1 in ((0, GROOVE_D), (GEAR_W - GROOVE_D, GEAR_W)):        # 前後兩側導槽
        rack -= box(x0 - 1, x1 + 1, y0 - 0.01 if y0 == 0 else y0, y1 + 0.01 if y1 == GEAR_W else y1,
                    GROOVE_Z - GROOVE_H / 2, GROOVE_Z + GROOVE_H / 2)

    y_finger0 = COVER_Y0 + COVER_T + 1.5          # 夾片從前蓋前方 1.5 mm 開始
    y_end = y_finger0 + FINGER_LEN
    riser = box(0, x1, 0, GEAR_W, RACK_TOP - 0.5, BEAM_Z0 + 0.5)   # 從夾片到齒條端，整段填滿
    beam = box(0, x1, 0, y_end, BEAM_Z0, BEAM_Z0 + BEAM_H)
    finger = box(0, FINGER_T, y_finger0, y_end, -FINGER_Z_REACH, BEAM_Z0 + BEAM_H)
    part = rack + riser + beam + finger
    part.label = "jaw"
    return part


def body():
    x0, x1 = BODY_X
    plate = box(x0, x1, PLATE_BACK_Y, PLATE_FRONT_Y, -BODY_Z, BODY_Z)
    # 伺服馬達本體穿過安裝板
    plate -= box(SERVO_CX - SERVO_L / 2 - FIT, SERVO_CX + SERVO_L / 2 + FIT,
                 PLATE_BACK_Y - 1, PLATE_FRONT_Y + 1, -SERVO_W / 2 - FIT, SERVO_W / 2 + FIT)
    part = plate
    for sign in (1, -1):
        block = box(x0, x1, PLATE_FRONT_Y - 0.01, -FIT, BLOCK_Z0, BODY_Z)
        rail = box(x0, x1, -FIT - 0.01, GROOVE_D - FIT,
                   GROOVE_Z - GROOVE_H / 2 + FIT, GROOVE_Z + GROOVE_H / 2 - FIT)
        piece = block + rail
        part += piece if sign == 1 else mirror(piece, Plane.XY)   # 上下鏡射（主體左右不對稱，不能用旋轉）
    # 讓夾爪伺服馬達上的舵盤（Ø47，連同背面的螺帽）可以轉動
    part -= Pos(0, 0.01, 0) * Rot(90, 0, 0) * Cylinder(HORN_D / 2 + 1.0, HORN_T + 4.0,
                                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    for px, pz in PILLAR_POS:
        pillar = box(px - PILLAR / 2, px + PILLAR / 2, PLATE_FRONT_Y - 0.01, COVER_Y0,
                     pz - PILLAR / 2, pz + PILLAR / 2)
        part += pillar
        part -= Pos(px, COVER_Y0, pz) * Rot(90, 0, 0) * fx.tap_hole("M3", 10)
    # 伺服馬達固定耳螺絲孔（自攻）
    for dx in (-SERVO_HOLE_DX / 2, SERVO_HOLE_DX / 2):
        for dz in (-SERVO_HOLE_DZ / 2, SERVO_HOLE_DZ / 2):
            part -= Pos(SERVO_CX + dx, PLATE_FRONT_Y, dz) * Rot(90, 0, 0) * fx.tap_hole("M3", PLATE_T)
    # 底部延伸板：從安裝板往後延伸，底面鎖在手腕的 MATRIX 舵盤上
    wx, wy = WRIST_CENTER
    shelf = box(wx - HORN_D / 2 - 3, wx + HORN_D / 2 + 3,
                SHELF_Y0, PLATE_BACK_Y + 0.01, -BODY_Z, -BODY_Z + SHELF_T)
    part += shelf
    for hx, hy in HORN_HOLES:
        top = Pos(wx + hx, wy + hy, -BODY_Z + SHELF_T)
        part -= top * fx.clearance_hole("M4", SHELF_T)
        part -= top * fx.nut_trap("M4")
    part.label = "body"
    return part


def cover():
    x0, x1 = BODY_X
    part = box(x0, x1, COVER_Y0, COVER_Y0 + COVER_T, -BODY_Z, BODY_Z)
    for sign in (1, -1):
        rail = box(x0, x1, GEAR_W - GROOVE_D + FIT, COVER_Y0 + 0.01,
                   GROOVE_Z - GROOVE_H / 2 + FIT, GROOVE_Z + GROOVE_H / 2 - FIT)
        part += rail if sign == 1 else mirror(rail, Plane.XY)
    part -= Pos(0, COVER_Y0 + COVER_T, 0) * Rot(-90, 0, 0) * Cylinder(5.0, COVER_T * 3)   # 鎖舵盤螺絲用
    for px, pz in PILLAR_POS:
        part -= Pos(px, COVER_Y0 + COVER_T, pz) * Rot(-90, 0, 0) * fx.clearance_hole("M3", COVER_T)
    part.label = "cover"
    return part


def assembly(opening: float = 0.0):
    """組合圖。opening = 兩夾片之間的距離（mm），0 = 全關。"""
    shift = opening / 2
    angle = math.degrees(shift / PITCH_R)
    top = Pos(shift, 0, 0) * jaw()
    bottom = Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw()
    gear = Rot(0, angle, 0) * pinion()   # 上方齒條往 +X，齒輪頂端也要往 +X
    parts = [body(), cover(), gear, top, bottom]
    for p, name in zip(parts, ["body", "cover", "pinion", "jaw_top", "jaw_bottom"]):
        p.label = name
    return Compound(children=parts, label="parallel_gripper")


if __name__ == "__main__":
    print(f"每邊行程 {STROKE:.1f} mm，全開 {2 * STROKE:.1f} mm（伺服轉 {SERVO_RANGE_DEG:.0f}°）")
    save(body(), "parallel_gripper_body")
    save(cover(), "parallel_gripper_cover")
    save(pinion(), "parallel_gripper_pinion")
    save(jaw(), "parallel_gripper_jaw")
    save(assembly(0), "parallel_gripper_assembly_closed", formats=("step",))
    save(assembly(2 * STROKE), "parallel_gripper_assembly_open", formats=("step",))
