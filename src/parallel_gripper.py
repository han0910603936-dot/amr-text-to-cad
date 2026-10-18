"""平行夾爪（照使用者照片的結構：小機身、齒條穿出兩側）。

設計優先順序：機構最簡單、好裝 → 檢查會不會影響功能 → 外觀最後。

零件（3 種）：
  body    機身：背面是伺服馬達的口袋（伺服直立、在中心線上）；上下兩條齒條座（長 100 mm），
          齒條座前面不包起來，只在外側留一條擋邊防止齒條往前脫出；小齒輪在正前方露出來，直接裝；
          底面直接鎖手腕 MATRIX 舵盤（機身底面 47 mm 深，跟舵盤前後切齊）。
  pinion  小齒輪：44 齒、模數 1.5，用 4 顆 M4 鎖在夾爪伺服馬達的 MATRIX 舵盤上
  jaw     齒條夾爪：齒條 12×12，夾臂從齒條正面（擋邊下方的開口）接出來，夾片往前伸。
          上下兩支是同一個零件（下面那支繞 Y 軸轉 180°），印 2 個。

座標：X = 開合方向，Y = 往前（夾片方向），Z = 上下。原點 = 小齒輪中心、小齒輪背面。對 X=0 左右對稱。

組裝：小齒輪鎖上舵盤 → 伺服從背面口袋放進去、鎖固定耳 → 舵盤＋小齒輪從前面裝上伺服軸、鎖中心螺絲
     → 伺服轉到全關位置 → 兩支齒條夾爪從左右滑進齒條座 → 舵盤鎖到手腕。

舵盤：MATRIX RC Servo Horn Wheel（13-0002），尺寸取自官方 Studio 2.0 零件檔 MX13-0002.dat：
Ø47、厚 3.2、12 個 Ø4.2 孔在 8 mm 格線上。MATRIX 結構螺絲是 M4。

夾爪伺服馬達：MG996R（Tower Pro 規格書）：本體 40.3 × 20、固定耳全長 53.6、固定耳下緣離底 26.6、
   殼頂 36.6、裝舵盤後最高 47.6、可轉約 120°（左右各 60°）。
   設計以 120° 計算開口；伺服若能轉 180°，程式要限制在 120° 以內（超過齒條會跑出去）。
"""

import math

from bd_warehouse.gear import RackGear, SpurGear
from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from cadkit import fasteners as fx
from cadkit import save

# ---------- 齒輪（不變）----------
MODULE = 1.5
PINION_TEETH = 44                     # 節圓 66；伺服轉 120° 每邊走 69 mm
PRESSURE_ANGLE = 20.0
GEAR_W = 8.0                          # 小齒輪齒寬
PITCH_R = MODULE * PINION_TEETH / 2   # 33
PITCH = math.pi * MODULE
SERVO_RANGE_DEG = 120.0               # MG996R 規格書：約 120°
STROKE = math.radians(SERVO_RANGE_DEG) * PITCH_R   # 每邊 69.1，全開 138.2

# ---------- 齒條（加大）----------
FIT = 0.3
RACK_W = 12.0                         # 齒條寬（沿 Y），比小齒輪寬，多出來的增加穩定
RACK_BASE = 12.0                      # 齒根到齒條背面的厚度
RACK_Z0 = PITCH_R - MODULE            # 齒尖 31.5
RACK_ROOT = PITCH_R + 1.25 * MODULE   # 齒根 34.9
RACK_Z1 = RACK_ROOT + RACK_BASE       # 齒條背面 46.9
RACK_TEETH = 21                       # 奇數；長 99
RACK_LEN = RACK_TEETH * PITCH
_PHASE = 0.5 if PINION_TEETH % 4 == 0 else 0.0
# 全關時齒條外端的 X：齒條和小齒輪正上方的齒/齒間要對上；夾臂接在齒條正面，不受機身限制
JAW_END = (RACK_TEETH / 2 + _PHASE - 8) * PITCH     # 14.1

# ---------- 機身 ----------
WALL = 3.0
SEAT_HALF = 50.0                      # 齒條座長 100 mm
LIP_H = 4.0                           # 外側擋邊蓋住齒條正面的高度
LIP_T = 3.0
LIP_Z0 = RACK_Z1 + FIT - LIP_H        # 擋邊下緣
FRONT_Y = RACK_W + FIT + LIP_T        # 擋邊前表面 15.3
BODY_DEPTH = 47.0                     # = 舵盤直徑，底面前後跟舵盤切齊
BACK_Y = FRONT_Y - BODY_DEPTH         # 機身背面 -31.7
BODY_TOP = RACK_Z1 + FIT + WALL       # 50.2
BODY_BOTTOM = -(RACK_Z1 + FIT) - 12.0  # 底下 12 mm 實心：手腕 M4 孔深 9.5，離齒條座 2.5 mm
CORE_HALF_X = PITCH_R + MODULE + 1.0 + WALL   # 中間（伺服、舵盤）需要的寬度；兩側挖空減重

# ---------- 夾臂、夾片 ----------
TAB_Z = (RACK_ROOT + 0.3, LIP_Z0 - 0.4)   # 從齒條正面穿出擋邊下方開口的高度
ARM_GAP = 0.5                         # 夾臂和擋邊前表面的間隙
ARM_T = 5.0                           # 夾臂厚（沿 Y）
FINGER_T = 5.0                        # 夾片厚（沿 X）
FINGER_LEN = 60.0                     # 夾片往前伸出長度
FINGER_Z = (-35.0, RACK_Z1)           # 夾片上下範圍

# ---------- 夾爪伺服馬達（MG996R，直立）----------
SERVO_L, SERVO_W = 40.7, 20.0           # 規格書 40.3 × 20（取大一點）
SERVO_SHAFT_FROM_END = 10.2           # 軸靠下端，本體往上
SERVO_EAR_L, SERVO_EAR_T = 54.5, 2.5
SERVO_HOLE_DL, SERVO_HOLE_DW = 49.5, 10.0
SERVO_BELOW_EAR = 28.0                # 固定耳以下的本體深度（估重心用）
FLANGE_TO_HORN_TOP = 47.6 - 26.6 - 2.5   # 固定耳上表面到舵盤頂面 18.5（規格書）
SERVO_CZ = SERVO_L / 2 - SERVO_SHAFT_FROM_END
EAR_FRONT_Y = -FLANGE_TO_HORN_TOP     # 固定耳前表面（貼在口袋底的座面）

# ---------- MATRIX 舵盤 ----------
HORN_D, HORN_T = 47.0, 3.2
HORN_NUT_SPACE = 4.0                  # 舵盤背面 M4 螺帽的空間
BUTTON_HEAD_D, BUTTON_HEAD_H = 7.6, 2.2
PINION_HOLES = [(16.0, 0.0), (-16.0, 0.0), (0.0, 16.0), (0.0, -16.0)]
WRIST_HOLES = [(16.0, 0.0), (-16.0, 0.0), (0.0, 16.0), (0.0, -16.0)]
WRIST_Y = (FRONT_Y + BACK_Y) / 2      # 手腕舵盤中心 = 機身底面中心


def box(x0, x1, y0, y1, z0, z1):
    """用座標範圍建立方塊（每個方向的兩個值順序不拘）。"""
    (x0, x1), (y0, y1), (z0, z1) = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def pinion():
    gear = SpurGear(MODULE, PINION_TEETH, PRESSURE_ANGLE, root_fillet=0.25, thickness=GEAR_W,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    gear -= Cylinder(7.0 / 2, GEAR_W, align=(Align.CENTER, Align.CENTER, Align.MIN))   # 舵盤中心螺絲
    for x, y in PINION_HOLES:
        gear -= Pos(x, y, GEAR_W) * fx.clearance_hole("M4", GEAR_W)
        gear -= Pos(x, y, GEAR_W) * Cylinder((BUTTON_HEAD_D + 0.6) / 2, BUTTON_HEAD_H + 0.4,
                                             align=(Align.CENTER, Align.CENTER, Align.MAX))
    return Rot(-90, 0, 0) * gear      # 軸沿 +Y，背面在 Y=0


def jaw():
    """上方那支（全關）。齒條在齒條座裡，夾臂從齒條正面穿出擋邊下方的開口，夾片往前伸。"""
    rack = RackGear(MODULE, RACK_TEETH, PRESSURE_ANGLE, thickness=RACK_W, base_height=RACK_BASE)
    rack = Pos(JAW_END - RACK_LEN / 2, RACK_W / 2, PITCH_R) * Rot(180, 0, 0) * rack   # 齒朝下
    arm_y0 = FRONT_Y + ARM_GAP
    tab = box(0, JAW_END, RACK_W - 0.5, arm_y0 + ARM_T, *TAB_Z)          # 齒條正面 → 夾臂
    finger = box(0, FINGER_T, arm_y0, arm_y0 + FINGER_LEN, *FINGER_Z)
    part = rack + tab + finger
    part.label = "jaw"
    return part


def body():
    # 後半部：伺服、舵盤所在的實心塊（寬度 = 齒條座長）
    part = box(-SEAT_HALF, SEAT_HALF, BACK_Y, 0, BODY_BOTTOM, BODY_TOP)
    # 前半部只有上下兩條齒條座：外側牆 + 擋邊（齒條正面其餘部分露出來）
    for sign in (1, -1):
        outer = box(-SEAT_HALF, SEAT_HALF, -0.01, FRONT_Y, RACK_Z1 + FIT, BODY_TOP)
        lip = box(-SEAT_HALF, SEAT_HALF, RACK_W + FIT, FRONT_Y, LIP_Z0, RACK_Z1 + FIT + 0.01)
        piece = outer + lip
        if sign == -1:
            piece = Rot(0, 180, 0) * piece
            piece += box(-SEAT_HALF, SEAT_HALF, -0.01, FRONT_Y, BODY_BOTTOM, -(RACK_Z1 + FIT))  # 底部加厚
        part += piece
    # 兩側挖空減重（中間留給伺服和舵盤；上下留牆給齒條座背面）
    for sx in (1, -1):
        part -= box(sx * (CORE_HALF_X + 0.5), sx * (SEAT_HALF - WALL), BACK_Y - 1, -WALL,
                    -(RACK_Z0 - FIT - WALL), RACK_Z0 - FIT - WALL)
    # 舵盤 + 背面螺帽轉動的空間（小齒輪本身在機身前面）
    part -= Pos(0, 0.01, 0) * Rot(90, 0, 0) * Cylinder(
        HORN_D / 2 + 1.0, HORN_T + HORN_NUT_SPACE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 伺服馬達口袋（從背面放進去）：本體 + 固定耳，固定耳前表面靠在座面上
    ear = box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, BACK_Y - 1, EAR_FRONT_Y,
              SERVO_CZ - SERVO_EAR_L / 2 - FIT, SERVO_CZ + SERVO_EAR_L / 2 + FIT)
    top_case = box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, BACK_Y - 1, 0,
                   SERVO_CZ - SERVO_L / 2 - FIT, SERVO_CZ + SERVO_L / 2 + FIT)
    part -= ear + top_case
    for dz in (-SERVO_HOLE_DL / 2, SERVO_HOLE_DL / 2):
        for dx in (-SERVO_HOLE_DW / 2, SERVO_HOLE_DW / 2):
            # 孔從固定耳座面往前（+Y）鑽進機身，螺絲從背面口袋鎖進來
            part -= Pos(dx, EAR_FRONT_Y, SERVO_CZ + dz) * Rot(90, 0, 0) * fx.tap_hole("M3", 8)
    # 手腕 MATRIX 舵盤：直接鎖在機身底面
    for hx, hy in WRIST_HOLES:
        part -= Pos(hx, WRIST_Y + hy, BODY_BOTTOM) * Rot(180, 0, 0) * fx.tap_hole("M4", 9.5)
    part.label = "body"
    return part


def assembly(opening: float = 0.0):
    shift = opening / 2
    angle = math.degrees(shift / PITCH_R)
    parts = [body(), Rot(0, angle, 0) * pinion(),       # 上方齒條往 +X，齒輪頂端也往 +X
             Pos(shift, 0, 0) * jaw(), Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw()]
    for p, name in zip(parts, ["body", "pinion", "jaw_top", "jaw_bottom"]):
        p.label = name
    return Compound(children=parts, label="parallel_gripper")


if __name__ == "__main__":
    print(f"每邊行程 {STROKE:.1f} mm，全開 {2 * STROKE:.1f} mm；齒條長 {RACK_LEN:.1f}，全關時外端 X={JAW_END:.1f}")
    save(body(), "parallel_gripper_body")
    save(pinion(), "parallel_gripper_pinion")
    save(jaw(), "parallel_gripper_jaw")
    save(assembly(0), "parallel_gripper_assembly_closed", formats=("step",))
    save(assembly(2 * STROKE), "parallel_gripper_assembly_open", formats=("step",))
