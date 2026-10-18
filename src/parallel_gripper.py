"""平行夾爪（照使用者照片的結構：小機身、齒條穿出兩側、Z 字形夾爪）。

設計優先順序：機構最簡單、好裝 → 檢查會不會影響功能 → 外觀最後。

零件（3 種）：
  body    機身：一塊方塊。兩條齒條從左右貫穿的方形隧道裡滑動；中間是小齒輪的圓孔（前面開窗，
          小齒輪從前面裝）；背面是伺服馬達的口袋（伺服直立、在中心線上）；底面直接鎖手腕 MATRIX 舵盤
          （機身底面 47 mm 深，跟舵盤前後切齊）。
  pinion  小齒輪：32 齒、模數 1.5，用 4 顆 M4 鎖在夾爪伺服馬達的 MATRIX 舵盤上
  jaw     齒條夾爪：齒條在機身裡，外端接一支夾臂，從機身前面繞回中間，夾片往前伸。
          上下兩支是同一個零件（下面那支繞 Y 軸轉 180°），印 2 個。

座標：X = 開合方向，Y = 往前（夾片方向），Z = 上下。原點 = 小齒輪中心、小齒輪背面。對 X=0 左右對稱。

組裝：小齒輪鎖上舵盤 → 伺服從背面口袋放進去、鎖固定耳 → 舵盤＋小齒輪從前窗裝上伺服軸、鎖中心螺絲
     → 伺服轉到全關位置 → 兩支齒條夾爪從左右插進隧道 → 舵盤鎖到手腕。

舵盤：MATRIX RC Servo Horn Wheel（13-0002），尺寸取自官方 Studio 2.0 零件檔 MX13-0002.dat：
Ø47、厚 3.2、12 個 Ø4.2 孔在 8 mm 格線上。MATRIX 結構螺絲是 M4。

⚠ 假設（要量實物確認）：夾爪伺服馬達是標準尺寸（MG996R 一類，40.7 × 19.7 mm）、可轉 180°，
   裝上舵盤後固定耳表面到舵盤上表面 15.5 mm。
"""

import math

from bd_warehouse.gear import RackGear, SpurGear
from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from cadkit import fasteners as fx
from cadkit import save

# ---------- 齒輪 ----------
MODULE = 1.5
PINION_TEETH = 32                     # 節圓 48；伺服轉 180° 每邊走 75 mm
PRESSURE_ANGLE = 20.0
GEAR_W = 8.0
PITCH_R = MODULE * PINION_TEETH / 2   # 24
PITCH = math.pi * MODULE
SERVO_RANGE_DEG = 180.0
STROKE = math.radians(SERVO_RANGE_DEG) * PITCH_R   # 每邊 75.4，全開 150.8

# ---------- 機身 ----------
FIT = 0.3
WALL = 3.0
BODY_HALF_X = 30.0                    # 機身寬 60（小齒輪孔 Ø53 + 牆）
BODY_DEPTH = 47.0                     # = 舵盤直徑，底面前後跟舵盤切齊
FRONT_Y = GEAR_W + FIT + WALL         # 機身前表面 11.3
BACK_Y = FRONT_Y - BODY_DEPTH         # 機身背面 -35.7
RACK_BASE = 8.0
RACK_Z0 = PITCH_R - MODULE            # 齒條齒尖 22.5
RACK_Z1 = PITCH_R + 1.25 * MODULE + RACK_BASE      # 齒條背面 33.9
BODY_TOP = RACK_Z1 + FIT + WALL + 2.0  # 上面多 2 mm：伺服固定耳的螺絲孔
BODY_BOTTOM = -(RACK_Z1 + FIT) - 10.0  # 底下留 10 mm 實心給手腕舵盤的 M4 螺絲

# ---------- 齒條夾爪 ----------
RACK_TEETH = 27                       # 奇數；長 127.2
RACK_LEN = RACK_TEETH * PITCH
_PHASE = 0.5 if PINION_TEETH % 4 == 0 else 0.0
# 全關時齒條外端的 X：連接塊要整個在機身外，且齒條和小齒輪正上方的齒/齒間對上
_k = math.floor(RACK_TEETH / 2 + _PHASE - (BODY_HALF_X + 1.0 + 6.0) / PITCH)
JAW_END = (RACK_TEETH / 2 + _PHASE - _k) * PITCH
END_W = 6.0                           # 齒條外端往前的連接塊寬（沿 X）
ARM_GAP = 1.0                         # 夾臂和機身前表面的間隙
ARM_T = 5.0                           # 夾臂厚（沿 Y）
FINGER_T = 5.0                        # 夾片厚（沿 X）
FINGER_LEN = 60.0                     # 夾片往前伸出長度
FINGER_Z = (-35.0, RACK_Z1)           # 夾片上下範圍

# ---------- 夾爪伺服馬達（假設：標準尺寸，直立）----------
SERVO_L, SERVO_W = 40.7, 19.7
SERVO_SHAFT_FROM_END = 10.2           # 軸靠下端，本體往上
SERVO_EAR_L, SERVO_EAR_T = 54.5, 2.5
SERVO_HOLE_DL, SERVO_HOLE_DW = 49.5, 10.0
SERVO_BELOW_EAR = 28.0                # 固定耳以下的本體深度（估重心用）
FLANGE_TO_HORN_TOP = 15.5
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
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def pinion():
    gear = SpurGear(MODULE, PINION_TEETH, PRESSURE_ANGLE, root_fillet=0.3, thickness=GEAR_W,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    gear -= Cylinder(7.0 / 2, GEAR_W, align=(Align.CENTER, Align.CENTER, Align.MIN))   # 舵盤中心螺絲
    for x, y in PINION_HOLES:
        gear -= Pos(x, y, GEAR_W) * fx.clearance_hole("M4", GEAR_W)
        gear -= Pos(x, y, GEAR_W) * Cylinder((BUTTON_HEAD_D + 0.6) / 2, BUTTON_HEAD_H + 0.4,
                                             align=(Align.CENTER, Align.CENTER, Align.MAX))
    return Rot(-90, 0, 0) * gear      # 軸沿 +Y，背面在 Y=0


def jaw():
    """上方那支（全關）。齒條在隧道裡，外端（+X）在機身外，夾臂從機身前面繞回中間。"""
    rack = RackGear(MODULE, RACK_TEETH, PRESSURE_ANGLE, thickness=GEAR_W, base_height=RACK_BASE)
    rack = Pos(JAW_END - RACK_LEN / 2, GEAR_W / 2, PITCH_R) * Rot(180, 0, 0) * rack   # 齒朝下
    arm_y0 = FRONT_Y + ARM_GAP
    end = box(JAW_END - END_W, JAW_END, 0, arm_y0 + ARM_T, RACK_Z0, RACK_Z1)
    arm = box(0, JAW_END, arm_y0, arm_y0 + ARM_T, RACK_Z0, RACK_Z1)
    finger = box(0, FINGER_T, arm_y0, arm_y0 + FINGER_LEN, *FINGER_Z)
    part = rack + end + arm + finger
    part.label = "jaw"
    return part


def body():
    part = box(-BODY_HALF_X, BODY_HALF_X, BACK_Y, FRONT_Y, BODY_BOTTOM, BODY_TOP)
    # 兩條齒條隧道（貫穿左右）
    z0, z1 = RACK_Z0 - FIT, RACK_Z1 + FIT
    for zz in ((z0, z1), (-z1, -z0)):
        part -= box(-BODY_HALF_X - 1, BODY_HALF_X + 1, -FIT, GEAR_W + FIT, *zz)
    # 小齒輪那一層：方孔直接接通上下兩條隧道（用圓孔會在隧道底留下薄片）
    gear_r = PITCH_R + MODULE + 1.0
    part -= box(-gear_r, gear_r, -FIT, GEAR_W + FIT, -(RACK_Z0 - FIT), RACK_Z0 - FIT)
    # 前窗：小齒輪 + 舵盤從前面裝進去
    part -= Pos(0, FRONT_Y + 1, 0) * Rot(90, 0, 0) * Cylinder(
        gear_r, FRONT_Y + 1 - GEAR_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 舵盤 + 背面螺帽轉動的空間
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
            part -= Pos(dx, EAR_FRONT_Y, SERVO_CZ + dz) * Rot(-90, 0, 0) * fx.tap_hole("M3", 8)
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
