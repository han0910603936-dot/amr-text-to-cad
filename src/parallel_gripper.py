"""平行夾爪（照使用者照片的結構：小機身、齒條穿出兩側）。

設計優先順序：機構最簡單、好裝 → 檢查會不會影響功能 → 外觀最後。

零件（3 種）：
  body    機身：背面是伺服馬達的口袋（伺服直立、在中心線上）；上下兩條齒條座（長 100 mm），
          齒條座前面不包起來，只在外側留一條擋邊防止齒條往前脫出；小齒輪在正前方露出來，直接裝；
          後半部（伺服座）寬度和手腕大舵盤一樣 47 mm，後面是圓弧、跟舵盤切齊；
          底面直接鎖手腕 MATRIX 大舵盤（舵盤前緣在機身前端後面，不會超出）。
  pinion  小齒輪：尺寸同 MATRIX 金屬齒輪 24 齒（模數 1、外徑 26、厚 7、中心孔 Ø8），
          固定孔同 MATRIX 小舵盤（(±8,0)、(0,±8)），M4 從舵盤背面鎖進小齒輪
  jaw     齒條夾爪：模數 1 齒條 12×12，夾臂從齒條正面（擋邊下方的開口）接出來，夾片往前伸。
          上下兩支是同一個零件（下面那支繞 Y 軸轉 180°），印 2 個。

座標：X = 開合方向，Y = 往前（夾片方向），Z = 上下。原點 = 小齒輪中心、小齒輪背面。對 X=0 左右對稱。

組裝：小齒輪用 4 顆 M4 從小舵盤背面鎖上 → 伺服從背面口袋放進去、鎖固定耳
     → 小舵盤＋小齒輪從前面裝上伺服軸、從小齒輪中心孔鎖舵盤螺絲
     → 伺服轉到全關位置 → 兩支齒條夾爪從左右滑進齒條座
     → 手腕：大舵盤先裝在手腕伺服上 → 夾爪放上去 → 4 顆 M4×12 從機身底部兩側的窗口放進去，
       扳手從頂面的 Ø5 細孔伸下來鎖，螺帽在舵盤下面。

MATRIX 零件尺寸都取自官方 Studio 2.0 零件檔（1 LDU = 0.4 mm），結構螺絲是 M4、孔 Ø4.2、8 mm 格線：
  大舵盤 RC Servo Horn Wheel 13-0002：Ø47、厚 3.2
  小舵盤 Micro Servo Horn Wheel 13-0012：Ø31、盤厚 1.6、軸套 Ø8 高 3.4，孔 (±8,0)、(0,±8)、(±8,±8)
  金屬齒輪 Metal Gear 24T 13-0024：外徑 26（模數 1）、厚 7、中心孔 Ø8

夾爪伺服馬達：外形用 MG996R（Tower Pro 規格書：本體 40.3 × 20、固定耳全長 53.6、固定耳下緣離底 26.6），
   使用者的是可轉 360° 的版本，所以開口由齒條行程決定：小齒輪轉約 322° 全開 135 mm。
   ⚠ 小舵盤裝上後的高度是估計值（固定耳上表面到舵盤頂面約 15.4 mm）。
"""

import math

from bd_warehouse.gear import RackGear, SpurGear
from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from cadkit import fasteners as fx
from cadkit import save
from cadkit.printprep import save_print

# ---------- 小齒輪（同 MATRIX 24 齒金屬齒輪）----------
MODULE = 1.0
PINION_TEETH = 24
PRESSURE_ANGLE = 20.0
GEAR_W = 7.0
GEAR_BORE = 8.0
PITCH_R = MODULE * PINION_TEETH / 2   # 12
PITCH = math.pi * MODULE
OPENING = 135.0                       # 全開（≥ 130）
STROKE = OPENING / 2                  # 每邊 67.5
PINION_TURN_DEG = math.degrees(STROKE / PITCH_R)   # 全開要轉 322°

# ---------- 齒條 ----------
FIT = 0.3
RACK_W = 12.0                         # 齒條寬（沿 Y）
RACK_BASE = 12.0                      # 齒根到齒條背面的厚度
RACK_Z0 = PITCH_R - MODULE            # 齒尖 11
RACK_ROOT = PITCH_R + 1.25 * MODULE   # 齒根 13.25
RACK_Z1 = RACK_ROOT + RACK_BASE       # 齒條背面 25.25
RACK_TEETH = 29                       # 奇數；長 91.1
RACK_LEN = RACK_TEETH * PITCH
_PHASE = 0.5 if PINION_TEETH % 4 == 0 else 0.0
# 全關時齒條外端的 X：齒條和小齒輪正上方的齒/齒間要對上（夾臂接在齒條正面，不受機身限制）
_k = math.floor(RACK_TEETH / 2 + _PHASE - 8.0 / PITCH)
JAW_END = (RACK_TEETH / 2 + _PHASE - _k) * PITCH     # 9.4

# ---------- 機身 ----------
WALL = 3.0
SEAT_HALF = 50.0                      # 齒條座長 100 mm
LIP_H = 6.0                           # 外側擋邊蓋住齒條正面的高度
LIP_T = 3.0
LIP_Z0 = RACK_Z1 + FIT - LIP_H        # 擋邊下緣
FRONT_Y = RACK_W + FIT + LIP_T        # 擋邊前表面 15.3

# ---------- 夾爪伺服馬達（外形 MG996R，直立）----------
SERVO_L, SERVO_W = 40.7, 20.0
SERVO_SHAFT_FROM_END = 10.2           # 軸靠下端，本體往上
SERVO_EAR_L, SERVO_EAR_T = 54.5, 2.5
SERVO_HOLE_DL, SERVO_HOLE_DW = 49.5, 10.0
SERVO_BELOW_EAR = 28.0                # 固定耳以下的本體深度（估重心用）
FLANGE_TO_HORN_TOP = 15.4             # 估計：花鍵頂 42.9 + 小舵盤 1.6 − 固定耳上表面 29.1
SERVO_CZ = SERVO_L / 2 - SERVO_SHAFT_FROM_END
EAR_FRONT_Y = -FLANGE_TO_HORN_TOP     # 固定耳前表面（貼在口袋底的座面）

# ---------- MATRIX 舵盤 ----------
MICRO_HORN_D, MICRO_HORN_T = 31.0, 1.6
MICRO_HORN_SPACE = 4.0                # 小舵盤背面 M4 螺絲頭的空間
PINION_HOLES = [(8.0, 0.0), (-8.0, 0.0), (0.0, 8.0), (0.0, -8.0)]
HORN_D, HORN_T = 47.0, 3.2            # 手腕大舵盤
# 手腕大舵盤用的孔：都在伺服口袋兩側（|x| = 16），螺絲從底部側面窗口放進去往下鎖，螺帽在舵盤下面
WRIST_HOLES = [(16.0, 0.0), (-16.0, 0.0), (16.0, -8.0), (-16.0, -8.0)]
WRIST_WEB = 4.0                       # 機身底板厚：M4×12 = 底板 4 + 舵盤 3.2 + 螺帽 3.2（全吃）
WRIST_WINDOW_H = 15.0                 # 側面窗口高：M4×12 螺絲（含頭 14.2）能斜著放進去
DRIVER_HOLE_D = 5.0                   # 上方細孔：給六角扳手 / 起子
BUTTON_HEAD_D = 7.6                   # M4 半圓頭螺絲頭直徑（通道要讓它通過）
WRIST_FRONT_GAP = 3.0                 # 大舵盤前緣在機身前端後面多少
WRIST_Y = FRONT_Y - WRIST_FRONT_GAP - HORN_D / 2   # 大舵盤中心 Y
BACK_Y = WRIST_Y - HORN_D / 2         # 機身背面 = 大舵盤後緣（切齊）

BODY_TOP = SERVO_CZ + SERVO_EAR_L / 2 + FIT + 2.5  # 伺服上方固定耳的螺絲孔要包在裡面
BODY_BOTTOM = -(RACK_Z1 + FIT) - 12.0  # 底下 12 mm 實心：手腕 M4 孔深 9.5，離齒條座 2.5 mm

# ---------- 夾臂、夾片 ----------
TAB_Z = (RACK_ROOT + 0.3, LIP_Z0 - 0.4)   # 從齒條正面穿出擋邊下方開口的高度
ARM_GAP = 0.5
ARM_T = 5.0                           # 夾臂厚（沿 Y）
FINGER_T = 5.0                        # 夾片厚（沿 X）
FINGER_LEN = 60.0                     # 夾片往前伸出長度
FINGER_Z = (-22.0, RACK_Z1)           # 夾片上下範圍


def box(x0, x1, y0, y1, z0, z1):
    """用座標範圍建立方塊（每個方向的兩個值順序不拘）。"""
    (x0, x1), (y0, y1), (z0, z1) = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def pinion():
    gear = SpurGear(MODULE, PINION_TEETH, PRESSURE_ANGLE, root_fillet=0.2, thickness=GEAR_W,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    gear -= Cylinder(GEAR_BORE / 2, GEAR_W, align=(Align.CENTER, Align.CENTER, Align.MIN))   # 同 MATRIX 齒輪
    for x, y in PINION_HOLES:
        gear -= Pos(x, y, 0) * Rot(180, 0, 0) * fx.tap_hole("M4", GEAR_W)   # 從背面（舵盤側）往前
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
    # 前面：齒條座（長 100）＝ 背板 + 上下外側牆 + 擋邊，底部加厚到跟後面一樣平
    part = box(-SEAT_HALF, SEAT_HALF, -WALL, 0, BODY_BOTTOM, RACK_Z1 + FIT + WALL)
    top = (box(-SEAT_HALF, SEAT_HALF, -0.01, FRONT_Y, RACK_Z1 + FIT, RACK_Z1 + FIT + WALL)
           + box(-SEAT_HALF, SEAT_HALF, RACK_W + FIT, FRONT_Y, LIP_Z0, RACK_Z1 + FIT + 0.01))
    part += top + Rot(0, 180, 0) * top
    part += box(-SEAT_HALF, SEAT_HALF, -0.01, FRONT_Y, BODY_BOTTOM, -(RACK_Z1 + FIT))
    # 後面：伺服座，外形是「前方後圓」：寬度 = 手腕大舵盤直徑，後面的圓弧跟舵盤切齊
    r = HORN_D / 2
    h = BODY_TOP - BODY_BOTTOM
    housing = Pos(0, WRIST_Y, BODY_BOTTOM) * Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    housing += box(-r, r, WRIST_Y, -WALL + 0.01, BODY_BOTTOM, BODY_TOP)
    # 只留背板後面；伺服出口那段圓弧截平，避免口袋側壁和圓弧交會處變成尖薄片（留 2.3 mm）
    flat_y = WRIST_Y - math.sqrt(r ** 2 - (SERVO_W / 2 + FIT + 2.3) ** 2)
    housing &= box(-r - 1, r + 1, flat_y, -WALL + 0.01, BODY_BOTTOM, BODY_TOP)
    part += housing
    # 小舵盤 + 背面螺絲頭轉動的空間（小齒輪本身在機身前面）
    part -= Pos(0, 0.01, 0) * Rot(90, 0, 0) * Cylinder(
        MICRO_HORN_D / 2 + 1.0, MICRO_HORN_T + MICRO_HORN_SPACE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 伺服馬達口袋（從背面放進去）：本體 + 固定耳，固定耳前表面靠在座面上
    ear = box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, WRIST_Y - r - 1, EAR_FRONT_Y,
              SERVO_CZ - SERVO_EAR_L / 2 - FIT, SERVO_CZ + SERVO_EAR_L / 2 + FIT)
    top_case = box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, WRIST_Y - r - 1, 0,
                   SERVO_CZ - SERVO_L / 2 - FIT, SERVO_CZ + SERVO_L / 2 + FIT)
    part -= ear + top_case
    for dz in (-SERVO_HOLE_DL / 2, SERVO_HOLE_DL / 2):
        for dx in (-SERVO_HOLE_DW / 2, SERVO_HOLE_DW / 2):
            # 孔從固定耳座面往前（+Y）鑽進機身，螺絲從背面口袋鎖進來
            part -= Pos(dx, EAR_FRONT_Y, SERVO_CZ + dz) * Rot(90, 0, 0) * fx.tap_hole("M3", 8)
    # 手腕 MATRIX 大舵盤：M4×12 從上往下穿過底板和舵盤，螺帽在舵盤下面。
    # 底部每側開一個朝外的窗口：螺絲從側面放進去；上方只留 Ø5 細孔讓扳手/起子伸下來鎖
    tr = BUTTON_HEAD_D / 2 + 0.5
    web_top = BODY_BOTTOM + WRIST_WEB
    for hx, hy in WRIST_HOLES:
        part -= Pos(hx, WRIST_Y + hy, web_top) * fx.clearance_hole("M4", WRIST_WEB)
        part -= Pos(hx, WRIST_Y + hy, web_top) * Cylinder(DRIVER_HOLE_D / 2, BODY_TOP - web_top + 1,
                                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in (1, -1):
        ys = [WRIST_Y + hy for hx, hy in WRIST_HOLES if hx * sx > 0]
        # 窗口往後多開 3 mm，讓它乾淨地切穿後面的圓弧（不然邊緣會留下薄片）
        part -= box(sx * (16.0 - tr), sx * (HORN_D / 2 + 2.0), min(ys) - tr - 3.0, max(ys) + tr,
                    web_top, web_top + WRIST_WINDOW_H)
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
    print(f"全開 {OPENING:.0f} mm（每邊 {STROKE:.1f}），小齒輪要轉 {PINION_TURN_DEG:.0f}°；"
          f"齒條長 {RACK_LEN:.1f}，全關時外端 X={JAW_END:.1f}")
    # STEP 用組裝座標；列印檔放在 PRINT/parallel_gripper/（轉好方向、排成一盤、附零件清單）
    parts = {"body": body(), "pinion": pinion(), "jaw": jaw()}
    for n, part in parts.items():
        save(part, f"parallel_gripper_{n}", formats=("step",))
    wrist = fx.pick_screw("M4", WRIST_WEB + HORN_T, nut=True)          # 底板 + 大舵盤，螺帽在舵盤下面
    gear = fx.pick_screw("M4", MICRO_HORN_T, into=GEAR_W)                # 小舵盤背面鎖進小齒輪
    ear = fx.pick_screw("M3", SERVO_EAR_T, into=8.0)                     # 伺服固定耳鎖進機身
    save_print("parallel_gripper", [
        ("機身", parts["body"], 1, Rot(0, 0, 0)),        # 底面朝下，6 個方向裡支撐最少
        ("小齒輪", parts["pinion"], 1, "auto"),
        ("夾爪", parts["jaw"], 2, "auto"),
    ], bom=[
        ("伺服馬達 MG996R（360°）", 1, "驅動小齒輪"),
        ("MATRIX 小舵盤 13-0012", 1, "伺服 ↔ 小齒輪"),
        ("MATRIX 大舵盤 13-0002", 1, "手腕伺服 ↔ 機身（手腕上原本那個）"),
        (f"M4×{wrist['length']} 內六角螺絲", len(WRIST_HOLES), "機身底板鎖大舵盤（從側窗放、上方細孔鎖）"),
        ("M4 螺帽", len(WRIST_HOLES), "套在大舵盤下面"),
        (f"M4×{gear['length']} 螺絲", len(PINION_HOLES), "小舵盤鎖小齒輪"),
        (f"M3×{ear['length']} 螺絲", 4, "伺服固定耳鎖機身"),
    ])
    save(assembly(0), "parallel_gripper_assembly_closed", formats=("step",))
    save(assembly(OPENING), "parallel_gripper_assembly_open", formats=("step",))
