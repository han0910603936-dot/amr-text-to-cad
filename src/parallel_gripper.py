"""平行夾爪（長盒式齒輪齒條）。

參考 MakerWorld「齿轮齿条平行夹爪」(models/2188993) 與 PapaBravo 的齒輪齒條夾爪 (models/940813)：
一個長盒把兩條齒條整段包住導引，中間一個小齒輪同時推動兩條齒條反向移動，
夾爪從前蓋的長槽伸出。盒子兩端兼作全開的機械限位。

零件（4 種）：
  box     長盒：背面裝夾爪伺服馬達（直立、在中心線上），手腕 MATRIX 舵盤直接鎖在盒底、對準重心
  cover   前蓋：上下兩條長槽讓夾爪伸出，4 角 M3 鎖在盒子兩端
  pinion  小齒輪：32 齒、模數 1.5，用 4 顆 M4 鎖在夾爪伺服馬達的 MATRIX 舵盤上
  jaw     齒條＋夾片一體：上下兩支是同一個零件（下面那支繞 Y 軸轉 180°），印 2 個

座標：X = 開合方向，Y = 往前（夾片伸出方向），Z = 上下。
原點 = 小齒輪中心、小齒輪背面（貼舵盤那面）。整個夾爪對 X=0 左右對稱。

舵盤：MATRIX RC Servo Horn Wheel（13-0002），尺寸取自官方 Studio 2.0 零件檔 MX13-0002.dat
（1 LDU = 0.4 mm）：Ø47、厚 3.2、12 個 Ø4.2 孔在 8 mm 格線上。MATRIX 結構螺絲是 M4。

⚠ 假設（要量實物確認，見「夾爪伺服馬達」參數）：
  - 夾爪伺服馬達是標準尺寸（MG996R 一類，40.7 × 19.7 mm），可轉 180°
  - 裝上舵盤後，固定耳表面到舵盤上表面 15.5 mm
"""

import math

from bd_warehouse.gear import RackGear, SpurGear
from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from cadkit import fasteners as fx
from cadkit import save

# ---------- 齒輪 ----------
MODULE = 1.5
PINION_TEETH = 32                     # 節圓直徑 48；4 顆 M4 沉頭孔外還有約 2 mm 到齒根
PRESSURE_ANGLE = 20.0
GEAR_W = 8.0                          # 齒寬（沿 Y）
PITCH_R = MODULE * PINION_TEETH / 2   # 24
PITCH = math.pi * MODULE

# ---------- 行程 ----------
SERVO_RANGE_DEG = 180.0
STROKE = math.radians(SERVO_RANGE_DEG) * PITCH_R   # 每邊 75.4 mm，全開 150.8 mm

# ---------- 齒條夾爪 ----------
RACK_TEETH = 21                       # 奇數，長 99 mm
RACK_LEN = RACK_TEETH * PITCH
RACK_BASE = 8.0                       # 齒根到齒條背面的厚度
RACK_TOP = PITCH_R + 1.25 * MODULE + RACK_BASE      # 齒條背面（離中心最遠）33.9
# 夾爪全關時齒條外端的 X：讓齒條和小齒輪正上方的齒/齒間對上
_PHASE = 0.5 if PINION_TEETH % 4 == 0 else 0.0
JAW_END = (RACK_TEETH / 2 - 8 + _PHASE) * PITCH     # 14.1
TAB_Z = (PITCH_R + 1.25 * MODULE + 0.6, RACK_TOP - 1.2)   # 伸出前蓋的凸片高度範圍
TAB_W = 8.0                           # 凸片沿 X 的寬度
FINGER_T = 5.0                        # 夾片厚
FINGER_LEN = 60.0                     # 夾片往前伸出長度
FINGER_H = 35.0                       # 夾片上下範圍 ±35
FIT = 0.3                             # 滑動間隙

# ---------- 長盒 ----------
WALL = 3.0                            # 上下牆厚
END_WALL = 6.0                        # 兩端牆厚（兼限位、前蓋螺絲）
COVER_T = 4.0
IN_Z = RACK_TOP + FIT                 # 盒內上下 ±34.2
# 盒內 X 範圍：齒條在全關到全開之間都要在盒內
IN_X = max(RACK_LEN - JAW_END, JAW_END + STROKE) + 0.5
COVER_Y0 = GEAR_W + FIT               # 前蓋內面
BACK_FACE_Y = -FIT                    # 齒條背面滑動面（盒內實心背板的前表面）

# ---------- 夾爪伺服馬達（假設：標準尺寸，直立擺放）----------
SERVO_L, SERVO_W = 40.7, 19.7         # 本體長（沿 Z，直立）、寬（沿 X）
SERVO_SHAFT_FROM_END = 10.2           # 輸出軸到本體近端距離（軸靠下端，本體往上，固定耳不會碰到底部墊塊）
SERVO_HOLE_DL, SERVO_HOLE_DW = 49.5, 10.0
FLANGE_TO_HORN_TOP = 15.5
SERVO_MASS_G = 55.0                   # 估重心用

# ---------- MATRIX 舵盤 ----------
HORN_D, HORN_T = 47.0, 3.2
HORN_HUB_SPACE = 4.0                  # 舵盤背面留給 M4 螺帽的空間
BUTTON_HEAD_D, BUTTON_HEAD_H = 7.6, 2.2
PINION_HOLES = [(16.0, 0.0), (-16.0, 0.0), (0.0, 16.0), (0.0, -16.0)]
# 手腕舵盤用的孔：要落在盒底實心導引帶（背板背面 ~ 齒條背面之間）的範圍內
FOOT_HOLES = [(16.0, 0.0), (-16.0, 0.0), (16.0, -8.0), (-16.0, -8.0)]
FOOT_TAP_DEPTH = 12.0                 # M4×16：舵盤 3.2 + 吃進盒底 12
FOOT_Y = -4.0                         # 手腕舵盤中心的 Y = 估計重心（見 checks/parallel_gripper_check.py）

# 由上面推算
BACK_WALL_BACK_Y = -FLANGE_TO_HORN_TOP           # 伺服固定耳貼在背板背面
SERVO_CZ = SERVO_L / 2 - SERVO_SHAFT_FROM_END    # 伺服本體中心 Z（軸在原點、靠下端）→ 本體往上
BACK_T = 4.0                          # 背板厚（兩條導引帶之間）
BAND_Z0 = PITCH_R - MODULE - 1.0      # 導引帶內緣高度（齒條齒尖再往內 1 mm）
HOUSING_HALF_W = HORN_D / 2 + 1.0 + 4.0   # 中央伺服座半寬


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
    return Rot(-90, 0, 0) * gear      # 軸改成沿 +Y，背面在 Y=0


def jaw():
    """上方那支（夾爪全關）。齒條在盒內，凸片穿過前蓋，夾片在盒子前面。"""
    rack = RackGear(MODULE, RACK_TEETH, PRESSURE_ANGLE, thickness=GEAR_W, base_height=RACK_BASE)
    rack = Pos(JAW_END - RACK_LEN / 2, GEAR_W / 2, PITCH_R) * Rot(180, 0, 0) * rack   # 齒朝下
    y_front = COVER_Y0 + COVER_T + 0.5
    tab = box(JAW_END - TAB_W, JAW_END, GEAR_W - 0.5, y_front + 6.0, *TAB_Z)
    bridge = box(0, JAW_END, y_front, y_front + 6.0, *TAB_Z)           # 凸片接到夾片
    finger = box(0, FINGER_T, y_front, y_front + FINGER_LEN, -FINGER_H, FINGER_H)
    part = rack + tab + bridge + finger
    part.label = "jaw"
    return part


def body_box():
    """背面是一整個平面（背面朝下印、免支撐）：齒條後面是實心導引帶，中間挖空。"""
    x_out = IN_X + END_WALL
    z_out = IN_Z + WALL
    shell = box(-x_out, x_out, BACK_WALL_BACK_Y, COVER_Y0, -z_out, z_out)
    shell += box(-HOUSING_HALF_W / 2, HOUSING_HALF_W / 2, BACK_WALL_BACK_Y, -5.0, z_out - 0.01, z_out + 3.0)  # 固定耳孔加厚
    shell -= box(-IN_X, IN_X, BACK_FACE_Y, COVER_Y0 + 1, -IN_Z, IN_Z)          # 齒條滑道
    shell -= box(-IN_X, IN_X, BACK_WALL_BACK_Y + BACK_T, BACK_FACE_Y + 0.01,   # 中間挖空（兩條導引帶之間）
                 -BAND_Z0, BAND_Z0)
    # 小齒輪 + 舵盤 + 螺帽 轉動的空間
    shell -= Pos(0, BACK_FACE_Y + 0.01, 0) * Rot(90, 0, 0) * Cylinder(
        HORN_D / 2 + 1.0, HORN_T + HORN_HUB_SPACE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 伺服馬達上半部、舵盤軸套穿過背板（固定耳貼在背板背面）
    shell -= box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, BACK_WALL_BACK_Y - 1, -(HORN_T + HORN_HUB_SPACE),
                 SERVO_CZ - SERVO_L / 2 - FIT, SERVO_CZ + SERVO_L / 2 + FIT)
    for dz in (-SERVO_HOLE_DL / 2, SERVO_HOLE_DL / 2):
        for dx in (-SERVO_HOLE_DW / 2, SERVO_HOLE_DW / 2):
            shell -= Pos(dx, BACK_WALL_BACK_Y, SERVO_CZ + dz) * Rot(-90, 0, 0) * fx.tap_hole("M3", 10)
    # 前蓋螺絲（盒子兩端牆，4 角）
    for sx in (-1, 1):
        for sz in (-1, 1):
            shell -= Pos(sx * (IN_X + END_WALL / 2), COVER_Y0, sz * (IN_Z - 4)) * Rot(90, 0, 0)                 * fx.tap_hole("M3", 10)
    # 手腕 MATRIX 舵盤直接鎖在盒底，中心在 (0, FOOT_Y)；M4 鎖進下方導引帶
    for hx, hy in FOOT_HOLES:
        shell -= Pos(hx, FOOT_Y + hy, -z_out) * Rot(180, 0, 0) * fx.tap_hole("M4", FOOT_TAP_DEPTH)
    shell.label = "box"
    return shell


def cover():
    x_out = IN_X + END_WALL
    z_out = IN_Z + WALL
    part = box(-x_out, x_out, COVER_Y0, COVER_Y0 + COVER_T, -z_out, z_out)
    # 上槽給上方夾爪（往 +X 開），下槽給下方夾爪（往 -X 開）
    x0, x1 = JAW_END - TAB_W - 0.6, JAW_END + STROKE + 0.6
    z0, z1 = TAB_Z[0] - 0.4, TAB_Z[1] + 0.4
    part -= box(x0, x1, COVER_Y0 - 1, COVER_Y0 + COVER_T + 1, z0, z1)
    part -= box(-x1, -x0, COVER_Y0 - 1, COVER_Y0 + COVER_T + 1, -z1, -z0)
    part -= Pos(0, COVER_Y0 + COVER_T, 0) * Rot(-90, 0, 0) * Cylinder(5.0, COVER_T * 3)  # 鎖舵盤中心螺絲
    for sx in (-1, 1):
        for sz in (-1, 1):
            part -= Pos(sx * (IN_X + END_WALL / 2), COVER_Y0 + COVER_T, sz * (IN_Z - 4)) \
                * Rot(-90, 0, 0) * fx.clearance_hole("M3", COVER_T)
    part.label = "cover"
    return part


def assembly(opening: float = 0.0):
    """opening = 兩夾片間距（mm），0 = 全關。"""
    shift = opening / 2
    angle = math.degrees(shift / PITCH_R)
    parts = [
        body_box(), cover(),
        Rot(0, angle, 0) * pinion(),                     # 上方齒條往 +X，齒輪頂端也往 +X
        Pos(shift, 0, 0) * jaw(),
        Rot(0, 180, 0) * Pos(shift, 0, 0) * jaw(),
    ]
    for p, name in zip(parts, ["box", "cover", "pinion", "jaw_top", "jaw_bottom"]):
        p.label = name
    return Compound(children=parts, label="parallel_gripper")


if __name__ == "__main__":
    print(f"每邊行程 {STROKE:.1f} mm，全開 {2 * STROKE:.1f} mm（伺服轉 {SERVO_RANGE_DEG:.0f}°）")
    save(body_box(), "parallel_gripper_box")
    save(cover(), "parallel_gripper_cover")
    save(pinion(), "parallel_gripper_pinion")
    save(jaw(), "parallel_gripper_jaw")
    save(assembly(0), "parallel_gripper_assembly_closed", formats=("step",))
    save(assembly(2 * STROKE), "parallel_gripper_assembly_open", formats=("step",))
