"""印表機與 FDM 列印的設計限制。

目前還沒有印表機，數值是一般 FDM（0.4 mm 噴頭）的保守預設值，
來源：text-to-cad dfam-check 的 process-limits（Hubs / ISO/ASTM 52910）。
買了印表機、印過孔徑測試片之後，再回來校正這裡的數字。
"""

PROFILE = {
    "printer": None,              # 例如 "Bambu Lab A1"；還沒有印表機
    "bed_mm": None,               # 列印平台 (X, Y, Z)，例如 (256, 256, 256)
    "nozzle_mm": 0.4,
    "layer_mm": 0.2,
    "min_wall_mm": 1.2,           # 有支撐的牆
    "min_wall_unsupported_mm": 1.6,
    "overhang_deg": 45.0,         # 從水平面量起，小於這個角度的朝下面需要支撐
    "min_hole_mm": 2.0,
    "min_feature_mm": 0.8,
    "max_bridge_mm": 10.0,
    "hole_comp_mm": 0.0,          # 孔徑補償：校正後若孔偏小，填正值（直徑加大量）
}
