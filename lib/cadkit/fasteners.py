"""螺絲孔、螺帽槽等「用來挖掉」的工具形狀。

每個函式回傳一個實體，放到正確位置後用減法挖掉：

    part -= Pos(10, 0, 0) * fx.clearance_hole("M3", depth=6)

孔的方向：沿 -Z 往下挖，孔口在 Z=0。要從別的面挖，就先旋轉再移動。
數值是 FDM 的起始值，印過孔徑測試片後要校正（見 printer.PROFILE["hole_comp_mm"]）。
"""

from __future__ import annotations

from build123d import Align, Cylinder, Pos, RegularPolygon, extrude

from cadkit.printer import PROFILE

# 公制螺絲資料（mm）。
#   clear: 螺絲穿過的孔        tap: 螺絲直接鎖進塑膠的孔
#   nut_af: 六角螺帽對邊寬     nut_h: 螺帽厚度
#   head_d / head_h: 內六角圓柱頭螺絲的頭部直徑與高度（沉頭孔用）
METRIC = {
    "M2":   {"clear": 2.4, "tap": 1.7, "nut_af": 4.0, "nut_h": 1.6, "head_d": 3.8, "head_h": 2.0},
    "M2.5": {"clear": 2.9, "tap": 2.2, "nut_af": 5.0, "nut_h": 2.0, "head_d": 4.5, "head_h": 2.5},
    "M3":   {"clear": 3.4, "tap": 2.6, "nut_af": 5.5, "nut_h": 2.4, "head_d": 5.5, "head_h": 3.0},
    "M4":   {"clear": 4.5, "tap": 3.5, "nut_af": 7.0, "nut_h": 3.2, "head_d": 7.0, "head_h": 4.0},
    "M5":   {"clear": 5.5, "tap": 4.4, "nut_af": 8.0, "nut_h": 4.0, "head_d": 8.5, "head_h": 5.0},
}

# 熱熔螺母（只有在指定時才用）。M3 數值來自 DigiKey/CNC Kitchen 教學，其他尺寸尚未查證。
HEAT_SET = {
    "M3": {"hole": 3.9, "length": 4.0},
}

# 內六角圓柱頭螺絲（ISO 4762）常見買得到的長度（mm），以及螺距
STOCK_LENGTHS = {
    "M2":   [3, 4, 5, 6, 8, 10, 12, 16, 20],
    "M2.5": [4, 5, 6, 8, 10, 12, 16, 20, 25],
    "M3":   [4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40],
    "M4":   [5, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40],
    "M5":   [6, 8, 10, 12, 14, 16, 20, 25, 30, 35, 40],
}
PITCH = {"M2": 0.4, "M2.5": 0.45, "M3": 0.5, "M4": 0.7, "M5": 0.8}

NUT_CLEARANCE = 0.3   # 螺帽槽對邊寬多留的空隙
HEAD_CLEARANCE = 0.5  # 沉頭孔直徑多留的空隙


def _spec(size: str) -> dict:
    try:
        return METRIC[size]
    except KeyError:
        raise ValueError(f"沒有 {size} 的資料，可用：{', '.join(METRIC)}") from None


def _hole(diameter: float, depth: float):
    d = diameter + PROFILE["hole_comp_mm"]
    return Cylinder(d / 2, depth, align=(Align.CENTER, Align.CENTER, Align.MAX))


def clearance_hole(size: str, depth: float, counterbore: bool = False):
    """螺絲穿過的孔；counterbore=True 會加沉頭孔，讓螺絲頭埋進去。"""
    s = _spec(size)
    tool = _hole(s["clear"], depth)
    if counterbore:
        tool += _hole(s["head_d"] + HEAD_CLEARANCE, s["head_h"] + 0.2)
    return tool


def tap_hole(size: str, depth: float):
    """螺絲直接鎖進塑膠的孔（自攻）。適合很少拆的地方。"""
    return _hole(_spec(size)["tap"], depth)


def nut_trap(size: str, depth: float | None = None):
    """六角螺帽槽，孔口在 Z=0 往下。depth 預設為螺帽厚度加 0.2。

    要搭配 clearance_hole 一起挖，螺絲才穿得過去。
    """
    s = _spec(size)
    depth = s["nut_h"] + 0.2 if depth is None else depth
    af = s["nut_af"] + NUT_CLEARANCE
    hexagon = RegularPolygon(af / 2, 6, major_radius=False)
    return Pos(0, 0, -depth) * extrude(hexagon, depth)


def heat_set_hole(size: str):
    """熱熔螺母孔：深度比螺母長 1 mm，孔周圍牆厚要至少 1.6 mm。"""
    try:
        s = HEAT_SET[size]
    except KeyError:
        raise ValueError(f"熱熔螺母目前只有 {', '.join(HEAT_SET)} 的資料") from None
    return _hole(s["hole"], s["length"] + 1.0)


def pick_screw(size: str, grip: float, nut: bool = False, into: float | None = None) -> dict:
    """挑買得到的螺絲長度（做法參考 openscad-skill 的 hardware.py pick）。

    grip：螺絲頭底面到被夾零件最遠那一面的厚度。
    nut=True：後面鎖螺帽，挑「穿過螺帽再多 2 牙」的最短長度。
    into=深度：直接鎖進塑膠孔，挑「不會頂到孔底（留 0.5 mm）」的最長長度。
    """
    s, lengths, pitch = _spec(size), STOCK_LENGTHS[size], PITCH[size]
    if nut:
        need = grip + s["nut_h"] + 2 * pitch
        ok = [L for L in lengths if L >= need]
        if not ok:
            raise ValueError(f"{size}：需要 ≥ {need:.1f} mm，超過常見長度 {lengths[-1]} mm")
        L = ok[0]
        res = {"size": size, "length": L, "engage_mm": min(L - grip, s["nut_h"]), "stick_out_mm": L - grip - s["nut_h"]}
        note = f"螺帽全部吃到，凸出螺帽 {res['stick_out_mm']:.1f} mm"
    elif into is not None:
        ok = [L for L in lengths if L - grip <= into - 0.5 and L > grip]
        if not ok:
            raise ValueError(f"{size}：夾住 {grip} mm、孔深 {into} mm，找不到不會頂到底的長度")
        L = ok[-1]
        res = {"size": size, "length": L, "engage_mm": L - grip}
        note = f"鎖進塑膠 {res['engage_mm']:.1f} mm" + ("（少於直徑的 1.5 倍，可能不夠牢）"
                                                      if res["engage_mm"] < 1.5 * float(size[1:]) else "")
    else:
        raise ValueError("要指定 nut=True 或 into=孔深")
    print(f"{size}×{L}：夾住 {grip:.1f} mm，{note}")
    return res
