"""機構檢查：零件兩兩之間有沒有重疊、重疊多深、會動的零件之間留了多少間隙。

做法參考 openscad-skill 的 asm.py --clash --clear 和 cad-khana 的 assert_distance：
重疊要看「深度」而不是只看體積（薄薄一片也可能有好幾 mm³），
會動的零件之間除了不重疊，還要留最小間隙，否則印出來會磨擦卡住。

    from cadkit.mechanism import check_pairs
    problems = check_pairs({"body": b, "jaw": j}, gap=0.2, gap_pairs=[("body", "jaw")])
"""

from __future__ import annotations

import itertools

MIN_VOLUME = 0.01   # mm³，小於這個當作只是面貼面


def overlap(a, b) -> dict | None:
    """兩個零件重疊的體積、深度（重疊部分外框的最短邊）和位置；沒重疊回傳 None。"""
    common = a & b
    vol = common.volume if common is not None else 0.0
    if vol < MIN_VOLUME:
        return None
    bb = common.bounding_box()
    c = bb.center()
    return {"volume_mm3": vol, "depth_mm": min(bb.size.X, bb.size.Y, bb.size.Z),
            "at": (round(c.X, 1), round(c.Y, 1), round(c.Z, 1))}


def check_pairs(parts: dict, gap: float = 0.2, gap_pairs=(), skip=(), depth_ok: float = 0.0) -> list[str]:
    """檢查所有零件兩兩重疊，以及 gap_pairs 裡的零件之間至少留 gap mm。

    skip：不檢查的配對（例如鎖在一起的小齒輪和舵盤）。
    depth_ok：允許的重疊深度（例如齒輪嚙合的網格誤差），預設 0。
    回傳問題清單（空的就是通過）。
    """
    skip = {frozenset(p) for p in skip}
    gap_set = {frozenset(p) for p in gap_pairs}
    problems = []
    for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
        key = frozenset((na, nb))
        if key in skip:
            continue
        o = overlap(a, b)
        if o and o["depth_mm"] > depth_ok:
            problems.append(f"{na}×{nb} 重疊 深 {o['depth_mm']:.2f} mm、{o['volume_mm3']:.1f} mm³，位置 {o['at']}")
            continue
        if key in gap_set:
            d = a.distance_to(b)
            if d < gap:
                problems.append(f"{na}↔{nb} 間隙只有 {d:.2f} mm（要 ≥ {gap} mm）")
    return problems


def min_gap(a, b) -> float:
    """兩個零件之間的最短距離（重疊時為 0）。"""
    return a.distance_to(b)
