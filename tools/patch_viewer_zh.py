"""把預覽器（ocp_viewer）的介面改成繁體中文。

    python tools/patch_viewer_zh.py            # 套用中文（可以重複執行）
    python tools/patch_viewer_zh.py --restore  # 還原成英文

ocp_viewer 沒有語言設定，這個工具直接替換它網頁程式裡的顯示文字。
套件更新後中文會被蓋掉，再執行一次即可。改完要在瀏覽器按 Ctrl+F5 重新整理。
只替換「顯示用」的文字；程式判斷用的值（選單的 value、零件名稱）不動。
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import ocp_viewer

JS = Path(ocp_viewer.__file__).parent / "server" / "static" / "js" / "three-cad-viewer.esm.js"
BACKUP = JS.with_name(JS.name + ".orig")
MARKER = "\n/* zh-TW patched by tools/patch_viewer_zh.py */\n"

# 工具列按鈕的提示文字（JS 字串）
TOOLTIPS = {
    "Show axes": "顯示座標軸",
    "Show axes at origin (0,0,0)": "在原點 (0,0,0) 顯示座標軸",
    "Show grid": "顯示格線",
    "Use perspective camera": "透視視角（關閉為正交視角）",
    "Show transparent faces": "半透明顯示",
    "Show black edges": "黑色邊線",
    "Reset view": "重設視角",
    "Resize object": "縮放到適合畫面",
    "Switch to iso view": "等角視圖",
    "Switch to front view": "前視圖",
    "Switch to back view": "後視圖",
    "Switch to top view": "上視圖",
    "Switch to bottom view": "下視圖",
    "Switch to left view": "左視圖",
    "Switch to right view": "右視圖",
    "Explode tool": "爆炸圖（把零件分開）",
    "Scale along the Z-axis": "沿 Z 軸縮放",
    "Measure distance between shapes": "量測距離（點選兩個面、邊或點）",
    "Show shape properties": "查看形狀資訊（點選一個面、邊或點）",
    "Copy shape IDs to clipboard": "複製形狀編號",
    "Pin viewer as png": "存成圖片",
    "Help": "說明",
    "Select object by double-click first": "請先雙擊選取物件",
    "Navigation Tree": "零件清單",
    "Clipping Tool": "剖面工具",
    "Zebra Tool": "斑馬紋（檢查曲面）",
    "Material Selection": "材質",
    "Studio Mode": "展示渲染",
}

# 量測面板標題（含前後文，避免改到其他地方）
EXACT = {
    '? "Distance (mesh based)" : "Distance"': '? "距離（依網格估算）" : "距離"',
    '? "Properties (mesh based)" : "Properties"': '? "形狀資訊（依網格估算）" : "形狀資訊"',
    'title=\\"Reset to original material\\"': 'title=\\"還原原本材質\\"',
    # 篩選選單用顯示文字判斷選了什麼：中文先轉回英文再判斷，畫面顯示中文
    "this.elements.value.innerText = topoType;":
        'const zhToEn = {"全部": "All", "無": "None", "實體": "Solid", "面": "Face", "邊": "Edge", "點": "Vertex"};'
        ' const enToZh = {all: "全部", none: "無", solid: "實體", face: "面", edge: "邊", vertex: "點"};'
        " topoType = zhToEn[topoType] ?? topoType;"
        " this.elements.value.innerText = enToZh[topoType.toLowerCase()] ?? topoType;",
}

# 左側面板 HTML 樣板裡的文字（樣板是 JS 字串，引號寫成 \"）
TAB_VALUES = {
    "Tree": "零件",
    "Clip": "剖面",
    "Zebra": "斑馬紋",
    "Material": "材質",
    "Studio": "渲染",
}

PANEL_TOOLTIPS = {
    "Collpase nodes with a single leaf": "收合只有一個子項的節點",
    "Collpase tree": "全部收合",
    "Edit material of selected object": "編輯選取物件的材質",
    "Expand root node only": "只展開第一層",
    "Expand tree": "全部展開",
    "Open/close info box": "開關資訊欄",
    "Pause animation": "暫停動畫",
    "Play animation": "播放動畫",
    "Reset to default clip planes": "重設剖面",
    "Reset to default values": "重設為預設值",
    "Reset to default zebra settings": "重設斑馬紋設定",
    "Reset to original values": "還原原本數值",
    "Select stripe color scheme": "條紋配色",
    "Select stripe mapping": "條紋對應方式",
    "Set blue clipping plane to view direction": "藍色剖面對齊目前視角",
    "Set green clipping plane to view direction": "綠色剖面對齊目前視角",
    "Set red clipping plane to view direction": "紅色剖面對齊目前視角",
    "Show clipping planes": "顯示剖面",
    "Stop and reset animation": "停止並重設動畫",
    "Use intersection clipping": "只保留交集區域",
    "Use object color caps instead of RGB": "切面用物件顏色（不用紅綠藍）",
}

TEXT_NODES = {
    "&lt;ESC&gt;": "&lt;ESC 鍵&gt;",
    "&lt;left mouse button&gt; double click": "&lt;滑鼠左鍵&gt; 雙擊",
    "&lt;left mouse button&gt;": "&lt;滑鼠左鍵&gt;",
    "&lt;mouse wheel&gt; or &lt;middle mouse button&gt;": "&lt;滾輪&gt; 或 &lt;滑鼠中鍵&gt;",
    "&lt;right mouse button&gt;": "&lt;滑鼠右鍵&gt;",
    "(Shows axis-aligned bounding box, AABB)": "（顯示外框尺寸 AABB）",
    "AO Intensity": "環境遮蔽強度",
    "All": "全部",
    "Ambient Light": "環境光",
    "Background": "背景",
    "Button '1' (all nodes with one leaf only)": "按鈕「1」（收合只有一個子項的節點）",
    "Button 'C'": "按鈕「C」",
    "Button 'E'": "按鈕「E」",
    "Button 'R'": "按鈕「R」",
    "CAD Object Tree": "零件清單",
    "Click on navigation tree label": "點零件清單裡的名稱",
    "Collapse all nodes": "全部收合",
    "Collapse single leafs": "收合單一子項",
    "Colors": "顏色",
    "Direct Light": "直射光",
    "Direction": "方向",
    "Distance": "距離",
    "Edge": "邊",
    "Env Intensity": "環境光強度",
    "Env Rotation": "環境旋轉",
    "Environment": "環境",
    "Expand all nodes": "全部展開",
    "Expand root only": "只展開第一層",
    "Exposure": "曝光",
    "Face": "面",
    "Filter object types": "篩選選取類型",
    "Gradient Dark Grey": "深灰漸層",
    "Gradient Grey": "灰色漸層",
    "Gray": "灰階",
    "Hide element": "隱藏這個物件",
    "Hide other elements": "隱藏其他物件",
    "Info": "資訊",
    "Intersection": "交集",
    "Isolate element": "只顯示這個物件",
    "Linear (none)": "線性（無）",
    "Material Editor": "材質編輯",
    "Measure Mode": "量測模式",
    "Metalness": "金屬感",
    "Mouse Navigation": "滑鼠操作",
    "Mouse Selection": "滑鼠選取",
    "Normal": "法線",
    "Pan": "平移",
    "Pick element": "選取物件",
    "Planes": "剖面",
    "Properties": "屬性",
    "Reflection": "反射",
    "Rotate left / right": "左右旋轉",
    "Rotate up / down": "上下旋轉",
    "Rotate": "旋轉",
    "Roughness": "粗糙度",
    "Select 1. (and 2.) object": "選第 1（和第 2）個物件",
    "Set camera target": "設為旋轉中心",
    "Shadow Intensity": "陰影強度",
    "Shadow Softness": "陰影柔和度",
    "Shape": "形狀",
    "Solid Dark Grey": "深灰",
    "Solid Grey": "灰色",
    "Solid White": "白色",
    "Solid": "實體",
    "Stripe Count": "條紋數量",
    "Stripe Opacity": "條紋透明度",
    "Texture Mapping": "貼圖方式",
    "Tone Mapping": "色調對應",
    "Tools": "工具",
    "Transparent": "透明",
    "Type menu or &lt;n&gt;one, &lt;s&gt;olid, &lt;f&gt;ace, &lt;e&gt;dge , &lt;v&gt;ertices":
        "用選單，或按 &lt;n&gt;無、&lt;s&gt;實體、&lt;f&gt;面、&lt;e&gt;邊、&lt;v&gt;點",
    "Unselect all objects": "取消全部選取",
    "Unselect last object": "取消上一個選取",
    "Use 4K maps": "使用 4K 貼圖",
    "Use center instead of min distance": "量中心距離（不量最短距離）",
    "Use object color caps": "切面用物件顏色",
    "Vertex": "點",
    "Zoom": "縮放",
}


def replacements() -> list[tuple[str, str]]:
    pairs = [(f'"{en}"', f'"{zh}"') for en, zh in TOOLTIPS.items()]
    pairs += list(EXACT.items())
    pairs += [(f'value=\\"{en}\\"', f'value=\\"{zh}\\"') for en, zh in TAB_VALUES.items()]
    pairs += [(f'data-tooltip=\\"{en}\\"', f'data-tooltip=\\"{zh}\\"') for en, zh in PANEL_TOOLTIPS.items()]
    # 長的先換，避免短字串先吃掉長字串的一部分
    pairs += sorted(((f">{en}<", f">{zh}<") for en, zh in TEXT_NODES.items()), key=lambda p: -len(p[0]))
    return pairs


def original_text() -> str:
    text = JS.read_text(encoding="utf-8")
    if MARKER in text:
        return BACKUP.read_text(encoding="utf-8")
    shutil.copyfile(JS, BACKUP)  # 第一次或套件更新後：保存最新的原檔
    return text


def apply() -> None:
    text = original_text()
    missing = []
    total = 0
    for old, new in replacements():
        n = text.count(old)
        if n == 0:
            missing.append(old)
        total += n
        text = text.replace(old, new)
    JS.write_text(text + MARKER, encoding="utf-8")
    print(f"已改成中文：替換 {total} 處")
    if missing:
        print(f"有 {len(missing)} 個詞在這個版本找不到（可能套件改版了）：")
        for m in missing:
            print("  ", m)
    print("請在預覽器頁面按 Ctrl+F5 重新整理")


def restore() -> None:
    if not BACKUP.exists():
        print("沒有備份，看起來從來沒改過")
        return
    if MARKER in JS.read_text(encoding="utf-8"):
        shutil.copyfile(BACKUP, JS)
    print("已還原成英文，請在預覽器頁面按 Ctrl+F5 重新整理")


if __name__ == "__main__":
    restore() if sys.argv[1:] == ["--restore"] else apply()
