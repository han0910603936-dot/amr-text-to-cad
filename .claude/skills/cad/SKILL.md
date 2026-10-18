---
name: cad
description: 用 build123d 建立或修改這個專案的 3D 列印零件（主要是自主移動機器人的結構件：支架、馬達座、感測器座、電池盒、底盤），匯出 STEP/STL/3MF，做 FDM 列印檢查（DFM）、截圖自我檢查並在瀏覽器預覽。使用者要求做零件、改尺寸、檢查能不能印、或預覽模型時使用。
---

# 專屬 CAD 建模流程

參考 [earthtojake/text-to-cad](https://github.com/earthtojake/text-to-cad)（MIT）的做法，精簡成這個專案用的版本。
研究背景見專案根目錄的 `研究筆記.md`。

## 環境

- Python 環境在 OneDrive 外面：`C:\Users\jeff2\.cad3d\venv`（Python 3.13，套件版本見 `requirements.txt`）。
- 下面的 `python` 一律指 `C:/Users/jeff2/.cad3d/venv/Scripts/python.exe`，加上 `-X utf8`，從專案根目錄執行。
- `lib/` 已透過 `.pth` 加進環境，模型裡可以直接 `from cadkit import ...`。
- 執行時出現 `cannot open font 'C:\Windows\Fonts\mstmc.ttf'` 是無害的警告，可以忽略。
- 要加裝套件時，`uv` 要設 `UV_CACHE_DIR=C:\Users\jeff2\.cad3d\cache`、`UV_PYTHON_INSTALL_DIR=C:\Users\jeff2\.cad3d\python`。
  桌面版 App 會把 `AppData\Roaming` 重新導向，uv 用預設位置會出錯。

## 資料夾

```
src/<零件>.py      一個零件一個檔，存檔名與檔名相同
lib/cadkit/        共用工具：save()、fasteners（螺絲孔）、printer.PROFILE（列印限制）
tools/             dfm_check.py、snapshot.py、view.py
STEP/ STL/ 3MF/    輸出檔（save() 自動寫入）
checks/            要保留的量測檢查
tmp/               暫時的檢查腳本和截圖
```

## 設計流程（使用者要求的工作模式，每個新零件、新機構都要照做）

1. **先研究參考，再設計**：去看做好、被驗證過的 3D 機構（MakerWorld、Printables、Thingiverse、
   ServoCity 等商品、FTC/GM0、text-to-cad 範例）。不只看同一種機構，也看常用的機構類型。
2. **選最簡單的現成結構**，再依需求調整。不要從頭自己推導，然後遇到問題就加零件去補。
3. 設計原則：
   - 零件越少越好，能做成一模一樣的就共用
   - 滑動件要整段被包住、被導引（導引長度 ≥ 寬度的 2 倍）
   - 左右對稱；伺服馬達和固定點放在中心線上；手腕固定點放在重心下方
   - 接口大小配合對方的零件（例如舵盤座跟 Ø47 的 MATRIX 舵盤切齊，不多做）
   - 保護伺服馬達：機械限位、夾片緩衝
4. 先跟使用者分享**學習心得**和設計構想，得到同意才開始建模。
5. **不要逐步回報進度**（使用者覺得太麻煩），做完再一次說明結果；使用者問「怎麼了」時才簡短說明目前狀況。

## 建模規則

1. **單位 mm**，Z 朝上；選一個有意義的基準（例如底面在 Z=0、安裝孔中心在原點），寫在檔案開頭的說明裡。
2. **尺寸寫成檔案上方的大寫常數**（`WIDTH = 40.0`），名稱要看得懂用途。使用者說「加高 10mm」時，改常數就好。
3. **一個零件一個檔**，照 `src/l_bracket.py` 的格式：說明（用途、座標、假設）→ 參數 → 建模函式 → `save(part, "名稱")`。
   互相配合的機構（例如 `src/parallel_gripper.py`）可以放在同一個檔，共用參數，每個零件用「機構名_零件名」存檔，
   另存一份組合圖 STEP，並在 `checks/` 寫干涉檢查（取多個運動位置算零件兩兩重疊體積）。
   已知的零件規格（例如 MATRIX 舵盤 13-0002）寫在檔案說明裡並註明來源。
4. **螺絲孔用 `cadkit.fasteners`**，不要手寫孔徑：
   - 預設固定方式：`clearance_hole`（穿孔，可加沉頭）、`tap_hole`（螺絲直接鎖塑膠）、`nut_trap`（六角螺帽槽）。
   - `heat_set_hole` 熱熔螺母只有在使用者指定時才用（使用者不常用）。
   - 孔的工具形狀沿 -Z 往下挖、孔口在 Z=0；從側面挖就先 `Rot` 再 `Pos`。
5. **沒講清楚的尺寸**：寫成假設放在檔案說明裡，做完回報。只有會影響組裝配合的（孔距、要裝的零件尺寸）才先問。
   給了型號（例如 RPLIDAR A1、JGB37-520）就先查規格；查不到就說明並請使用者量。
6. **做不出某個特徵就老實說**，不要偷偷縮小、拿掉或換成近似形狀。
7. **為列印而設計**：牆厚 ≥ 1.6 mm（無支撐），懸空不超過 45°，孔徑 ≥ 2 mm，橋接 ≤ 10 mm。
   受力的零件（馬達座、保險桿）用加厚、圓角和補強，並考慮列印方向：層與層之間最弱，受力不要把層扯開。

### build123d 常見地雷

- 倒角、圓角要在實體**最後一次布林運算之後**才選邊；之前選的邊可能已經不屬於新實體。
- `Box` 預設置中、`Cylinder` 也置中；需要從某個面開始長時，明確設定 `align`。
- `.located()` 會覆蓋掉旋轉；要保留旋轉用 `.moved()` 或 `Pos(...) * Rot(...) * shape`。
- 穿孔的工具要比材料長，或剛好從表面開始，否則會留下薄膜。
- 布林運算之後檢查實體數（`len(part.solids())`）和 `part.is_valid`，結果不對就別交出去。

## 流程（每次建立或修改零件都要做）

```bash
python -X utf8 src/<零件>.py                           # 建模 + 存 STEP/STL/3MF，印出尺寸和體積
python -X utf8 tools/dfm_check.py STL/<零件>.stl         # 列印檢查（✅ / ❌ / ❓）
python -X utf8 tools/snapshot.py STL/<零件>.stl          # 四視圖存到 tmp/<零件>_views.png
```

1. **跑模型**，確認印出的外形尺寸和體積合理（可以心算估一下）。
2. **量測關鍵尺寸**：在 `tmp/` 寫一個小腳本，用 `import_step` 讀 STEP，量使用者指定的尺寸（孔徑、孔距、高度等）。
   範例見 `tmp/check_l_bracket.py`（圓形邊的 `radius`、`arc_center`）。之後還會用到的檢查放到 `checks/`。
3. **列印檢查**：有 ❌ 就修模型再跑；❓ 要在回報中說明。
4. **截圖自己看**：用 Read 打開 `tmp/<零件>_views.png`，確認形狀符合要求。這是自我檢查，不需要附給使用者。
5. **給使用者預覽**：
   ```bash
   python -X utf8 tools/view.py STEP/<零件>.step
   ```
   預覽器會在背景啟動，網址 http://127.0.0.1:3939/viewer（要在預覽器頁面打開後再送，畫面才會更新）。
   關閉：`python -X utf8 tools/view.py --stop`
   預覽器介面已用 `tools/patch_viewer_zh.py` 改成中文；更新 ocp_viewer 套件後要再執行一次。

## 回報給使用者

用繁體中文、白話說明：
- 產生的檔案（STEP/STL/3MF 路徑）
- 實際量過的尺寸，以及列印檢查結果（不通過和沒檢查到的項目要講）
- 用了哪些假設、需要使用者確認的地方
- 建議的列印方向和材料（受衝擊用 PETG，原型用 PLA）

## 更新 GitHub（每次修改後都要做）

專案已上傳到私人儲存庫 https://github.com/han0910603936-dot/amr-text-to-cad 。
使用者要求：**有改到任何檔案，做完並驗證後就 commit 並 push**，不用再問。

1. 在 `README.md` 的「更新紀錄」最上面，用日期加條列寫這次改了什麼。
2. `git add -A`（`tmp/` 已被忽略），commit 訊息用繁體中文：第一行是摘要，空一行後條列說明改了什麼、為什麼。
3. `git push`。push 失敗就告訴使用者，不要用 `--force`。
4. 驗證沒通過、還在修的半成品不要 push。

## 尚未完成 / 之後再做

- 還沒有印表機：`lib/cadkit/printer.py` 的 `printer`、`bed_mm` 未設定，孔徑也還沒校正。
  買了印表機之後，先做孔徑測試片校正 `hole_comp_mm` 和 `fasteners.METRIC`。
- URDF（ROS 2）是第二階段，使用者說之後再說；資料在 `研究筆記.md`。
