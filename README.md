# 我的 3D 列印建模工具

用講的就能產生 3D 列印零件。跟 Claude 描述你要的零件，它會寫程式建模、檢查能不能印，再讓你在瀏覽器裡預覽。

## 怎麼用

直接在這個資料夾跟 Claude 說，例如：

> 做一個 RPLIDAR A1 的支架，把光達墊高 80mm，底部用 4 顆 M3 鎖在底盤上，中間留穿線孔。

Claude 會照 `.claude/skills/cad/SKILL.md` 的流程做：建模 → 量尺寸 → 列印檢查 → 截圖自我檢查 → 打開預覽。

改尺寸時講「哪裡、改多少」就好，例如「立板加高 10mm」「孔太緊，放大 0.3mm」。
怎麼寫提示詞可以參考 `研究筆記.md`。

## 資料夾

| 資料夾 | 內容 |
|---|---|
| `src/` | 每個零件的建模程式（一個零件一個檔） |
| `STEP/` | STEP 檔：可以匯入其他 CAD 軟體 |
| `STL/`、`3MF/` | 給切片軟體的列印檔 |
| `lib/cadkit/` | 共用工具：存檔、螺絲孔、列印限制 |
| `tools/` | 列印檢查、截圖、預覽器 |
| `tmp/` | 暫存的檢查檔和截圖，可以隨時刪 |

## 自己執行（不透過 Claude 也可以）

在 PowerShell 裡，從這個資料夾執行：

```bash
C:\Users\jeff2\.cad3d\venv\Scripts\python.exe -X utf8 src\l_bracket.py
```
```bash
C:\Users\jeff2\.cad3d\venv\Scripts\python.exe -X utf8 tools\dfm_check.py STL\l_bracket.stl
```
```bash
C:\Users\jeff2\.cad3d\venv\Scripts\python.exe -X utf8 tools\view.py STEP\l_bracket.step
```

預覽網址：http://127.0.0.1:3939/viewer

預覽器介面是用 `tools\patch_viewer_zh.py` 改成中文的。如果更新了預覽器套件，中文會被蓋掉，再執行一次這個工具就好（加 `--restore` 可以還原成英文）。

## 環境

- Python 3.13 環境放在 `C:\Users\jeff2\.cad3d\`（OneDrive 外面，避免同步上千個檔案）。
- 套件版本記錄在 `requirements.txt`。
- 還沒有印表機：列印限制是一般 FDM 的預設值，買了印表機後要校正 `lib/cadkit/printer.py`。

## 在另一台電腦重建環境（Windows）

在 PowerShell 裡，從專案資料夾依序執行。路徑請換成你自己的。

1. 安裝 uv：
   ```bash
   winget install --id astral-sh.uv --exact
   ```
2. 建立 Python 3.13 環境（放在 OneDrive 外面），並安裝套件：
   ```bash
   uv venv $HOME\.cad3d\venv --python 3.13 --managed-python
   ```
   ```bash
   uv pip install --python $HOME\.cad3d\venv\Scripts\python.exe -r requirements.txt
   ```
   如果是在 Claude 桌面版裡執行而出現路徑錯誤，先設定 `$env:UV_PYTHON_INSTALL_DIR="$HOME\.cad3d\python"` 和 `$env:UV_CACHE_DIR="$HOME\.cad3d\cache"`。
3. 讓環境找得到 `lib/`：在 `$HOME\.cad3d\venv\Lib\site-packages\` 建立 `cad3d_lib.pth`，內容是這個專案 `lib` 資料夾的完整路徑（UTF-8 編碼）。
4. 預覽器改中文：
   ```bash
   $HOME\.cad3d\venv\Scripts\python.exe -X utf8 tools\patch_viewer_zh.py
   ```
5. 測試：執行 `src\l_bracket.py`，`STEP\`、`STL\`、`3MF\` 裡出現 `l_bracket` 就成功了。

## 參考來源

- 做法參考 [earthtojake/text-to-cad](https://github.com/earthtojake/text-to-cad)（MIT 授權）：建模規則、檢查流程，以及 3D 列印檢查的量測方式。
- 建模：[build123d](https://github.com/gumyr/build123d)、[bd_warehouse](https://github.com/gumyr/bd_warehouse)
- 預覽器：[OCP CAD Viewer](https://github.com/bernhard-42/vscode-ocp-cad-viewer)（ocp_viewer）
- 網格分析：[trimesh](https://github.com/mikedh/trimesh)
- 研究過程和數值來源見 `研究筆記.md`。

## 更新紀錄

### 2026-10-18
- 建立第一階段工具：Python 3.13 環境、`cadkit` 建模函式庫（存檔、螺絲孔、列印限制）
- 列印檢查 `tools/dfm_check.py`、四視圖截圖 `tools/snapshot.py`、瀏覽器預覽 `tools/view.py`
- 預覽器介面中文化 `tools/patch_viewer_zh.py`
- 專屬建模技能 `.claude/skills/cad/SKILL.md`
- 測試零件：L 型支架 `src/l_bracket.py`
