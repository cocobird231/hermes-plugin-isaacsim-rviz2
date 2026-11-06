# 程式結構索引

本索引用於定位公開套件的 Python 模組、函式與靜態關係，不能代替實際安裝、Isaac Sim 或 Nav2 驗收。

## 來源與範圍

- 建立日期：2026-11-06，使用 Graphify 0.9.61，在本機執行 AST 抽取與圖分析。
- 索引來源提交：`86a8ed07a012ee64aeff4bf19ff92cded9c31ab9`，分支 `coco-codex/portable-hermes-plugins`。索引最初建立於工作樹，提交後已逐一核對49份來源與此提交相同；後續純文件更新不改變此AST快照。
- 來源清單 SHA-256：`f8b6e0a03d02bfd52f22a0357a217e04cf30bcb6735e98c7e01d15358c6cca30`。
- 實際抽取 **48 個生產 Python 檔案**，另記錄 `configs/rv1_max_speed_profile.json` 的來源 hash；該 JSON 未產生符號節點。
- Python 範圍包括根目錄設定模組、兩個 plugin 的程式與可重用 scripts；排除所有 `tests/`、文件語意、YAML、shell、相容性 client JSON，以及任何私有資產、實驗與執行結果。
- 49 份來源在抽取前後 hash 相同，且與開發主機的對應檔案逐一比對一致。
- 外部 LLM：**input 0／output 0 tokens**；沒有呼叫語意抽取服務。

來源清單存於忽略的 `graphify-out/source-manifest.json`。摘要 hash 的計算為該檔 `files` 陣列經 `json.dumps(files, sort_keys=True, separators=(",", ":"))` 編碼為 UTF-8 後取 SHA-256；每列包含相對路徑、檔案 hash、bytes 與行數。這使來源版本可在尚未提交時明確辨識。

## 索引結果

| 項目 | 結果 |
| --- | --- |
| 節點／有向圖邊／社群 | 450／1,086／17 |
| 本機節點來源定位 | 446 筆均有有效檔案與行號 |
| 圖邊來源定位 | 1,086 筆均有有效檔案與行號 |
| 外部或基底類別 stub | 4：`Exception`、`ValueError`、`Path`、`Node`，沒有本機定義行號 |
| 最終圖缺少端點／懸空端點 | 0／0 |
| 關係標記 | `EXTRACTED` 890、`INFERRED` 196 |
| Python 檔案符號覆蓋 | 48／48 |

圖為有向 `DiGraph`，但 AST 關係仍是靜態抽取與啟發式推導，**不是實際執行軌跡**。196 條 `INFERRED` 沒有使用外部 LLM，也不代表經執行驗證。

原始抽取保存 1,389 條邊，其中有 291 條懸空端點與 23 組同端點多種 relation。建圖會解析部分名稱、略過無法連結的端點，並合併相同端點關係，因此最終的 1,086 條邊不是完整的 import／relation 清單。`raw-extraction.json`、`health-raw.json` 與 `health-final.json` 保留差異；不能把最終端點完整解讀成原始抽取沒有缺漏。

四條 self-loop 都來自 extractor 將 `super()` 呼叫連回目前方法，不能當作程式自行遞迴：

- `codes/isaac-sim/capture.py:26`
- `codes/isaac-sim/runtime.py:57`、`:64`
- `codes/rviz2-navigation/rviz_navigation/ros_worker.py:196`

## 常用入口

| 問題 | 直接查閱 |
| --- | --- |
| 設定檔位置、參數與路徑驗證 | [hermes_sim_config.py](../hermes_sim_config.py) |
| 安裝、保留既有 plugin、預設停用 | [install_plugins.py](../scripts/install_plugins.py) |
| 使用者服務啟動、所有權與 readiness | [launch_bridges.py](../scripts/launch_bridges.py) |
| 公館與 RV1 場景設定 | [scene_scripts.py](../codes/isaac-sim/scene_scripts.py)、[gongguan.py](../codes/isaac-sim/rv1/gongguan.py) |
| Isaac 程序與場景操作 | [manager.py](../codes/isaac-sim/manager.py)、[runtime.py](../codes/isaac-sim/runtime.py) |
| RViz／Nav2 程序、profile 與運動規劃 | [supervisor.py](../codes/rviz2-navigation/rviz_navigation/supervisor.py)、[ros_worker.py](../codes/rviz2-navigation/rviz_navigation/ros_worker.py) |

已查詢 `load_config launch_plan wait_ready` 與 `asset_path run_script insert_rv1`，分別涵蓋 174 與 282 個 BFS 節點。初始 3,500-token 顯示預算會截斷；最終改為 20,000 後沒有截斷。查詢輸出只用來定位，再直接核對來源；stub 與不完整的 import 解析不能取代程式內容。

若本機已有索引，可在套件根目錄執行：

```bash
graphify query "load_config launch_plan wait_ready" --budget 20000
graphify query "asset_path run_script insert_rv1" --budget 20000
```

## 保存方式與更新

`graphify-out/` 已被 Git 忽略，僅保留於建立索引的本機，沒有放入公開發佈內容，也沒有同步整份索引至開發主機。此說明文件會同步。新 clone 沒有圖檔時，請先建立索引再查詢。

主要產物：`graph.html`、`graph.json`、`GRAPH_REPORT.md`、`raw-extraction.json`、`source-manifest.json`、`provenance.json`、`source-evidence.json`、`remote-source-verification.json`、`health-raw.json`、`health-final.json`、`cost.json` 及兩份查詢輸出。

重建須使用相同的顯式檔案清單：根目錄、`codes/`、`scripts/` 的 Python，排除所有 tests／cache／graphify-out，再加入 `configs/*.json`。以 Graphify 的 `extract(paths, root=package_root)`、`build_from_json(..., directed=True)`、`cluster()`、`to_json()` 建圖，保留原始抽取並核對來源 hash、節點／邊行號。不要直接對整個私有工作區做語意掃描。生產程式或設定修改後，舊圖應視為快照並重建；僅改文件不影響此 AST 來源快照。
