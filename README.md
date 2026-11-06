# Hermes Isaac Sim 與 RViz2 plugins

兩個可分別啟用的 Hermes plugins，適用於已具備 Isaac Sim 6、ROS 2 Jazzy 與 Hermes 的 Linux 電腦。Agent 管理模擬、資產與導航目標；路徑及連續運動由 Nav2 執行。

- `isaac-sim`：13 個工具，提供啟停、USD 載入、具名場景／RV1 組合、截圖、occupancy map／URDF 匯出與 RV1 ROS adapter。
- `rviz2-navigation`：13 個工具，提供 profile、RViz、地圖／機器人顯示、初始定位核對、目標、規劃、導航與取消。
- 此 repository 只含程式、參數範本、測試與操作文件。場景、機器人模型、地圖、校準、錄影及機器設定由使用者另外提供。

## 安裝

先 clone 本 repository 並進入目錄；GitHub URL 由儲存庫建立後提供。已有相同系統環境的電腦執行一行：

```bash
bash install.sh
```

安裝器詢問 **Isaac Python／python.sh、公館 USD／USDZ、RV1 USD** 的實際路徑，驗證後保存到 `$HERMES_HOME/sim-plugins.json`；未設定 `HERMES_HOME` 時使用 `~/.hermes`。找不到 Hermes CLI 時也會詢問。不把個人路徑寫回 repository。

接著執行：

```bash
hermes plugins
```

以方向鍵選取 `isaac-sim`、`rviz2-navigation`，空白鍵勾選，依畫面提示確認／離開；新 session 生效。安裝器不自動啟用 plugin，也不改模型服務設定。

安裝方式是從 `HERMES_HOME/plugins/` 建立連結到目前 clone，**請保留 clone 位置**。同位置重跑可更新設定且保留手動啟用狀態；同名 plugin 若指向其他來源會停止並保留現況。需要並存測試時指定另一個 `--hermes-home`，不要刪除原安裝。

## 系統前提

- Ubuntu 24.04／ROS 2 Jazzy、RViz2、所需 Nav2 套件；Isaac Sim 6 相容的 NVIDIA GPU 與驅動。
- Hermes 已安裝；系統 Python 具備 PyYAML。橋接服務使用可用的 user systemd 與 GUI display／Xauthority。
- `install.sh` 安裝的是 plugins，不會安裝 Isaac、下載模型、改顯示驅動或設定模型憑證。缺少 ROS 相關套件時，另參閱 `scripts/bootstrap_ros2.sh --help`。
- RV1 的來源 USD 可能仍引用外部材質，模型擁有者須自行準備相依資產；本套件不保證離線解析所有材質。

## Agent／非互動安裝

先詢問使用者資產位置，取得回答後以參數帶入；下列是路徑格式示例，須換成該電腦上的實際檔案：

```bash
bash install.sh --non-interactive \
  --isaac-python /opt/isaac-sim/python.sh \
  --gongguan-usd /data/scenes/gongguan.usdz \
  --rv1-usd /data/robots/rv1.usd
```

也可使用 `bash install.sh --config /path/to/host.json --non-interactive`。安裝器會保存正規化設定至選定的 Hermes home；後續修改路徑可重跑相同指令。設定檔權限為 0600，缺少必要檔案時不猜測或套用其他主機的路徑。

若使用自訂 Hermes home，後續選單、session 與啟動腳本也須使用同一個 `HERMES_HOME`；例如先 `export HERMES_HOME=/path/to/profile`。若 Hermes CLI 不在 PATH，使用安裝時提供的完整執行檔路徑。

完整可選參數以 `bash install.sh --help` 為準，包括 ROS setup／Python、display、DDS domain、兩個 port、外部 state／export 目錄及 Hermes 路徑。

## 開始使用

安裝與啟用完成後，在 clone 目錄執行：

```bash
bash scripts/start_bridge.sh
bash scripts/start_rviz2_bridge.sh
```

不同設定檔可加 `--config /path/to/host.json`；Hermes session 同時設定 `HERMES_SIM_CONFIG=/path/to/host.json`。啟動器使用獨立 user service，拒絕重用配置不一致的既有服務，不自行重啟模擬。

讓 Agent 閱讀 [完整操作流程](docs/agent-runbook.md) 與兩份 skills：

- [Isaac skill](codes/isaac-sim/skills/isaac-sim/SKILL.md)
- [RViz2／Nav2 skill](codes/rviz2-navigation/skills/rviz2-navigation/SKILL.md)

[設定欄位](docs/configuration.md)｜[跨機器驗證](docs/portability-validation.md)｜[開發清單](TODO.md)

## 範圍

目前一次管理一個 robot profile；RV1 採模擬真值定位與靜態 occupancy，未提供 AMCL／動態避障／多車協調。具名公館配方保留該場景的 mesh prim 與 RV1 出生位置；換成其他地圖時應使用一般 USD／地圖工具及對應機器人配置。變更檔案位置不等於讓任意場景都符合公館配方。

程式沿用 manifests 的 MIT 授權；使用者模型、地圖及第三方材質不受本 repository 的程式授權涵蓋。
