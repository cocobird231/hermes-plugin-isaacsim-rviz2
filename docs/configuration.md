# 跨機器設定

安裝器將使用者回答寫到 `HERMES_HOME/sim-plugins.json`。選擇順序為工具／CLI 明確參數、對應支援的環境覆寫、設定檔、一般預設；`HERMES_SIM_CONFIG` 可選另一份 JSON。安裝器的 `--config` 是輸入檔，結果仍保存到選定 Hermes home。相對 JSON 路徑以該 JSON 所在目錄為基準。

| 欄位 | 用途／一般預設 |
| --- | --- |
| `isaac_python` | 使用者提供的 Isaac Python／python.sh；不預設環境名稱 |
| `gongguan_usd`、`rv1_usd` | 使用者提供的現存 USD 資產；缺少時要求設定 |
| `ros_setup`、`ros_python` | ROS 環境與解譯器；預設 Jazzy setup 與系統 Python |
| `screenshot_python` | 提供 GI／GDK 的 Python，預設系統 Python |
| `display_number`、`xauthority` | 預設 display 1 與使用者 Xauthority；明確 display 工具參數優先 |
| `ros_domain_id`、`ros_localhost_only` | 預設 37／true；並存測試需選不同 domain |
| `rmw_implementation` | 預設 rmw_fastrtps_cpp；其他 RMW 必須在 ROS 與 Isaac 環境具備相容函式庫 |
| `isaac_port`、`rviz2_port` | 預設 8768／8769；兩者不得相同 |
| `state_root` | 預設 Hermes home 下的 sim-plugins-state；兩服務分別使用 isaac／rviz2 子目錄 |
| `export_root` | 預設 state_root/exports；USD 匯出只允許其下的輸出目錄 |
| `hermes_cli`、`hermes_source` | CLI 可自動從 PATH／使用者安裝位置尋找；可選的工具證據 exporter 需要提供 Hermes 原始碼位置 |
| `isaac_ros_lib_dir` | 可選的 Isaac 隨附 ROS library 目錄；未提供時按 Isaac 安裝結構探測 |
| `schema_version` | 目前為 1 |

設定檔與 state／export 目錄不應位於公開 checkout；模型／地圖由使用者控制，程式不改寫來源資產。RV1 測試場景會在當次 session 內產生引用已設定模型的 wrapper。公館插入操作只改 session layer。

兩個 plugin client 保留 `HERMES_ISAAC_URL`／`HERMES_ISAAC_TOKEN_FILE` 與 `HERMES_RVIZ2_URL`／`HERMES_RVIZ2_TOKEN_FILE` 覆寫；一般同機部署不需設定。環境變數若殘留舊值，會覆蓋設定檔，切換前須檢查。token 檔由服務產生，不把 token 值寫進 JSON 或 Git。

變更 port、Python、ROS 或 display 後，既有服務不會自動套用新設定。先完成／取消目前導航、量測停妥、關閉此服務擁有的模擬，再停止啟動器列出的 user service，重新啟動。不要停止其他來源或其他設定的服務。
