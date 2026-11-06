# isaac-sim plugin

獨立提供 13 個 Isaac lifecycle、場景、視窗截圖、2D 地圖／通用 URDF 匯出及 RV1 ROS adapter 工具。RViz／Nav2 操作由相鄰獨立 `rviz2-navigation` plugin 提供，agent 只選目標，沒有直接駕駛工具。

從整個套件根目錄執行 `bash install.sh`，再於 `hermes plugins` 手動啟用。保留 `codes/`、根目錄 `hermes_sim_config.py` 與 `configs/` 佈局；不要只複製單一 plugin 子資料夾。完整安裝見[套件手冊](../../README.md)，工具流程見[Agent skill](skills/isaac-sim/SKILL.md)。

主機設定來自 `HERMES_SIM_CONFIG` 或 `${HERMES_HOME:-~/.hermes}/sim-plugins.json`。安裝程序詢問 Isaac Python、Gongguan USD／USDZ、RV1 USD 等路徑後保存到外部 JSON；不得把某部電腦的路徑提交 Git。既有設定可用 `bash install.sh --gongguan-usd PATH --rv1-usd PATH` 更新。具體命令參數以 `bash install.sh --help` 為準。

bridge 支援 `--config`、`--isaac-python`、`--state-dir`、`--host`、`--port`、`--display`、`--xauthority`、`--screenshot-python`、`--ros-domain-id`、`--rmw-implementation`、`--ros-localhost-only`。Client 先讀部署 profile，再讀選用 `bridge-client.json`，最後由 `HERMES_ISAAC_URL`／`HERMES_ISAAC_TOKEN_FILE` 覆寫連線設定。原始 token 只存於外部 state_root/isaac/token。

Named scripts 每次解析使用者保存的資產路徑；`load_rv1` 在 session state 中建立引用設定 RV1 的新測試 stage。模板、模型與場景原檔不修改。公館場景仍需符合既有碰撞 mesh 結構；跨機器不代表自動支援不同幾何／車輛。匯出只能寫到設定 export_root 下的新目錄，runtime／token／截圖不寫入公開套件。
