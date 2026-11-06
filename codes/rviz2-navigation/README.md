# rviz2-navigation plugin

獨立提供 13 個 RViz2／Nav2 工具。Agent 設定 robot profile、載入地圖與 URDF、核對初始定位、選擇目標；Nav2 負責規劃路徑與速度。ROS-only profile 不需要 Isaac Sim。

從整個套件根目錄執行 `bash install.sh`，再於 `hermes plugins` 手動啟用。保留 `codes/`、根目錄 `hermes_sim_config.py` 與 `configs/` 佈局；不需要安裝／啟用 Isaac plugin 即可註冊本 plugin。完整安裝見[套件手冊](../../README.md)，工具流程見[Agent skill](skills/rviz2-navigation/SKILL.md)。

部署設定來自 `HERMES_SIM_CONFIG` 或 `${HERMES_HOME:-~/.hermes}/sim-plugins.json`。ROS setup／Python、domain、RMW、localhost discovery、display／Xauthority、ports 與外部 state_root 均可調。bridge 提供 `--config`、`--state-dir`、`--host`、`--port`、`--ros-domain-id`、`--ros-python`、`--ros-setup`、`--display-number`、`--xauthority`、`--rmw-implementation`、`--ros-localhost-only`。

Client 先讀部署 profile，再讀選用 `bridge-client.json`，最後套用 `HERMES_RVIZ2_URL`／`HERMES_RVIZ2_TOKEN_FILE`。Isaac adapter 同樣讀部署設定與選用 `isaac-client.json`，支援 `HERMES_ISAAC_URL`／`HERMES_ISAAC_TOKEN_FILE`。兩個 bridge token 與 state 分離，分別位於 state_root/rviz2 和 state_root/isaac。

RV1 Nav2 presets 隨完整套件根目錄 configs 配送；`profile.params_file` 相對路徑固定以套件根目錄解析，不使用程序工作目錄；外部自訂 YAML 建議傳絕對路徑。其他 robot 透過 profile.params_file 帶入使用者選定 YAML。地圖與 URDF 由工具參數傳入，或先詢問使用者保存其結果路徑後帶入。禁止把機器私有路徑、token、地圖與模型放進公開 plugin Git。導航需持續有效 TF／odom／clock，取消命令後仍須查實測停妥。
