# Isaac Sim — Hermes Agent 操作手冊

本 skill 管理 **isaac-sim 的 13 個工具**。RViz2／Nav2 使用另一個 plugin：`rviz2-navigation`，開始導航前另讀其 `rviz2-navigation:rviz2-navigation` skill。Isaac plugin 負責模擬、資產匯出與現有 RV1 ROS adapter；agent 不直接發布速度或操作輪組。

## 環境與狀態

部署資訊由 `HERMES_SIM_CONFIG` 指定的 JSON 記錄；未指定時使用 `${HERMES_HOME:-~/.hermes}/sim-plugins.json`。所有資產路徑都是執行 bridge 的主機路徑。先讀此檔核對 Isaac／ROS Python、ROS setup、display、domain、ports、state_root 與 export_root。不得回顯 token。

若 `gongguan_usd`／`rv1_usd` 尚未設定，先詢問使用者檔案位於哪裡；確認後由套件根目錄執行 `bash install.sh --gongguan-usd PATH --rv1-usd PATH` 記錄。不要猜測其他電腦的路徑。`isaac_sim_list_scripts` 回報 logical asset key、設定路徑與 configured 狀態；缺少資產時 named script 回傳 CONFIGURATION_ERROR，應補設定而非改程式。

第一次操作先 `isaac_sim_status({})`；核對 session、stage、timeline、fresh state 及 pending operation。重用已開啟的 instance，不重送 pending 操作。`accepted`／`pending` 不表示完成；根據 operation ID 或 export job ID 輪詢結果。Lifecycle `wait_seconds` 為 0–60。

## 全部工具

| 工具 | 作用與主要參數 |
| --- | --- |
| `isaac_sim_open` | 啟動或重用受管理 Isaac；`display_number` 省略時使用部署設定（通用預設 1）、可指定既有 X display，亦支援 `headless`、`wait_seconds`。不自動播放。 |
| `isaac_sim_load_usd` | 以 `path` 載入 USD／USDA／USDC／USDZ；須先停止模擬，會切換 stage。 |
| `isaac_sim_play` | 開始播放；查 physics time／steps 持續推進。 |
| `isaac_sim_stop` | 停止 timeline 並重設時間，保留應用程式。 |
| `isaac_sim_close` | 關閉本 plugin 擁有的 instance；必要時 `force` 只處理核對過的 owned process。 |
| `isaac_sim_status` | 讀程序、場景、timeline、operation、顯示及新鮮物理量測。 |
| `isaac_sim_list_scripts` | 查固定註冊的場景流程及前置條件。 |
| `isaac_sim_run_script` | 以 `script_name` 執行註冊流程；既有 instance 且 timeline stopped，不接受任意 Python。 |
| `isaac_sim_capture` | 以 `label` 擷取受管理 GUI 視窗及 state sidecar；不是 RViz 或 sensor render。 |
| `isaac_sim_export_occupancy_map` | 獨立 Isaac worker 匯出 2D occupancy PNG／YAML；解析度、高度帶、範圍與品質參數可調。 |
| `isaac_sim_export_robot_urdf` | 通用 USD 機器人匯出；`source_path`、選填 `root_prim_path`、`robot_name`、`base_frame`、`output_dir`。 |
| `isaac_sim_export_status` | 以 `job_id` 查匯出結果、worker exit、來源未變及產物。 |
| `isaac_sim_prepare_rv1_ros2` | 在 stopped 且 RV1 已存在時準備 ROS adapter／控制腳本；開始時 disarmed。這項 adapter 仍是 RV1 專用。 |

舊 `isaac_sim_export_rv1_urdf` 已改名。舊 RViz／Nav2 工具不再屬於此 plugin，不能以舊名稱重試。參見[跨 plugin 操作手冊](../../../../README.md)。

## 公館與 RV1 載入

若現有導航正在執行，先以 `rviz2_cancel_navigation` 取消，保持 Isaac playing 並確認 `state.ros2.armed=false`、`state.ros2.rest_verified=true` 後再 stop／換場景。每一步等待完成：

```text
isaac_sim_open({"wait_seconds":60})
isaac_sim_run_script({"script_name":"load_gongguan_xinhai","wait_seconds":60})
isaac_sim_run_script({"script_name":"add_rv1_to_gongguan","wait_seconds":60})
isaac_sim_prepare_rv1_ros2({"config":{"localization_mode":"sim_truth","map_from_stage_xyz_yaw":[0,0,0,0],"max_linear_mps":1.0,"max_yaw_rps":0.5,"command_timeout_seconds":3.0},"wait_seconds":60})
isaac_sim_play({"wait_seconds":60})
```

`load_rv1` 會載入獨立 RV1 測試 stage；公館內插入車輛使用 `add_rv1_to_gongguan`，不要混用。已正確載入時重用現有 stage，不重複插入。底層 `robots/rv1_tune_control.py` 由 runtime 匯入處理 Nav2 Twist，不啟動來源鍵盤控制器。

播放後確認 fresh state、有效量測與 ROS 回授。公館插入流程自動啟用物理位置相機跟拍，`state.scene_setup.camera_follow.status="tracking"` 表示追隨更新；錄影時仍需檢查實際畫面。跟拍不控制車體。

接續 RViz 工作使用另一個 plugin：`rviz2_configure_robot({"profile_name":"rv1"})` → `rviz2_load_map` → `rviz2_load_robot` → `rviz2_open` → `rviz2_navigation_start`，詳見其 skill。

## 資產匯出

獨立 GPU 匯出要求受管理 Isaac 已關閉，且一次只有一個 export job。先完成導航取消／實測停妥，再關閉 Isaac；只關 RViz 不符合匯出條件。輸出位於外部 `export_root`，可傳其下新的 `output_dir`；不能覆寫原資產或舊結果。

完整公館碰撞場景範例：

```json
{"source_path":"<設定檔 gongguan_usd 的絕對路徑>","bounds_mode":"full_scene","resolution_m":0.2,"seed_stage_xyz_m":[160,-155,0.8],"z_min_m":0.3,"z_max_m":1.8,"geometry_mode":"physx","max_cells":4000000,"max_voxels":40000000}
```

`full_scene` 不同時傳入 `bounds_stage_xy_m`。Gaussian Volume 未網格化區域無碰撞形狀，不能冒稱已轉圖；完整 XY 圖幅也不保證地面支撐、坡面、多樓層或動態障礙可通行。unknown 不可改成 free。調參後重新核對 bounds、品質及 footprint。

通用 URDF 範例使用 `isaac_sim_export_robot_urdf`：

```json
{"source_path":"<設定檔 rv1_usd 的絕對路徑>","robot_name":"rv1","base_frame":"base_link"}
```

其他機器人使用其來源、名稱及 root。省略 `robot_name` 時輸出前綴為 `robot`；多 articulation 場景必須指定只包含一個 articulation 的 `root_prim_path`。目前僅支援 metersPerUnit=1、Z-up、連通無環的 fixed／revolute／prismatic 結構；不把任意 USD 宣稱為可匯出機器人。輸出為 `<robot_name>.raw.urdf`、`<robot_name>.urdf`、meshes 與 metadata。衍生 URDF 用於顯示，不是模擬控制器。

兩類匯出皆輪詢 `isaac_sim_export_status`，直到 `success=true`、`pending=false`、`process_running=false`、`worker_exit_code=0`，並查 `source_unchanged`、metadata 與輸出 hash。使用回傳 `outputs.map_yaml`／`outputs.urdf`，不要假設固定 job 路徑。完整參數及限制見[資產手冊](../../../../README.md)。

## 收尾與維護

RV1 導航後以 `rviz2_cancel_navigation` 撤銷控制，保持 timeline playing，查新鮮 `isaac_sim_status` 的 `state.ros2.rest_verified=true`；它要求實際速度與 yaw rate 均小於 0.05，持續至少 0.25 模擬秒。取消成功或命令為零不能代替停妥量測。完成後按需 `rviz2_navigation_stop` → `isaac_sim_stop`／`isaac_sim_close`。

stop／reset 後須重新 prepare、play、核對初始位置，不沿用舊 goal。更新 bridge／runtime 前先停妥並關閉 owned Isaac 與匯出 worker；避免 systemd restart 直接終止工作中子程序。完成更新後 reload Hermes 並開新 session，核對此 plugin **13 個工具**。
