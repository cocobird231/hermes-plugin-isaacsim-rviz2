# RViz2／Nav2 — Hermes Agent 操作手冊

本 skill 對應獨立 `rviz2-navigation` plugin 的 **13 個工具**。它管理 RViz、地圖／機器人顯示、單一 robot profile 與 Nav2 actions。Agent 選擇目標並請求導航；速度與路徑由 Nav2 產生。Isaac lifecycle／USD 轉檔屬於另一個 `isaac-sim` plugin，需使用時另讀 `isaac-sim:isaac-sim`。

## 開始前

部署資訊由 `HERMES_SIM_CONFIG` 指定的 JSON 記錄；未指定時使用 `${HERMES_HOME:-~/.hermes}/sim-plugins.json`。所有資產路徑都是執行 bridge 的主機路徑。先讀此檔核對 Isaac／ROS Python、ROS setup、display、domain、ports、state_root 與 export_root。不得回顯 token。

先 `rviz2_navigation_status({})`，核對 active profile、程序、map revision、pending goal 及定位狀態；不要盲目重啟或重送目標。使用 `rviz2_configure_robot({"profile_name":"rv1"})` 明確選擇內建 RV1，或設定其他機器人的 profile。一次只管理一個 profile；更換 profile／地圖／模型後須重新載入並核對初始位置，不沿用舊 goal／plan。

## 全部工具

| 工具 | 作用 |
| --- | --- |
| `rviz2_configure_robot` | 選擇 `profile_name="rv1"`，或傳入自訂 `profile`；配置 robot frames／topics、Nav2 YAML、時間來源、控制 backend 與 footprint。 |
| `rviz2_open` | 開啟或重用 owned RViz，`display_number` 省略時使用部署設定（通用預設 1）；不開 Isaac、不導航。 |
| `rviz2_close` | 只關 owned RViz 視窗。 |
| `rviz2_load_map` | `map_yaml_path` 交由 owned map_server 載入並發布給 RViz／Nav2，不載入 USD。 |
| `rviz2_load_robot` | `urdf_path` 交由 robot_state_publisher 顯示；不建立 Isaac 車輛。 |
| `rviz2_navigation_start` | 啟動 profile 的 Nav2 節點與生命週期；不送出目標、不行車。 |
| `rviz2_navigation_status` | 查 profile、程序、TF／odom／clock、初始定位、目標、plan、action result 與 control relay 狀態。 |
| `rviz2_set_initial_pose` | 用 `x_m`、`y_m`、`yaw_rad` 核對目前 map-frame TF；不瞬移、不發布 AMCL 初始估計。 |
| `rviz2_set_goal` | 驗證 map-frame 目標與 footprint，回傳 `goal_id` 並顯示目標；不移動。 |
| `rviz2_plan_path` | `goal_id` 呼叫 Nav2 `ComputePathToPose`，發布預覽；不發布行車命令。 |
| `rviz2_navigate` | `goal_id` 呼叫 Nav2 `NavigateToPose`；Nav2 自行規劃／重規劃及生成 Twist。 |
| `rviz2_cancel_navigation` | 撤銷 private Nav2→robot relay、清零並取消 action；結果需輪詢，靜止另由量測驗證。 |
| `rviz2_navigation_stop` | 取消並停止本 plugin 擁有的 ROS／RViz 程序；不關閉 Isaac。 |

不使用舊 `isaac_sim_rviz2_*`、`isaac_sim_rv1_*` 導航工具。沒有直接方向、速度、持續時間或輪組控制工具。即使可讀到既有其他 plugin，也不要用它介入同一 robot 的 `/cmd_vel`。

## Profile 與通用範圍

內建 `rv1` 使用 Isaac RV1 adapter、`sim_truth`、模擬時間與已驗證 RV1 footprint。其他機器人使用自訂 profile 的 `control_backend="ros2"`；此 backend 不需要 Isaac bridge。必填欄位是 `robot_id`、`frames`、`topics`、`use_sim_time`、`params_file`、`localization_mode`、`control_backend`。完整例子見[操作手冊](../../../../README.md)。

`frames` 必須提供不同的 `map`／`odom`／`base`；`topics` 提供不同的絕對名稱 `map`／`odom`／`cmd_vel`／`joint_states`／`robot_description`／`clock`。`use_sim_time=false` 時不要求 `/clock` 推進。`localization_mode="tf_verified"` 核對既有定位來源；`sim_truth` 要求模擬時間。兩種模式都需要有效 map→base TF 及新鮮 odom，均不啟動 AMCL。

`params_file` 必須是該機器人適用的現存 Nav2 YAML；相對路徑以安裝套件根目錄解析，外部檔案請傳絕對路徑；global/local costmap footprint 與 padding 必須一致，也可用 profile `footprint`／`footprint_padding_m` 同時覆寫。不可沿用 RV1 尺寸代表另一台車，或縮小 footprint 以繞過障礙檢查。`limits` 限制 relay 允許的速度、命令逾時與回授逾時；外部控制器仍須有自身斷訊停止機制。這不是硬體急停保證。

目前單一 active profile 與 root Nav2 actions，不支援同時多車。URDF 顯示與 generic API 不代表所有機器人動力學已實測。

## RV1 完整流程

先由 Isaac plugin 完成場景載入、RV1 插入、`isaac_sim_prepare_rv1_ros2` 及播放，確認有效回授。接續：

```text
rviz2_configure_robot({"profile_name":"rv1"})
rviz2_load_map({"map_yaml_path":"<完整公館 map.yaml 的 host 路徑>"})
rviz2_load_robot({"urdf_path":"<匯出結果 outputs.urdf>"})
rviz2_open({})
rviz2_navigation_start({})
rviz2_navigation_status({})
```

`<…>` 必須換成實際工具回傳路徑。等 `navigation_ready=true`，確認 map/model、TF、odom 及必要的 clock 新鮮。由 `state.pose` 取得實際 x／y／yaw，呼叫 `rviz2_set_initial_pose`；seed 與 map YAML origin 都不是機器人初始位置。

在已核對 ground support 與完整 footprint 的 free 區域選目標：

```text
rviz2_set_goal({"x_m":185,"y_m":-157,"yaw_rad":-0.07983})
rviz2_plan_path({"goal_id":"<當次 set_goal ID>"})
rviz2_navigate({"goal_id":"<同一當次 goal ID>"})
```

上列座標僅為既有公館測試案例，換地圖／profile 後不可直接沿用。使用者只要求規劃時，不呼叫 navigate。Nav2 `NavigateToPose` 會自行重規劃，預覽不是它必須逐點跟隨的命令。地圖只有靜態障礙，完整圖幅不是全路網安全通行證明。

## 導航結果與停止

導航期間以 `rviz2_navigation_status` 查同一 goal／attempt 的 action feedback／result。RV1 再用 `isaac_sim_status` 比對真實 PhysX 位移與 yaw。接受 goal、看見路徑或回傳 pending 都不是到達；到達需 `NavigateToPose` succeeded 及當次量測誤差。

收尾呼叫 `rviz2_cancel_navigation`；持續查 `state.control.relay_armed=false`、`rest_verified=true` 與新鮮 odom。`rest_verified` 需要量測速度持續符合靜止條件至少 0.25 秒，且 state 會標示來源。RV1 保持 Isaac playing，另核對 `state.ros2.armed=false`、`rest_verified=true`；沒有新鮮有效回授就不能宣稱停妥。確認停妥後再 `rviz2_navigation_stop`，依需求另停止／關閉 Isaac。

模擬 clock 回跳會撤銷舊 TF、初始定位與目標。重新播放後等 fresh TF／odom／clock，核對初始位置，再設新 goal。服務失聯時不得持續重送導航；控制器本身的 watchdog 應處理斷訊。

## 證據與更新

記錄兩個 plugin 的工具名稱、實際參數、profile、map revision、job／goal／attempt ID、Nav2 action 結果與真實量測。不要把歷史 RV1 報告當成新 profile 驗收。`isaac_sim_capture` 只拍 Isaac；RViz 畫面需另有 desktop 截圖／錄影。

本 plugin 只管理自己的 state-dir、token、worker 與程序。更新前取消導航並確認停妥，再停止 owned ROS／RViz 程序；reload Hermes 後開新 session 核對 **13 個 rviz2_* 工具**。與 Isaac 合用時共 **26 個工具**。

## RV1 設計最高速設定

使用 `rviz2_configure_robot({"profile_name":"rv1_max_speed"})` 明確選擇前進上限 2.5 m/s、轉向上限 0.5 rad/s；既有 `rv1` 保留 Nav2 0.4 m/s 設定。切換前先停止舊導航。Isaac 端須在模擬停止時呼叫 `isaac_sim_prepare_rv1_ros2({"config":{"max_linear_mps":2.5,"max_yaw_rps":0.5}})`，再播放模擬；兩邊限制一致後才開始導航。最高速 profile 使用加速度 2、減速度 3 m/s²、16×16 m 局部地圖及 6 m 前方路徑範圍，保留原 footprint、到達容差與 watchdog。2.5 m/s 是速度指令上限，彎道及接近目標仍由 Nav2 自行減速；須依實測速度判斷是否達到最高速度。
