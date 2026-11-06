# Agent 操作流程

先讀兩個 plugin skills。本文路徑是使用者本機檔案，不預設 SSH 主機名稱、帳號或 Workspace 目錄。

1. **取得設定**：若尚未安裝，詢問使用者 Isaac Python、公館 USD／USDZ 與 RV1 USD 的實際路徑；以 `install.sh` 互動回答或明確參數保存。請使用者在 `hermes plugins` 勾選兩個 plugins。缺少資料時不要猜測舊主機路徑。
2. **啟動服務**：使用目前 clone 的 `scripts/start_bridge.sh` 與 `start_rviz2_bridge.sh`；自訂設定須透過 `--config` 與 Hermes 的 `HERMES_SIM_CONFIG` 指向同一檔案。先查兩個 status，避免重複啟動／提交。
3. **準備 ROS 資產**：詢問既有地圖 YAML、URDF 位置；沒有時，使用 `isaac_sim_export_occupancy_map`／`isaac_sim_export_robot_urdf` 提供來源絕對路徑。匯出需受管 Isaac 已關閉；輸出目錄在 export_root 內。等待 `isaac_sim_export_status` 成功，保存本次實際產物路徑，不沿用別台電腦的 URDF mesh URI。
4. **開啟模擬**：`isaac_sim_open` → `isaac_sim_run_script(load_gongguan_xinhai)` → `isaac_sim_run_script(add_rv1_to_gongguan)`。名稱括號為說明，實際參數使用 `script_name`。需要獨立 RV1 地面場景時改用 `load_rv1`。
5. **接上控制器**：timeline stopped 時呼叫 `isaac_sim_prepare_rv1_ros2`，再 `isaac_sim_play`。控制器隨 plugin 提供，不另跑鍵盤控制，也不直接發布 Twist。
6. **載入 RViz／Nav2**：`rviz2_configure_robot` 選 `rv1`，再 `rviz2_load_map`、`rviz2_load_robot`、`rviz2_open`、`rviz2_navigation_start`；路徑使用步驟 3 的本機產物。查 status 至 TF、odom、clock 與 navigation ready。
7. **目標與導航**：從當次 status 讀位置，以 `rviz2_set_initial_pose` 核對；選自由且具地面支撐的目標，呼叫 `rviz2_set_goal` → `rviz2_plan_path` → `rviz2_navigate`。沿用當次 goal_id；所有中途路徑與速度由 Nav2 決定。
8. **驗收與收尾**：查同一 attempt 的成功狀態與真實位姿誤差。呼叫 `rviz2_cancel_navigation` 後，保持模擬 playing，確認兩端解除控制及新鮮停妥量測，再 `rviz2_navigation_stop` → `isaac_sim_stop` → `isaac_sim_close`。

`pending` 或 HTTP timeout 不代表失敗；先查當次 request／operation／job 狀態，不重複提交。Nav2 succeeded 不等於已停妥，仍須量測。

預設 rv1 profile 保留低速配置。最高速設定需明確選 `rv1_max_speed`，且 Isaac adapter 也設定 `max_linear_mps=2.5`、`max_yaw_rps=0.5`；由使用者決定是否適合該場景。

如需截圖／工具證據，使用 `scripts/` 中對應 helper 的 `--help`，輸出指定到公開 repository 外。腳本 `plugin_call.py` 是 handler 測試入口，不是自然語言 Hermes 模型執行證據。
