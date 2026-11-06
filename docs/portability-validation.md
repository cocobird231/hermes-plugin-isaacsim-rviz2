# 跨機器部署驗證

日期：2026-11-06。所有安裝、程式及模擬測試在指定 Linux／Isaac 測試主機執行。本次以不同 clone／Hermes home、含空白的資產路徑、獨立 state、替代 ports 與 DDS domain 驗證搬移；尚未在第二台實體電腦重跑。

## 完成範圍

- 公開程式與私有資產分開；安裝器向使用者取得 Isaac Python、場景與 RV1 位置，保存外部 JSON。
- 一行安裝、重複安裝、外部設定、含空白路徑與衝突保留均已驗證。原正式 plugin 連結與模型設定不變。
- 真實 `hermes plugins` 選單顯示兩個未勾選 plugin；在隔離 profile 啟用後，真實 registry 載入 26 個工具。
- 原 80 個來源程式／設定／腳本的 SHA-256 未變；主機手動開啟的 Isaac 保留。新版只從新的公開套件測試。

## 測試結果

| 類別 | 結果 |
| --- | --- |
| Isaac plugin：系統 Python | 368 passed；6 個 USD 案例改在 Isaac Python 執行 |
| Isaac USD suite：Isaac Python | 15 passed，其中 9 個與前列重複 |
| RViz2 plugin：ROS Jazzy Python | 61 passed |
| Plugin 不重複案例 | 435 passed |
| 安裝器／設定 | 21 passed |
| 啟動與輔助腳本 | 19 passed |
| 靜態檢查 | Pyflakes、Python 編譯、Bash 語法通過 |
| 埠號衝突 | 真實 bridge 啟動回報失敗，既有測試 socket 保留，不誤報 ready |

設定解譯器測試包含實際 `sys.prefix` 核對，避免解析 Python symlink 後失去 virtualenv。服務啟動成功須核對 owned PID、listen socket 及認證狀態回覆；只有 systemd 接受請求不算 ready。

## 真實整合測試

透過 **Hermes registry 的真實 handler** 呼叫，未使用語言模型 session：

1. 將原 RV1 USD 複製為含空白的新檔名，原始與副本 hash 相同；設定新的模型與場景路徑。
2. 以獨立 Isaac worker 匯出 RV1 URDF（13 links／12 joints）與簡化地面 occupancy map；輸出均在公開 repository 外。
3. 開啟受管 Isaac，透過具名場景工具載入地面 fixture、插入真實 RV1、載入既有 ROS 控制器並播放。
4. 載入本次產生的地圖與 URDF，啟動 RViz／Nav2、核對起點，設定距起點 3 m 的目標；路徑與速度全部由 Nav2 產生。
5. Nav2 SUCCEEDED，實測平面位移 **2.706 m**、終點誤差 **0.295 m**；取消後取得三筆兩端解除控制且停妥的回授。
6. 截圖保存，停止導航；另完成 `load_rv1` 動態 wrapper 載入及播放／停止，最後要求關閉受管 Isaac。

上述導航使用簡化靜態地面測試 fixture，**不是完整公館路網的新驗收**。另已直接開啟原公館 USDZ，核對 Z-up、meter 與原碰撞 mesh；未在本次重做全圖長距離錄影。

受管 Isaac 已回報 closed／close succeeded 且程序退出；兩個隔離 bridge 已停止，原手動 Isaac 仍在。Kit 關閉期間出現既有 menu teardown 訊息，未使用強制終止。本機／測試主機共1,460個交付檔案已逐一核對SHA-256一致。截圖有其他視窗遮擋，未將它當成完整車體可視驗收；導航結果依真實位姿與速度回授判定。

## 適用限制

- 安裝器不安裝系統套件或模型；已安裝 Hermes 第一次使用新的 isolated home 時，Hermes 本身可能準備自己的依賴環境。
- 匯出 worker 沿用既有 `isolated_worker_process_exit` 策略；結果已驗證且 exit 0，不代表 Isaac Kit 原生 teardown 問題已修復。
- RV1 原始模型可能有外部材質依賴；URDF mesh URI 為該次主機輸出位置，搬移產物後須重新匯出／適配。
- 本套件保留單一 robot profile、模擬真值定位與靜態障礙範圍；未增加 AMCL、動態避障或實車驗收。
- 本次尚未建立或發布 GitHub repository；使用者可將此目錄單獨初始化為公開 repository。完整原始證據與資產存於另外的 private-data，不隨公開套件發布。
