# HISTORY

## 0.1.0-dev — 2026-11-06

- S0：從既有導航專案建立獨立公開程式套件，模型、地圖與實驗資料另外存放；原部署保留。
- S1：新增單指令互動／非互動安裝、私有設定檔、衝突保護與預設不啟用的 Hermes plugin 安裝；實測選單及26工具註冊。
- S2：移除主機資產／憑證路徑，設定解譯器、ROS、display、DDS、port與輸出位置；產生臨時RV1 wrapper，保留既有26工具及Nav2控制分工。
- S3：435個不重複plugin案例、21個安裝器案例、19個腳本案例通過；真實registry完成USD轉檔、RV1模擬、RViz／Nav2三公尺目標與停妥；未在第二台實體主機實測。
- Graphify：49個來源檔AST索引，來源與完整性紀錄見docs/GRAPHIFY.md；外部LLM tokens為0。
- 此為公開儲存庫建立前的開發版，未上傳GitHub。完整限制與驗證見docs/portability-validation.md。
