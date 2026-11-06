# 跨機器部署 TODO

依據：2026-11-06 使用者要求，將既有兩個 Hermes plugins 改為可設定部署，公開程式與私有資料分離。此為已授權實作；原始部署與歷史證據保留。

- [x] S0：設定與套件邊界
  - Test criteria：公開套件不含主機身分、私有資產、token 或執行證據；原來源及正式 plugin 連結不變。
  - [x] S0.01：建立公開 `hermes-sim-plugins` 與私有 `private-data`。
  - [x] S0.02：盤點主機路徑、資產參照、程序設定與文件；記錄可設定欄位。
- [x] S1：外部設定與安裝
  - Test criteria：同環境任意使用者／含空白路徑可安裝；互動詢問資產並持久保存，非互動缺參數明確失敗；重複安裝不改使用者啟用狀態，不覆寫其他 plugin。
  - Unit test：設定型別／範圍、缺少來源、配置優先序、外部檔案權限、衝突與重裝。
  - Integration test：指定測試主機 的隔離 HERMES_HOME 真實 discovery、registry 與 `hermes plugins` 手動啟用介面。
  - [x] S1.01：集中設定、命令列輸入及初次詢問。
  - [x] S1.02：一行安裝兩個 plugin，保持預設未啟用。
- [x] S2：核心與腳本移植
  - Test criteria：26 工具保留；具名場景／RV1 使用設定路徑；輸出落在外部可寫目錄；ROS、Python、display、DDS、port 可設定；不加入直接駕駛 API。
  - Unit test：既有兩個 plugin suites 與新增設定／路徑測試。
  - Integration test：不同 clone 路徑、替代資產檔名、獨立橋接服務／DDS；真實 Isaac／ROS 的載入、模擬與受控結束。
  - [x] S2.01：資產與 token 配置、臨時 RV1 composition。
  - [x] S2.02：服務啟動、錄影／監測／Hermes helpers 可設定。
  - [x] S2.03：Agent skills 與跨機器操作文件。
- [x] S3：交付驗證
  - Test criteria：指定測試主機 完成測試；原檔與正式安裝不變；本機與主機公開檔案一致；公開目錄可獨立初始化 Git，且不引用私有同層資料作為執行必要條件。
  - [x] S3.01：lint／compile／unit／真實整合驗證並保存私有證據。
  - [x] S3.02：整理公開文件、私有資產清單及限制。
  - [x] S3.03：Graphify 建置與來源定位；同步、Git 提交。
  - 遠端 GitHub 建立／push：由使用者後續提供儲存庫，本次不發布。

驗證紀錄：見 [跨機器部署驗證](docs/portability-validation.md) 與 [Graphify](docs/GRAPHIFY.md)。完整證據存於另外的私有資料夾。GitHub 尚未建立或發布。
