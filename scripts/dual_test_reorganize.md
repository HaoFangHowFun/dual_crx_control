我覺得現在讓雙臂運動的腳本太凌亂了，我想要整理一下。

我指的是C:\Users\Howard\dual_crx_control\scripts dual_test開頭的所有檔案
我覺得C:\Users\Howard\dual_crx_control\scripts\simple_motion.py，是我現在最喜歡的架構
1. 檔名不要有test，改成簡短但可以理解的檔案
2. prefix格式統一
3. 每個檔案header要付執行方式(主要是prefix)
4. 能精簡的地方就精簡，維持功能就好
5. record的方式也希望能統一參考C:\Users\Howard\dual_crx_control\scripts\simple_motion.py



你的方法:

以下是重構採用的方案。已完成六支 dual_test 腳本整理、英文 header、CLI 與錄製整合；使用方式與舊新參數對照見 ../docs/motion_scripts.md。

### 1. 先把六支 dual_test 腳本整理成四個入口

| 現有檔案 | 建議入口 | 處理方式 |
| --- | --- | --- |
| dual_test2_periodic.py | joint_sine.py | 保留單臂正弦運動，以 --arms left 或 right 選擇 |
| dual_test3_sychronized_motion.py | joint_sine.py | 合併雙臂有限時間正弦運動，以 --arms left right 選擇 |
| dual_test3_sychronized_motion_infinite.py | joint_sine.py | 共用同一實作，以 --continuous 啟用持續運動 |
| dual_test_5_cartesion_sychro_motion.py | cartesian_sine.py | 保留 Cartesian 單軸正弦運動 |
| dual_test_6_cartesian_circle_motion.py | cartesian_circle.py | 保留一般 TCP 圓形運動 |
| dual_test_7_cartesian_facing_circle_motion.py | facing_circle.py | 保留相向 TCP 圓形運動及其專屬預設值 |

simple_motion.py 保留原名稱及階梯目標功能，作為入口與錄製風格的參考。
它與正弦運動的軌跡不同，不直接合併。後續確認 test2_j1_periotic.py 的左臂正弦功能已由 joint_sine.py --arms left 涵蓋，因此也移除該舊入口。

一般圓形與相向圓形雖然共用 controller，仍保留兩個簡短入口：後者有不同的姿態準備、中心、半徑、週期、圈數及速度設定，避免合併後誤用預設值。

### 2. 統一命令參數與 prefix 的意義

這裡將 prefix 分成兩件事處理，避免把命令選項與 joint 名稱混在一起：

- 命令列選項統一採 simple_motion.py 的 argparse 形式：--rate、--duration、--output-dir 等長選項使用小寫及連字號。
- 機器人一律使用 left / right；joint prefix 為 left_ / right_，joint names 為 left_J1..J6、right_J1..J6。
- joint_sine.py 使用 --arms left、--arms right 或 --arms left right；Cartesian 系列目前固定雙臂，不新增未實作的單臂選項。
- 不加入任意 --prefix 字串：目前 interpolation node 僅接受固定左右臂 joint names，改名必須連同整套模型與 controller 一起處理。
- 新範例不再混用 robot1 / robot2；若保留舊 namespace 選項作過渡，集中透過 canonical_side() 轉換並提示新寫法。
- 單位寫在選項名稱：關節 --amplitude-deg、Cartesian --amplitude-m、圓形 --radius-m；時間及頻率在 help 中明列秒與 Hz。
- --rate 控制 motion sender；launch 的 input_rate_hz 仍控制插值時間，兩者需一致。不要重新把 rate 放回 JointTargetClient。
- --continuous 與有限執行選項互斥；圓形保留 --cycles，避免為了統一介面而失去以圈數結束的功能。

Cartesian 原本使用 --ros-args -p 的參數需建立舊名到新 CLI 名稱的對照；集中在入口轉換成 controller 設定，不在兩處維護不同預設值。保留 ROS remapping 等 ROS arguments，沿用 simple_motion.py 分離 ROS arguments 的方式。

### 3. scripts 保留入口，共用實作集中到 src

建議結構：

```text
scripts/
  simple_motion.py
  joint_sine.py
  cartesian_sine.py
  cartesian_circle.py
  facing_circle.py

src/dual_crx_control/motion/
  __init__.py
  joint_sine.py
  cartesian_controller.py
  circle_controller.py
  facing_circle.py
  startup_motion.py
```

把現有 scripts/motion/ 搬到 dual_crx_control.motion，更新所有引用，包括 search_facing_circle.py 等間接使用者，並移除 CMake 對頂層 motion package 的獨立安裝。
scripts 的入口負責參數解析與啟動，controller 負責運動流程，共用錄製維持在 motion_recording.py。
先共用三支 joint sine 的重複流程，不為所有 motion 類型建立大型通用基底類別。

### 4. 每個入口的 header 都放可複製的執行方式

每支腳本的 header（檔案開頭的 docstring）一律使用英文，包含用途、操作步驟、範例註解、參數說明與注意事項；可執行的命令、檔名、topic、prefix 與參數名稱保留原文。

Header 統一說明：用途、mock 啟動方式、另一個終端機的執行指令、左右臂 prefix、主要參數與單位、結束方式及輸出位置。

以下是重構後的執行範例（需重新 build 並 source 新的 install）：

```bash
# Terminal 1：啟動 mock 與 50 Hz 輸入的插值設定
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0

# Terminal 2：left_/right_ 雙臂同相正弦運動
ros2 run dual_crx_control joint_sine.py --arms left right --joint 1 --amplitude-deg 1 --period 4 --duration 20 --rate 50 --output-dir motion_recordings

# 單臂或持續模式使用相同入口
ros2 run dual_crx_control joint_sine.py --arms left --joint 1 --duration 20 --rate 50
ros2 run dual_crx_control joint_sine.py --arms left right --joint 1 --continuous --rate 50
```

每支 Cartesian 入口也提供自己的完整範例，明列軸／平面、振幅／半徑、週期與圈數，不只寫「同上」。

### 5. 錄製統一參考 JointRecording，但保留 Cartesian 的分析功能

- 統一 --output-dir，預設採相對路徑 motion_recordings，不寫死 /home/msc-crx/ws_fanuc。
- 每次執行建立獨立時間戳目錄，輸出 joints.csv、left_joints.png、right_joints.png；單臂執行只產生該臂圖。
- joints.csv 沿用 time_s、arm、source、J1_rad..J6_rad；source 統一為 target、interpolated、feedback。
- target 記錄實際送出的目標；interpolated 訂閱 /interpolation/joint_commands；feedback 訂閱左右臂 joint_states。三者分開，避免把目標當成實際 controller command。
- 基本 joints.csv 沿用本機 monotonic 接收／發送時間並共用同一起算點；如需保存訊息 header timestamp，另列欄位，避免混用時間基準。
- CSV 持續寫入、定期 flush，正常完成或 Ctrl+C 時統一收尾與出圖。補上單臂支援及重複關閉保護。
- 持續模式不能在結束時直接把全部 CSV 載入記憶體：圖表用分批讀取、降採樣或有界視窗，完整原始資料保存在磁碟。
- Cartesian 原本的 TCP 軌跡、誤差、JSON 設定與統計保留為額外輸出，逐步共用資料來源；不要直接用 JointRecording 取代後遺失 FK/TCP 功能。
- --latency-csv 的 generated、target_publish_return 等事件不等同標準 joints.csv。若維持完整功能，保留為可選事件輸出並共用 recorder，不直接刪除。
- 一般延遲分析繼續使用 analyze_latency.py，已支援 interpolated 與 feedback。

### 6. 以功能對照為基準精簡

合併前逐項列出各腳本的預設值、開始與結束行為。三支 joint sine 除執行時間外仍有差異，例如有限模式的回起點／hold，以及持續模式中斷後停止發布；合併時以明確選項或模式保留。

保留現有的啟動確認、feedback readiness／timeout、振幅 ramp、IK 失敗處理、步幅與速度限制、相向姿態與奇異點檢查。採用 simple_motion.py 的結構，不代表所有運動都改成先移到它的 INITIAL_JOINTS_DEG。

統一從 joint_config 取得 joint 順序，依 JointState.name 排序；左右臂 targets 一次交給 JointTargetClient.publish()。重構後確認沒有使用者，才移除舊的逐臂 publisher 相容層。

### 7. 分階段落地與驗證

1. 建立功能／參數對照，先改檔名、搬移共用 module、更新 CMake、imports 與 node names。
2. 合併 joint sine 三支實作，保留單臂、雙臂、有限與持續模式。
3. 統一 CLI、header 與錄製；同步處理 Cartesian 額外輸出與持續錄製的記憶體需求。
4. 用獨立 build/install 驗證新入口及 imports，避免舊 executable 殘留掩蓋錯誤；確認 --help 與 header 範例一致。
5. 在隔離的 mock 環境檢查各運動模式、正常結束與 Ctrl+C 的 CSV／圖表，並比對重構前後目標軌跡及預設值。實機行為需另外驗證，不將 mock 通過視為實機驗證。

scripts 與 tools 維持 LF，配合現有 .gitattributes 規則。回歸測試位於 tools/test_motion_refactor.py，ROS 軟體模擬驗證入口位於 tools/smoke_motion.py。
