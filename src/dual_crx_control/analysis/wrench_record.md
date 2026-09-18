# Wrench Recording and Live Plot

## Scope

新增雙臂 wrench 記錄與即時圖，沿用現有 ROS 與 Matplotlib。第一版已實作；使用方式與參數另見 `scripts/analysis/record_wrench.py` header 和 `--help`。

- 每臂記錄量測 wrench：`Fx, Fy, Fz`（N）與 `Tx, Ty, Tz`（N·m）；雙臂共 12 通道，不是 12 個關節力矩。
- 同筆資料附上該臂當下可取得的 EEF pose：以 joint feedback + FK 算出的 TCP 位置與姿態，不使用 command target，也不宣稱是外部感測器直接量測的位置。
- 純訂閱與記錄，不發布控制命令，可和鍵盤控制、teleop 或預排軌跡並行。
- 第一版不做力控制、濾波、歸零、重力補償或 wrench 座標轉換，保留來源資料。

## Existing Interfaces and Open Questions

已確認 FANUC driver 提供力／力矩介面；本機 driver 原始碼有 `ft_sensor/force.x` 至 `ft_sensor/torque.z`，也已安裝標準 `force_torque_sensor_broadcaster`。目前本專案自訂 launch 只啟用 joint state broadcaster 與 position controller，沒有啟用力／力矩 broadcaster；載入 driver hardware 並不等於已發布 wrench topic。

採用標準 `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster`，不用 `fanuc_gpio_controller` 取得 wrench，也不必為 recorder 引入 FANUC 自訂 message。標準輸出為 `geometry_msgs/msg/WrenchStamped`。[ROS 官方文件](https://control.ros.org/jazzy/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html)

FANUC 官方說明此資料是 flange 的 resultant force/torque；CRX 預設使用 embedded sensor。文件列出的支援條件為控制器軟體 V9.40P/85 以上及 ROS driver v2.0.0，仍需確認真機版本。官方 launch 會啟用 broadcaster，但我們的自訂 launch 不會自動繼承該行為。[FANUC 官方文件](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/controller_usage.html#fanuc-controllers-fanuc-force-sensor-broadcaster)

實作前需由正在運行的 sensor/driver 確認：

- 左右臂實際 topic 名稱、message type、發布頻率與 QoS。
- `header.stamp` 是否為有效且共用時基的量測時間。
- wrench 的座標系、力矩參考點、單位，以及 driver 是否已歸零或補償。

第一版支援 `geometry_msgs/msg/WrenchStamped`。依本專案 namespace，獨立 broadcaster 啟用後預期使用 `/left/force_torque_sensor_broadcaster/wrench` 與 `/right/force_torque_sensor_broadcaster/wrench`，作為 recorder 的預設值；仍可透過 `left_wrench_topic`、`right_wrench_topic` 覆寫。這些是規劃啟用後的 topic，尚未在真機確認。

位置來源沿用 `/left/joint_states`、`/right/joint_states` 與 `/robot_description`，使用 `CRXKinematics(description, 'left_tcp' / 'right_tcp')`，輸出 pose 以 `world` 為座標系。

## File Layout

- `src/dual_crx_control/analysis/wrench_recording.py`：資料訂閱、FK、配對、CSV 寫入與即時 figure。先集中在一個模組，以小型 class/function 區分責任，不新增框架。
- `scripts/analysis/record_wrench.py`：薄入口；啟動指令、topic 參數、figure 說明與停止方式寫在 header，標題使用英文。
- `tools/test_wrench_recording.py`：資料配對、異常處理與記錄測試。
- `launch/dual_arm.launch.py`：完整雙臂 bringup；以 `wrench:=true` 按需啟用左右臂標準 force/torque broadcaster，預設不啟用。
- `config/wrench_controllers_left.yaml`、`config/wrench_controllers_right.yaml`：獨立 broadcaster 設定，分別設定 flange frame，不改既有基本 controller YAML。
- `tools/smoke_wrench.py`：隔離 domain 179 的 ros2_control mock + 合成 wrench 整合測試，`--gui` 額外測即時視窗。
- 更新 `CMakeLists.txt` 安裝入口與註冊測試；`package.xml` 明確宣告 `geometry_msgs` 與 `force_torque_sensor_broadcaster` 依賴。

不修改既有 motion recording CSV schema，避免影響目前分析工具。

## Optional Wrench Launch

平常使用 `dual_arm.launch.py` 或 `dual_arm_teleop.launch.py`，預設不載入 wrench broadcaster。需要 wrench 時，直接將同一個完整 bringup 設為 `wrench:=true`；不再使用獨立 `dual_arm_with_wrench.launch.py`，也不重複啟動 hardware 或 position controller。

- 與完整 bringup 平行啟動兩個 `controller_manager/spawner`，在短暫延遲後（等待兩臂 joint controller 完成資源設定）透過獨立 parameter file 載入、configure、activate broadcaster；`sensor_name: ft_sensor` 沿用 driver 介面。
- 左右臂透過各自 parameter file 設定 `frame_id` 為 `left_fanuc_flange`、`right_fanuc_flange`，對應本機 FANUC driver 的命名；不標成 TCP 或 world。
- 正常 Ctrl+C 停止此 launch 時，launch 會停止兩個 spawner；不停止同一 launch 管理的 robot control nodes。若 controller manager 已有同名 broadcaster，spawner 會回報已存在，不能和另一份 bringup 重複啟動。
- controller manager 未啟動、interface 缺失或單臂啟動失敗時由 spawner 明確回報；兩臂是否都 active 需在 recorder/測試中確認。
- `ft_sensor` 名稱不加左右 prefix：兩臂有不同 controller manager，沿用 driver 提供的 interface 名稱即可。

啟用 topic 不代表 recorder 已開始存檔，兩者分開操作。`wrench:=false` 是預設值；`wrench:=true` 才載入 broadcaster。已在純 mock 驗證完整 bringup lifecycle；未連真機。SIGKILL、controller manager 中斷或服務逾時不能保證清理，會要求人工檢查。不可同時啟動另一份相同的 bringup。

## Pose and Wrench Pairing

以每一筆收到的 wrench 為記錄事件，左右臂各自配對，不強迫雙臂具有相同頻率或時間戳。

第一版採可追溯的接收時間配對：

1. joint feedback 到達時驗證 joint names、有限值與關節限位，以 FK 更新該臂最新 pose cache，保存原始 ROS stamp 與本機 monotonic 接收時間。
2. wrench 到達時擷取該臂最近一筆已收到的有效 pose；用兩者 monotonic 接收時間的差計算 `pose_age_s`。
3. 建議 `pose_timeout_s=0.10`（可設定）。沒有有效 pose 或超時時，仍存 wrench，但 pose 欄位留空並標示 `pose_valid=false`；不把過期 pose 假裝成當下位置。
4. 原始 wrench 與 joint ROS stamp 都保存，供離線檢查傳輸延遲與不同步。接收時間接近不代表量測時間同步，不混用 ROS time 與 monotonic time 做減法。

這是第一版的近似關聯，不是嚴格時間同步。若後續要做動態接觸分析，再於確認共同量測時基後，增加時間緩衝與以 wrench stamp 為準的 joint interpolation。暫不加入 message_filters 或雙臂四路同步。

## Coordinate Semantics

pose 明確標示 `pose_frame=world`、`tcp_frame=left_tcp/right_tcp`；wrench 保留原始 `header.frame_id`。兩者放在同一列，不代表 wrench 已轉到 world 或 TCP。

空 frame_id 顯示為 unknown 並警告，不自行填成 TCP。metadata 另外記錄來源已知的力矩參考點與補償狀態，未知就註明 unknown。若未來需要把 sensor wrench 搬到 TCP，除了旋轉還涉及力臂造成的力矩變化，不能只改 frame 名稱或只旋轉六個數值。

## Output Format

每次啟動建立唯一時間戳目錄，例如 `wrench_recordings/<timestamp>/`：

- `wrench.csv`：逐筆保存兩臂資料，以 `arm` 區分。
- `metadata.json`：topic、message type、QoS、單位、配對策略、參數、模型資訊與接收／無效／丟棄筆數。
- `wrench.png`：正常結束時保存最後一個顯示視窗；完整歷史仍以 CSV 為準。

CSV 欄位：

```text
time_s, arm, wrench_stamp_s, joint_stamp_s, pose_age_s, pose_valid,
wrench_valid, wrench_frame, pose_frame, tcp_frame,
x_m, y_m, z_m, qx, qy, qz, qw,
fx_N, fy_N, fz_N, tx_Nm, ty_Nm, tz_Nm
```

`time_s` 是 wrench 接收時間相對錄製開始的 monotonic 秒數。零值／缺失的來源 stamp 不偽造，metadata 說明其有效性。含 NaN/Inf 的 wrench 記錄為無效樣本，figure 斷線且計數，不替換成零。

## Live Figure

單一視窗使用 **6 rows × 2 columns**：左欄左臂、右欄右臂；由上而下為 Fx、Fy、Fz、Tx、Ty、Tz，每個通道各自一張 subplot，避免不同量級擠在同一張圖。

- X 軸共用接收後經過秒數，預設顯示最近 `window_s=10` 秒。
- 圖片預設 `plot_rate_hz=10`，接收／CSV 寫入不跟著降到 10 Hz。
- 每張 subplot 標明通道與 N／N·m；各臂標出 wrench frame、最新 pose、pose age 與來源接收狀態。
- plot buffer 限時且限筆數，例如每臂最多 10,000 筆；超量只裁切或降採樣顯示，不影響 CSV 的記錄策略。
- 超過 `wrench_timeout_s=0.5` 未收到新資料時顯示 STALE，曲線不延伸成假量測；恢復資料時不跨越已知中斷區間連線。
- 沒資料時顯示 WAITING，不畫零值假資料。兩臂各自更新，一邊離線不阻塞另一邊。

使用 Matplotlib GUI 主執行緒定時更新既有 line objects，不反覆重建 figure；ROS executor 在背景接收資料，GUI 透過短鎖取得 buffer snapshot。這是供人閱讀的 live figure，不是硬即時或安全監控系統。

## Recording and Shutdown

ROS callback 只做驗證、快取與入列；獨立 writer 使用 bounded queue 批次寫 CSV，約每秒 flush。queue 滿時不阻塞 ROS callback：丟棄新入列資料並明確警告、計數；磁碟錯誤則報錯並停止錄製，不顯示仍在成功記錄。

CSV 保存所有成功收到、成功入列的樣本，不宣稱能避免 DDS 或磁碟造成的資料遺失。metadata 記錄實際筆數與可觀察到的丟棄數。

Ctrl+C 或關閉 figure：停止接收、排空 writer queue、flush/close CSV，保存 metadata 與最後視窗圖片後退出。這只停止記錄，不停止機器人。

Linux / WSL 需要可用 GUI backend（例如 WSLg）；提供 `--no-plot` 做無 GUI 錄製。GUI 不可用時清楚報錯並提示此選項，不默默忽略使用者要求的 live figure。

## Usage

建置並 source ROS workspace、啟動雙臂後使用：

```bash
# Terminal 1: existing robot bringup (dual_arm or dual_arm_teleop).
# Terminal 2: enable optional wrench topics only when needed.
ros2 launch dual_crx_control dual_arm.launch.py mock:=false wrench:=true

# Terminal 3: record and plot using the default topics.
ros2 run dual_crx_control record_wrench.py \
  --window-s 10 --plot-rate-hz 10
```

`dual_arm.launch.py` 預設不啟用 wrench；使用 `wrench:=true` 後，GenericSystem 的 sensor interface 可用於 topic/lifecycle 測試，但不是模擬接觸力。既有 `dual_mock_robot.py` 不提供 controller manager 或 wrench；合成波形測試使用 `tools/smoke_wrench.py`，與 driver topic 分開。

只存檔不顯示視窗：`ros2 run dual_crx_control record_wrench.py --no-plot`。每次輸出包含 `wrench.csv`、`metadata.json` 與最後視窗 `wrench.png`。

## Verification

已完成：離線錄製／繪圖回歸測試、ros2_control 雙臂 mock 的完整 bringup、broadcaster topic/frame、合成 wrench + FK pose CSV，以及 TkAgg 即時視窗、Ctrl+C 與關閉視窗的存檔清理。未連真機，未驗證真實力感測精度或補償設定。

在 source 最新 build 的環境、repo 根目錄執行：

```bash
python3 tools/smoke_wrench.py --gui
# Headless equivalent:
python3 tools/smoke_wrench.py
```

測試固定使用 localhost domain 179，啟動 `mock:=true` bringup，另發布合成 wrench 至 `/test/left_wrench` 與 `/test/right_wrench`；自動關閉所有測試 subprocess，結果寫入印出的 `/tmp/dual-crx-wrench-smoke-*` 目錄。不要同時執行多份測試或在 domain 179 連真機。

- 合成不同頻率的左右 wrench 與 joint feedback，驗證逐臂配對、時間欄位、12 個通道順序與單位。
- 驗證缺失／過期 joint feedback、亂序或零 stamp、NaN/Inf、未知 frame 與單臂中斷的標示；無資料時不生成假零值。
- 驗證 FK 的 TCP 與 world 定義沿用既有模型，CSV 記錄 measured-feedback pose 而不是 command pose。
- 驗證 figure 降採樣不降低 CSV 記錄率、長時間 buffer 有界、queue overflow 有計數。
- 在隔離 ROS domain 使用軟體 mock + 合成 wrench 驗證完整流程，再人工確認 GUI 12 張圖與 frame/單位標示。
- 驗證 Ctrl+C、關閉視窗、`--no-plot`、磁碟錯誤與輸出檔正常收尾。不在此階段連真機或修改控制流程。
