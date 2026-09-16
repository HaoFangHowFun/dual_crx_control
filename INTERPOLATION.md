# 獨立 joint interpolation

```text
motion script（先完成 IK）／teleop bridge
  → /interpolation/joint_targets（joint sequence）
  → joint_interpolation（linear 或 cubic，固定 500 Hz）
  → /left/forward_position_controller/commands
  → /right/forward_position_controller/commands
```

**輸入與輸出都是 joint sequence。interpolation 不接收 Cartesian pose、不做 FK／IK，也不讀取 URDF 或檢查運動限制。**

## 啟動

建置並 source workspace 後，在第一個終端啟動 robot bringup：

```bash
# mock 與實機共用同一個入口；預設 mock=true
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false method:=linear
# cubic 可替換為 method:=cubic
# 實機設定為 mock:=false（本次沒有執行實機測試）
```

另一個終端啟動所需 motion，例如：

```bash
ros2 run dual_crx_control dual_test_5_cartesion_sychro_motion.py
```

`dual_arm.launch.py` 統一原本 joint／Cartesian、mock／實機的四個入口。mock 使用 `mock_components/GenericSystem`，實機使用 FANUC hardware plugin，兩者均啟動 `joint_state_broadcaster` 與 `forward_position_controller`。控制器設定在 `config/dual_arm_controllers.yaml`。保留左右 driver 私有 TF、校正後雙臂全域 TF、joint state 合併、IP 與 RViz 選項。

teleoperation 的入口維持不變，輸入固定 **20 Hz**、輸出固定 **500 Hz**：

```bash
ros2 launch dual_crx_control teleop_joint.launch.py mock:=true rviz:=false method:=cubic
```

兩個 launch 均直接使用 `interpolation = Node(...)`，不再有 `interpolation.launch.py`。readonly launch 不啟動 interpolation。若另有 robot bringup，單獨啟動節點：

```bash
ros2 run dual_crx_control interpolation_node --ros-args -p input_rate_hz:=50.0 -p method:=linear
```

bringup 本身不發送移動命令。motion script 不會另開 interpolation process；client 等待 target subscriber 再傳送資料。應只啟動一份 interpolation，節點不額外檢查重複 publisher 或管理控制權。

## 頻率與介面

| 設定 | 預設 | 說明 |
| --- | --- | --- |
| `input_rate_hz` | 50.0 | 有限且在 `(0, 500]`；目標到達時間為接收後一個輸入週期 |
| `method` | linear | linear／cubic；需重啟節點變更 |
| 輸出 | 固定 500 Hz | steady timer 每 0.002 秒取樣 |

motion 的 `rate` 與 interpolation 的 `input_rate_hz` 應設成相同；client 不再使用 service 協商或核對遠端參數。teleop bridge 每 50 ms 最多傳送一筆最新目標，外部來源應以 20 Hz 發送。feedback 頻率另設，預設 100 Hz。

- `/interpolation/joint_targets`：`sensor_msgs/JointState`，每筆為完整單臂 6 軸或雙臂 12 軸。名稱是 `left_J1`～`left_J6`、`right_J1`～`right_J6`；位置單位 rad，依名稱重排。header 不參與輸入排序或控制權判斷。
- `/interpolation/joint_commands`：`JointState`，記錄實際發布的 joint commands，供 motion recording 觀測。header stamp 是節點本機 **CLOCK_MONOTONIC**，不是 ROS wall-clock；frame_id 不使用。
- controller command topics：`Float64MultiArray`，每臂依 J1～J6 排序。

雙臂使用相同取樣時間，通過資料檢查後依序發布。兩個 ROS topic 的傳輸不具原子性。遠端 target 可直接傳送，無須共享時鐘；若將 command report 時間與本機觀測時間相減，則須在同一主機上量測。

## 持有與恢復

第一段由有效 joint feedback 初始化，尚無 feedback 時保留最新待處理目標，不假設全零位置。後續段從最後已發布的 joint position 銜接，feedback 不再影響插值，也沒有 freshness watchdog。

沒有新輸入時，完成最後一段，然後**持續以 500 Hz 發布最後 joint position**。新目標到達後直接繼續。motion script 結束或中止只會停止供應新目標；interpolation 仍完成已收到的最後一段並保持端點，直到節點停止。未曾收到目標的手臂不發布命令；已啟用的手臂即使後來只更新另一臂，仍持續保持。

沒有 session、start／finish／abort service、控制權 token、斷線 watchdog 或重複 publisher 檢查。`JointTargetClient` 只是共用傳送工具，負責組成完整 joint target 與確認有 subscriber。

## 僅保留資料檢查

檢查 joint 名稱、數量、陣列長度、重複／缺失 joints、NaN／Inf、頻率及插值時間。非法目標會被拒絕，原本有效段繼續執行。

不檢查 URDF 關節上下限、速度、加速度、每步變化或 spline 極值，不因超限改成 linear。既有上游 motion 的檢查維持；不同入口不保證具有相同運動檢查。

cubic 使用近期已發布樣本及下一目標建立 natural `CubicSpline`，只有收到新目標才重建。歷史不足時先用 linear；不外插超過最後端點。cubic 可能 overshoot，也不保證每次重建間速度連續；資料合法不代表曲線已符合運動限制。

## 程式與測試

- `src/dual_crx_control/interpolation.py`：純 joint 插值數學。
- `interpolation_node.py`／`interpolation_client.py`：ROS 輸出節點／target 傳送工具。
- `scripts/motion/`：startup、Cartesian、circle、facing-circle 上游規劃與控制。
- `src/dual_crx_control/robot_description.py`：launch 共用 driver description 與 mock 初始角度。

Cartesian／circle CSV 區分 `target`、`command`、`feedback`；command 來自 interpolation 實際輸出。

```bash
colcon test --packages-select dual_crx_control --ctest-args \
  -R '^(cartesian|interpolation|interpolation_node|motion_routes|circle|facing_circle|latency|teleop_bridge|dual_test4_transport)$' --output-on-failure
colcon test-result --verbose
python3 tests/measure_interpolation_mock.py  # 8 組，每組約 30 秒
python3 tests/verify_cartesian_mock.py
```

驗證使用隔離 localhost ROS domains。輸出 500 Hz 是排程目標，實測頻率與 jitter 見 `INTERPOLATION_TEST_RESULTS.md`，不是硬即時保證。

**interpolation 的進、出及內部插值一律都是 joint sequence，不包含任何 Cartesian 資料或運算。**
