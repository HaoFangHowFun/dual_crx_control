我想把interpolation變成一個rosnode，所有要控制motion的不管是mock還是real robot test 都要叫醒他，並且用他產生要給forward_position_control的joint motion 的 topic

1. interpolation 輸入就是joint data以及數入的頻率，輸出的頻率幫我hardcoded，現在是500Hz
2. 並且在interpolation裡面可以選擇要現在的linear_interpolation，還是更進階的spline，可以參考這邊的cubic spline C:\Users\Howard\retargeting_crx\src\retargeting_ros\nodes\robot_real_high_freq.py
3. 除了launch以外，記得要檢查script裡面的有沒有額外再呼叫一次interpolation，script裡面
4. 我覺得src 裡面start motion 和 facing circle有點太多餘了，可以把他並回去script的motion嗎
5. 記得在模擬環境完成測試


write your methood here:

已依 0915 討論完成實作，以下為目前方法（取代原先含 session／watchdog／運動限制的方案）：

1. motion script 先完成規劃與 IK，使用 `JointTargetClient` 傳送完整 6／12 軸 joint targets。client 只負責資料傳送，不建立第二個 interpolation process，也不管理控制權。
2. 獨立 `joint_interpolation` node 接收 joint sequence 與 `input_rate_hz` 設定；輸出 hardcoded 500 Hz。teleoperation launch 固定使用 20 Hz 輸入，feedback 頻率另設。
3. 保留 linear，另提供 natural cubic spline；近期已發布樣本加新目標構成曲線，資料不足時先用 linear，不向最後端點之後外插。每筆目標到達時間為本機接收時間加一個輸入週期，採 monotonic／steady 排程。
4. feedback 只用於第一段起點。之後由最後已發布 joint position 銜接。沒有新目標就完成最後一段，持續以 500 Hz 發布端點；新資料到達直接繼續。
5. interpolation 只檢查 joint 名稱、數量、長度、有限值、頻率與時間。不讀 URDF，不檢查運動限制，也不做超限退回 linear、session、watchdog 或重複 publisher 管理。上游 motion 既有檢查保留。
6. 四個 bringup 合併成 `dual_arm.launch.py`，以 `mock:=true|false` 選擇 GenericSystem／FANUC plugin，共用 forward position controller。其他三個入口刪除；teleop 入口保留。interpolation 直接用具名 `Node(...)`，不另外包 launch。
7. startup motion、Cartesian／circle controller 與 facing-circle 共用流程位於 `scripts/motion/`。motion recording 分別觀測 target、500 Hz command 與 feedback。
8. 執行單元、ROS 節點、安裝後 GenericSystem launch、暫停／恢復及 20／50／100／500 Hz 的 linear／cubic 量測。操作見 `INTERPOLATION.md`，實測結果見 `INTERPOLATION_TEST_RESULTS.md`。本次不執行實機 motion。

最後強調：**interpolation 輸入和輸出都是 joint sequence，linear／cubic 都只在 joint space 中計算。interpolation 不包含 Cartesian position、pose、orientation、Cartesian 軌跡或 IK；Cartesian motion 必須由上游先轉成 joint sequence。**
