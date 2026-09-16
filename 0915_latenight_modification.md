# 0915 深夜修改討論紀錄

狀態：使用者已同意開始實作。下方保留討論歷程；以本節最終定案與 `INTERPOLATION.md` 為準。

## 最終定案與實作

- 無新輸入時完成最後一段，持續以 500 Hz 發布端點；新目標直接接續。
- feedback 只用於第一段起點；client 只傳資料，移除 session、watchdog 與重複 publisher 檢查。
- interpolation 僅保留資料檢查；不讀 URDF、不檢查運動限制或因超限退回 linear。
- 四個 bringup 合併到 `dual_arm.launch.py`，mock 使用 GenericSystem，其餘三個入口刪除。
- interpolation 在 bringup／teleop 中直接以 Node 定義，刪除單獨 launch 包裝。teleop 輸入維持 20 Hz。
- 建置成功，相關測試 102 通過、1 略過。已執行 GenericSystem launch、頻率與軌跡驗證；高負載降頻和 facing-circle 精度門檻未通過的結果如實記錄於 `INTERPOLATION_TEST_RESULTS.md`。未執行實機 motion。
- interpolation 進出都是 joint sequence，不含 Cartesian。

---
以下是實作前的討論紀錄，當時的「待確認」項目已由上述定案取代。

## 已確認的方向

interpolation 的職責簡化為「接收 joint sequence、插值、輸出 joint sequence」，只保留資料檢查。運動限制由上游 motion／規劃端負責，不在 interpolation 裡重複檢查。

既有需求維持：

- 輸入與輸出都是 joint sequence，不包含 Cartesian 資料或運算，也不做 FK／IK。
- 可選 linear 或 cubic spline。
- interpolation 輸出頻率固定設定為 500 Hz。
- teleoperation launch 的輸入頻率為 20 Hz。

## 待實作：保留資料檢查，移除 interpolation 的運動限制

保留的資料檢查：

- joint 數量、陣列長度及名稱是否符合介面。
- 是否有重複／缺失的 joint；依 joint name 排序。
- joint 數值是否為有限值，拒絕 NaN／Inf。
- 輸入頻率是否合法。
- 插值使用的時間點是否有效、是否嚴格遞增，避免無法建立 spline。

從 interpolation 移除：

- 依 URDF 檢查關節角度上下限。
- 關節速度限制。
- 每次輸出的 joint step 限制。
- cubic 曲線的位置／速度極值限制檢查，以及因超限而退回 linear 或中止的流程。
- 僅為上述運動限制存在的 URDF 載入、參數、launch 設定與相應測試。

資料不足時先用 linear 等數學上的初始化處理，可繼續保留；這與超限後切換 linear 是不同的事情。沒有未來目標時不做無限外插。

這次不會順便移除上游 motion script 原本的角度、速度、加速度、碰撞或追蹤檢查。各入口的檢查並不一致，不能把移除 interpolation 的限制描述成「所有入口都已有相同保護」。

## 待繼續確認的行為

以下是前面討論過、尚需整理定案的細節，暫不實作：

1. **沒有新輸入時**：討論方向是完成最後一段、維持最後 joint position，新目標到達後繼續，而不是因未更新就中止 session。需確認維持位置時是否持續以 500 Hz 發布最後值。
2. **feedback 的角色**：需確認是否只用於第一段的起始 joint position，以及是否移除 feedback 過期就停止的條件。
3. **node／client 的簡化**：需確認移除 session 控制權、重複 publisher 檢查、斷線 watchdog 等額外管理後，client 要保留哪些最小傳送功能。
4. **正常結束與停止**：需確認有限 motion 如何送完最後目標，以及何時停止發布；應與輸入暫停的行為分開定義。
5. **cubic 的結果**：移除運動限制後，cubic 可能超出輸入 waypoint 的數值範圍。資料合法不代表整條插值曲線已通過運動限制檢查；interpolation 不再負責這項判定。

## 討論完成後的整體修改

- 依定案內容修改 interpolation 數學核心、node 與 client。
- 同步調整 motion scripts、teleop bridge、launch 及設定，刪除不再使用的參數。
- 更新測試：保留資料驗證、linear／cubic 數學、頻率與 joint sequence 流程測試；移除已取消機制的測試，補上最後目標與暫停／恢復行為的測試。
- 直接執行 ROS mock 測試，不需再次取得使用者同意；本討論不授權實機試跑。
- 更新操作文件與測試報告，清楚記錄 interpolation 已不再執行運動限制檢查。


## 新增討論：合併四個 robot bringup launch

使用者希望將下列四個檔案合併，以 `dual_arm.launch.py` 作為統一入口：

- `dual_arm.launch.py`
- `dual_arm_mock.launch.py`
- `dual_cartesian.launch.py`
- `dual_cartesian_mock.launch.py`

### 合併原則

joint motion 與 Cartesian motion 最後都產生 joint commands，使用同一組 `forward_position_controller`，因此 robot bringup 不需依 joint／Cartesian 分開。motion script 負責選擇軌跡與 IK，launch 只負責 robot、controller、interpolation 與顯示。

mock／實機改由 launch argument 切換，例如：

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true
ros2 launch dual_crx_control dual_arm.launch.py mock:=false
```

此處應使用 `mock`／`use_mock` 參數，而不是 ROS 的 `prefix`。`prefix` 是 joint／link 名稱前綴，例如 `left_`、`right_`；`namespace` 是節點／topic 分組，例如 `/left`、`/right`。兩者都不能自行切換硬體後端。若使用者說的 prefix 是泛指命令後面的選項，實作上就是上述 launch argument。

### 保持 dual_arm.launch.py 的寫法

保留 `generate_launch_description()`，先以具名變數清楚定義 `robot_description`、`robot_state_publisher`、`robot1`、`robot2`、controller spawners、interpolation 與 RViz，最後集中放入 `LaunchDescription([...])`。

這與其他檔案使用的是同一套 ROS 2 Python launch API。`actions.append()`、迴圈和 `GroupAction` 是不同組織方式，不是另一種 launch 格式；合併後可以維持使用者偏好的明確具名變數寫法，必要的條件與 remapping 仍需保留。

### 已查到的功能差異，合併時需處理

- 目前 `dual_arm_mock.launch.py` 已只是轉呼叫 `dual_cartesian_mock.launch.py`。
- 目前 Cartesian mock 使用 `dual_mock_robot.py` 直接模擬 topic，沒有真正的 ros2_control controller manager；它並非單純把實機 driver 改個參數。
- 若希望 mock／實機真的共用同一個 `forward_position_controller` 架構，建議 mock 改用 ros2_control `GenericSystem`，可沿用 teleop launch 已驗證的 hardware plugin 切換方式。software mock 是否只留下作為測試工具，於實作前統整定案。
- `dual_cartesian.launch.py` 另有 driver TF 隔離、左右 joint states 合併、IP／RViz 參數與 `child_link` 設定，不能因為合併而漏掉。應以校正後的雙臂 URDF 維持全域 TF。
- 本機已安裝的 `fanuc_physical_control.launch.py` 沒有宣告或轉傳 `use_mock` 給 Xacro。現有 `dual_arm.launch.py` 雖傳入 `use_mock: false`，不能據此認定改成 true 就會切換成 mock。統一 launch 必須把選項實際接到支援 `use_mock` 的 Xacro／hardware plugin，不能只修改 IncludeLaunchDescription 的字串。
- interpolation 只啟動一次，mock 與實機維持相同 joint/topic 命名；不新增 Cartesian 專用控制介面。

### 待辦與範圍

- 討論完成後再合併，這次不修改 launch 程式。
- 以 `dual_arm.launch.py` 作為主要維護檔案；其餘三個檔案要直接刪除或短期保留轉呼叫，仍待決定，避免留下四份重複實作。
- 更新 motion 範例、mock 驗收腳本與文件中的 launch 名稱及參數。
- 本次提出的合併範圍是上述四個檔案；teleop 與 readonly launch 的外部入口暫不變動。

## 已確認：不要隨意額外包 launch

- 單一 ROS node 直接在使用它的 launch 中以具名變數 `Node(...)` 定義，最後加入 `LaunchDescription([...])`。
- interpolation 直接定義為 `interpolation = Node(...)`，不再透過 `IncludeLaunchDescription` 載入 `interpolation.launch.py`。
- 討論完成後，移除這份多餘的 interpolation launch 包裝，同步調整引用它的 bringup、teleop、測試與文件；teleop 的入口名稱及 20 Hz 輸入要求維持。
- 不為了包裝單一節點或轉呼叫而任意新增 launch 檔案。既有外部 driver 若以 launch 啟動完整子系統，可依實際需要保留 include；這項規則不是要求把外部 driver 的全部內容複製進來。
- 優先維持 `dual_arm.launch.py` 的直接、具名變數寫法，避免不必要的巢狀包裝。

以上只記錄已確認的修改方向，仍等全部討論完成後再一起實作。
