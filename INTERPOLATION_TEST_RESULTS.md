# Joint interpolation 驗證紀錄

2026-09-15，ROS 2 Jazzy、本機 localhost 隔離 DDS domains。以下為簡化後的版本；沒有執行實機 motion。

## 建置與回歸

`colcon build` 成功，安裝至 `/tmp/dual_crx_build/install`。相關 pytest 結果為 **102 passed、1 skipped、0 failures**：

| Suite | Passed |
| --- | ---: |
| cartesian | 27 |
| interpolation | 18 |
| interpolation_node | 18 |
| motion_routes | 4 |
| circle | 16 |
| facing_circle | 6 |
| latency | 8 |
| teleop_bridge（包含統一 bringup） | 5 |

`dual_test4_transport` 略過：工作樹中的 `scripts/dual_test4.py` 已不存在，未重新建立。CMake 同步移除不存在的執行檔安裝項目。這不代表 legacy test4 規劃或 transport 在本版已通過。

覆蓋 linear／cubic、20／50／100／500 Hz、6／12 軸、名稱排序、非法資料拒絕、等待首筆 feedback、input 暫停後持續保持／恢復、feedback 停止後仍繼續，以及不再使用 session 或運動限制參數。

安裝後 `dual_arm.launch.py mock:=true` 與 `teleop_joint.launch.py mock:=true` 都以 GenericSystem 執行：確認兩臂 controller active、初始實際 feedback、啟動不產生命令、小幅 joint 移動、500 Hz 持續保持與恢復。teleop 輸入為 20 Hz，feedback 約 100 Hz。測試驗證唯一 controller publisher，但執行中的節點不實施 publisher 排他檢查。

標準 joint_state_publisher 在兩臂 discovery 期間可能發布空位置或尚未填齊的合併位置；測試以原始單臂 feedback 核對硬體初始姿態，待合併完成再驗證 `/joint_states`。此合併 topic 不作為 interpolation 的輸入 feedback。

## 每組約 30 秒頻率量測

安裝後統一 GenericSystem launch，每組暖機約 1 秒，再量測約 30 秒。全部端點誤差為 **0 rad**。

| 輸入 Hz | 方法 | 平均輸出 Hz | p95 間隔 ms | p99 間隔 ms | 最大間隔 ms |
| ---: | --- | ---: | ---: | ---: | ---: |
| 20 | linear | 499.67 | 2.090 | 2.205 | 9.010 |
| 20 | cubic | 499.44 | 2.181 | 2.269 | 16.143 |
| 50 | linear | 499.87 | 2.196 | 2.314 | 6.238 |
| 50 | cubic | 499.58 | 2.390 | 2.797 | 7.529 |
| 100 | linear | 492.52 | 2.335 | 3.211 | 14.541 |
| 100 | cubic | 498.05 | 2.256 | 2.639 | 11.939 |
| 500 | linear | 497.62 | 2.199 | 2.462 | 13.018 |
| 500 | cubic | 499.11 | 2.234 | 2.550 | 7.187 |

數據：[rate_metrics.json](docs/interpolation/rate_metrics.json)。8 組均通過平均 475～525 Hz 門檻。500 Hz cubic 首次與其他 ROS 回歸／軌跡測試同時執行時只達 **469.72 Hz**，未通過；停止其他重負載測試後單獨重測為表內 **499.11 Hz**。因此不能宣稱高負載下仍保證 500 Hz，這是一般 Python／作業系統排程的實測限制。

取樣時間使用節點本機 CLOCK_MONOTONIC。JSON 的雙臂 skew 是訂閱端 callback 接收時間差，不是硬體同步或網路延遲保證。

## Cartesian 完整 GenericSystem mock

安裝後 launch 與獨立 motion executable 的完整驗收通過：啟動靜止、雙臂 world-X 正弦 ±20 mm、固定 orientation、校正後全域 TF、command／feedback 紀錄，以及上游退出後仍持續發布最後位置。

- 左／右實測 controller topic 約 494.10／494.11 Hz。
- motion 觀測 command 491.71 Hz，IK 49.66 Hz。
- 最大非運動軸誤差約 2.74 μm，雙臂 Cartesian 位移差約 1.09 μm。
- 結束後 CSV 與圖檔正常產生。

數據：[cartesian_metrics.json](docs/interpolation/cartesian_metrics.json)。Cartesian 分析在上游／測試完成，interpolation 本身仍僅處理 joint sequence。

## Facing-circle 完整 GenericSystem mock：精度門檻未通過

完成起步、面對面 approach 與 10 圈有限軌跡，正常回報 `Finite motion finished` 並產生 CSV／圖檔，無 ABORT。command 平均 **498.91 Hz**，IK **49.95 Hz**。

但是既有半徑誤差門檻 **0.1 mm 未通過**：左／右 command 與 feedback 最大半徑誤差分別為 **0.12596／0.13074 mm**。XZ 閉合誤差約 **0.00467／0.00480 mm**。記錄中 command 最大取樣間隔 **26.08 ms**，target 最大間隔 **54.26 ms**；平均頻率不能保證每一步準時。

數據：[facing_circle_metrics.json](docs/interpolation/facing_circle_metrics.json)。此結果取代舊 software mock 的驗收數據，不能宣稱本版 GenericSystem facing-circle 精度驗收全部通過。未放寬門檻、加入 Cartesian 修正，或恢復已取消的運動限制；interpolation 的輸入／輸出仍然都是 joint sequence。

## 行為界線

沒有 URDF 運動限制、step／velocity 防護、超限退回 linear、session、輸入／feedback watchdog 或重複 publisher 中止。相關舊測試已移除，改驗證本次約定的資料檢查與持續保持。

上游 motion 既有的 IK、joint target 與起步檢查仍保留；不能將這些結果解讀為所有入口具有相同運動防護。本次沒有實機、硬即時或碰撞驗證。
