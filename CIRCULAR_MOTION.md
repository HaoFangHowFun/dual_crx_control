# 雙臂 Cartesian 畫圓：dual_test4.py

`scripts/dual_test4.py` 先擷取雙臂即時 joint states，以該姿態使用 Pinocchio 完成整段軌跡與碰撞驗證，再透過既有的 `forward_position_controller/commands` 同步發送雙臂六軸位置命令。右臂是 `robot1`，左臂是 `robot2`。

## 環境與啟動

工作站現有的 `retargeting_ros` 環境已具備 Python 3.12、Pinocchio 4.1.0、Coal、NumPy、SciPy、Matplotlib，並可搭配 ROS 2 Jazzy。以下指令直接使用該 Python，無需更改系統 Python 或重建工作區：

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash

/home/msc-crx/miniconda3/envs/retargeting_ros/bin/python \
  src/dual_crx_control/scripts/dual_test4.py --dry-run
```

`--dry-run` 預設會訂閱兩臂的即時 joint states，但不建立運動命令 publisher，也不要求 forward controller 已啟用。缺少任一臂的新鮮回授會超時退出，不會退回全零姿態。xacro 及 mesh 套件查找仍需要 source ROS 與工作區環境。第一次驗證可能需要數分鐘，主要成本為 mesh 距離計算。

如需完全離線重播，使用 `--dry-run --initial-joints-file <initial_joints.json>`。JSON 格式為 `{"robot1": [六個 J1～J6 rad], "robot2": [六個 J1～J6 rad]}`；每次擷取後都會儲存這個檔案。此參數只允許 dry-run，實機執行一律重新讀取 live feedback。

其他工作站可以使用相容的系統套件，例如 `ros-jazzy-pinocchio`、`ros-jazzy-xacro`、`python3-numpy`、`python3-scipy`、`python3-matplotlib`，以及本 package.xml 的 ROS 執行依賴。Pinocchio 必須包含 collision geometry 支援。不要將 PyPI 上名稱為 `pinocchio` 的其他套件誤認為機器人運動學庫。

查看參數：

```bash
/home/msc-crx/miniconda3/envs/retargeting_ros/bin/python \
  src/dual_crx_control/scripts/dual_test4.py --help
```

實機執行時，先啟動既有 `dual_arm_launch.launch.py` 並讓兩臂停留在希望開始畫圓的姿態。省略 `--dry-run` 後，腳本會擷取兩臂即時角度，再完成規劃、檢查 controller 與回授，並要求在終端按 ENTER 才開始：

```bash
/home/msc-crx/miniconda3/envs/retargeting_ros/bin/python \
  src/dual_crx_control/scripts/dual_test4.py \
  --radius 0.10 --cycles 1 --duration 20
```

這份實作使用來源檔案直接執行，未新增 `ros2 run` 安裝入口。

## 軌跡與預設值

| 項目 | 行為 |
| --- | --- |
| 模型 | `description/urdf/dual_crx.urdf.xacro`，可用 `--urdf` 指定 |
| 起始關節 | 從 `/robot1/joint_states` 與 `/robot2/joint_states` 擷取的新鮮 J1～J6 角度；依名稱映射 |
| EEF | `right_ee_mount`／`left_ee_mount`，與目前 URDF flange 重合 |
| 半徑 | 0.10 m |
| 圓心 | 各臂擷取姿態的 EEF 沿 world −X 平移一個半徑 |
| 平面 | world XY；各臂 world Z 固定 |
| 方向 | 從 world +Z 向下看逆時針；`--direction cw` 可反轉 |
| 時間 | 一圈 20 秒；`--duration` 是所有 `--cycles` 的總畫圓時間 |
| 加減速 | 整段使用 quintic 相位函數，起訖速度及加速度為零；角速度不是常數 |
| 姿態 | 預設 `--orientation-mode position`，只求解 J1～J3，J4～J6 維持各自擷取的角度；工具方向會隨路徑改變 |
| 固定方向 | `--orientation-mode fixed` 使用六軸 IK 維持初始方向；若奇異點、分支閉合或其他檢查失敗就拒絕執行 |
| 發送率 | 主機目標 500 Hz，可用 `--rate` 指定；不是硬即時保證 |
| 動作順序 | 2 秒起始小幅銜接 → 1 秒 hold → 畫圓 → 1 秒 hold |

Xacro 僅提供幾何、關節軸線、限制與 base 安裝位置。起始關節角來自即時回授，並由 FK 計算各臂的起始 EEF 位置、方向與圓心；不要求歸零，也不以其他 SRDF 的 default group state 作起點。

位置模式固定 wrist 是為這個三維位置任務選定冗餘自由度的方式，讓軌跡容易理解、保持連續並回到擷取的起始姿態。程式不會在位置模式失敗後自行釋放 wrist 或改用另一個起始姿態。

可行性取決於每次擷取的姿態，會重新檢查完整路徑。固定方向模式若不能收斂或閉合就拒絕執行，不自動切換位置模式。

## 驗證與中止

- IK 使用一致的 world 表示、阻尼、步長限制與回溯。每個 waypoint 以前一點為 seed。不能收斂就回報手臂、相位與目標位置。
- 完整一圈必須回到相同的 joint configuration。週期 cubic spline 的首尾設定為擷取的初始關節角；允許的 IK 終點修正最多為 1 microradian，並驗證修正後的插值。
- 精確計算 cubic spline 的關節極值與一、二階導數上界，再與平滑時間函數的導數上界合成。若所需時間較長，同時延長兩臂的總時間；不改半徑、高度或圈數。
- 速度限制預設為 URDF 的 10%；加速度上限預設為每軸 0.5 rad/s²，這是明確配置值，並非 URDF 自帶限制。
- 在 2,881 個相位樣本檢查插值後的 FK、半徑、Z 與方向。位置門檻為 1 mm；固定方向模式的方向門檻為 0.5°。這些是模型驗證門檻，不能當作實機 TCP 精度保證。
- 使用 URDF collision meshes 檢查自碰撞、雙臂與桌面，包含固定 base/table 組合。只排除同 link 或 URDF 直接相鄰的碰撞對，並在報告列出排除項目。
- 碰撞檢查使用區間中點距離和保守的剛體移動上界；無法證明整個區間有至少 2 mm clearance 就繼續細分，碰撞或達到細分上限時拒絕軌跡。這涵蓋規劃的連續關節曲線；實機的跟隨誤差與控制器內插行為仍可能偏離該曲線。
- 規劃起點為開始時擷取的 joint snapshot。規劃後及操作者按 ENTER 後，均重新取得回授；若任一軸偏離擷取值超過 `--start-tolerance-deg`（預設 0.5°），在送出命令前退出，提示重新執行以擷取新姿態及重新規劃。容許範圍內的小幅偏差會先做已驗證的平滑銜接，目標是擷取姿態而非零位。
- 使用 controller-manager service 確認兩臂的 forward position controller 為 active，並讀取其 `joints` 參數確認為對應的 J1～J6 順序。執行期間持續查詢 active 狀態。
- 任一臂 stale feedback、訂閱者消失、controller 異常、追蹤誤差持續超標、主迴圈延遲或下一筆關節步長過大，都會讓兩臂共同退出軌跡，不再發送後續目標。

兩次 publish 使用同一時刻的軌跡，但 ROS topic 沒有共同執行時間戳，且兩次發送不是原子操作。`publish_span_s` 量測的是兩次 publish 呼叫所花時間，並非硬體實際啟動偏差。

現有 FANUC `hardware_interface.cpp::write()` 會持續發送儲存的 `joint_targets_`；停止本程式的 publisher 不等於急停，機器人仍可能走到最後收到的目標。本程式異常時不額外送出回零或未驗證的保持姿態。需停止實體動作時使用硬體既有停止機制。

碰撞模型只包含指定 URDF 的物件。其桌面為 0.30 × 0.30 m，沒有實際工具及周邊完整環境；world/base 安裝位置必須與實體相符。

## 結果與程式結構

每次在 `circular_motion_results/<時間戳>/` 建立獨立資料夾，可透過 `--output-dir` 更改。包含：

- `expanded.urdf`：本次使用的模型與解析後 mesh 路徑。
- `initial_joints.json`：本次擷取的兩臂 J1～J6 rad，可用於離線重播；`summary.json` 另記錄回授 timestamp、接收年齡與 FK 起點。
- `summary.json`：參數、Pinocchio 版本、IK／幾何檢查、連續碰撞 clearance 下界、關節導數上界與執行狀態。失敗亦寫入原因。
- `planned.csv`、`planned_cartesian.png`、`planned_joints.png`：離線目標與 FK 結果。
- 實機已送出命令時另有 `executed.csv` 與對應圖表；中斷時仍儲存已收集資料。CSV 在繪圖前寫出，繪圖失敗也保留量測資料。

CSV 記錄 J1～J6 rad、目標／命令 FK／回授 FK、回授到達年齡及 ROS timestamp、命令時間與發布耗時。繪圖與 FK 後處理在動作結束後執行。回授比較使用各臂最新樣本，沒有把不同 ROS timestamp 假定為同時量測；它也不是外部 TCP 量測。

程式主要區塊依執行流程排列：`RobotModel`（模型、IK、碰撞）→ `CirclePlan`（曲線、時間、驗證）→ `RobotIO`（ROS 介面）→ `execute`（共同時鐘）→ `save_records`（結果）。

## 測試

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/home/msc-crx/miniconda3/envs/retargeting_ros/bin/python \
  -m unittest discover -s src/dual_crx_control/tests -v
```

測試包含零位 FK、10 cm 圓、關節閉合、導數限制、反向與多圈時間配置、不可達目標、碰撞拒絕、起始角度不符及 CLI 無效值。ROS 測試使用 localhost-only 的 domain 173 與假 controller，驗證實際 message/service、六軸排序、單臂 stale feedback、controller inactive、訂閱者消失及追蹤誤差中止；不連線到實機所在的 ROS domain。

### 2026-09-11 舊版零位規劃驗證記錄（歷史結果）

- 13 個自動測試已分組執行通過，包含完整串流迴圈、Ctrl+C 中斷後保存部分 CSV／圖表，以及固定方向 Jacobian 的有限差分核對。
- 預設 0.10 m、20 秒、逆時針、位置模式 dry-run 通過。模型上的最大位置誤差小於 `1e-10 m`；此數字表示求解及插值的數值一致性，不是實機精度。
- 整條關節曲線的保守 collision clearance 下界約 15.6 mm，大於設定的 2 mm。工具方向相對零位最大改變約 12.79°。
- 反向兩圈並要求 0.1 秒的測試中，共用時間自動延長至約 24.29 秒，以滿足配置的關節導數限制。
- 固定方向模式在初始零位的第一個非零相位拒絕執行；已保存失敗原因，未發出硬體命令。
- Python 語法檢查與 flake8 的 E/F/W 檢查通過（使用 119 字元行寬，允許運算子前換行）。未執行任何真實機器人動作。

工作站上的完整預設 dry-run 輸出位於：
`/home/msc-crx/ws_fanuc/circular_motion_results/20260911_164142_969634/`。

### 2026-09-11 即時初始配置更新

- 起始角度改為雙臂即時 joint snapshot；圓心、初始方向、固定 wrist 角度、首尾關節值與執行前漂移檢查均使用同一份 snapshot。原本的實機執行命令可直接沿用。
- 19 個測試已分組通過，新增非零起點的完整圓、原 wrist 角度保持、首尾閉合、唯讀即時擷取、非零起點串流、操作者等待時移動的拒絕行為、缺少任一臂回授時超時，以及離線 JSON 重播與格式檢查。
- 使用你提供的實際角度做模型重播時，初始配置已被接受，但目前 Xacro 在該姿態回報 `left_J2_link_0 / right_J4_link_0` 碰撞。程式會在規劃前明確回報這個模型碰撞，保留碰撞檢查；它不代表已量測到實機碰撞。需確認雙臂 base 安裝位姿與模型是否一致。
- 未執行真實機器人動作；驗證使用離線資料及隔離的 mock ROS domain。
