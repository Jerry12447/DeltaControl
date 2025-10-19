# Delta Firmware 虛擬環境包

這個資料夾包含對應實際硬體的Delta Firmware虛擬環境實現，包含指令轉換層和底層控制器，實現笛卡爾空間插值方法與虛擬環境的關節參數要求。

## 檔案結構

```
delta_firmware/
├── __init__.py                    # Python包初始化
├── delta_firmware_motion.py       # 運動規劃核心模組（對應Motion.cpp）
├── delta_firmware_control.py      # 底層控制器（對應Delta_Firmware）
├── delta_firmware_api.py          # 指令轉換層（對應DeltaRobot_API.py）
└── README.md                      # 說明文件
```

## 系統架構

### 對應關係
| 實際硬體 | 虛擬環境 | 功能 |
|----------|----------|------|
| `Delta_Firmware/` | `delta_firmware_control.py` + `delta_firmware_motion.py` | 底層控制器 |
| `DeltaRobot_API.py` | `delta_firmware_api.py` | 指令轉換層 |
| `Trajectory_plan.py` | `Trajectory_plan.py` | 座標轉換 |

### 數據流
```
Trajectory_plan.py → /delta_x/target_array → delta_firmware_api.py → /delta_firmware/g1 → delta_firmware_control.py → /target → Isaac Sim
```

## 主要特性

### 1. 指令轉換層（delta_firmware_api.py）
- 接收 `/delta_x/target_array`（來自Trajectory_plan.py）
- 轉換為G1指令格式
- 發布到 `/delta_firmware/g1`
- 對應實際硬體中的DeltaRobot_API.py

### 2. 底層控制器（delta_firmware_control.py）
- 接收 `/delta_firmware/g1`（來自delta_firmware_api.py）
- 執行笛卡爾空間插值
- 發布關節狀態到 `/target`（給Isaac Sim）
- 對應實際硬體中的Delta_Firmware

### 3. 運動規劃核心（delta_firmware_motion.py）
- 實現笛卡爾空間插值算法
- 提供逆運動學計算
- 對應實際硬體中的Motion.cpp和DeltaKinematics.cpp

### 4. 虛擬環境兼容性
- 保持原有的關節參數要求（-38° 到 90°）
- 使用ROS2 JointState消息格式
- 200Hz控制頻率
- 弧度制角度單位

## 使用方法

### 1. 啟動完整系統

```bash
# 終端1 - 啟動底層控制器
python3 delta_firmware_control.py

# 終端2 - 啟動指令轉換層
python3 delta_firmware_api.py
```

### 2. 測試指令

```bash
# 測試座標轉換結果（模擬Trajectory_plan.py輸出）
ros2 topic pub --once /delta_x/target_array std_msgs/msg/Float32MultiArray "{data: [100.0, 100.0, -700.0, -100.0, -100.0, -700.0]}"

# 監聽關節狀態
ros2 topic echo /target

# 簡化架構：移除執行完成狀態監聽（不再需要）
```

## 簡化架構說明

**移除的功能**：
- `delta_execution_complete` 信號發布和訂閱
- `yolo_execution_complete` 信號發布
- 批次計數和狀態管理邏輯
- 複雜的執行完成回調機制

**簡化後的流程**：
1. `Trajectory_plan.py` 輸出座標
2. `delta_firmware_api.py` 接收並發送G1/M指令
3. `delta_firmware_control.py` 執行指令
4. 完成（不需要回報給API層）

## 與原版本的差異

| 項目 | 原版本 | 虛擬環境版本 |
|------|--------|-------------|
| 插值空間 | 關節角度空間 | 笛卡爾座標空間 |
| 分段方式 | 控制頻率×角速度 | 距離÷每段長度 |
| 軌跡特性 | 曲線路徑 | 直線路徑 |
| 計算次數 | 一次逆運動學 | 多次逆運動學 |
| 硬體兼容性 | 模擬環境 | 實際硬體 |

## 注意事項

1. 虛擬環境版本需要更多的計算資源（多次逆運動學）
2. 軌跡路徑更接近直線，與實際硬體行為一致
3. 保持與原版本的ROS2接口兼容性
4. 指令轉換層專注於指令轉換，不直接控制機器人

## 未來改進

- [ ] 優化計算效率
- [ ] 添加更多軌跡類型（圓弧、貝塞爾曲線）
- [ ] 實現動態速度控制
- [ ] 添加碰撞檢測