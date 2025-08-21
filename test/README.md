# Delta機器人軌跡控制測試模組


### 2. 角度控制模組 (AngleController)
- 高頻控制（預設200Hz）
- 連續發布位置到 `/joint_state` 通道
- 支援軌跡執行和位置保持模式
- 自動發送精確的最終位置

### 3. 主控制模組 (MainController)
- 整合軌跡規劃和角度控制
- 提供ROS2服務介面
- 支援測試軌跡執行

## 使用方法

### 1. 啟動主控制器
```bash
ros2 run delta_robot_isaacsim main_controller
```

### 2. 執行測試
```bash
ros2 run delta_robot_isaacsim test_trajectory
```



### 4. 發布目標角度
```bash
# 發布目標角度 [θ1, θ2, θ3]（弧度）
ros2 topic pub /target_angles std_msgs/msg/Float32MultiArray "data: [1.57, 0.785, 1.047]"
```

## 控制參數

- **預設最大角速度**: π/2 rad/s (90°/s)
- **預設最大角加速度**: π/4 rad/s² (45°/s²)
- **控制頻率**: 200 Hz
- **關節名稱**: 
  - part_reducer1________377
  - part_reducer2________334
  - part_reducer3________362

## 軌跡類型
均速控制



