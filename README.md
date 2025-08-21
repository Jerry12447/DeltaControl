## 🏗️ 系統架構

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Isaac Sim     │    │   Delta Motion  │    │  Delta Control  │
│   Simulation    │◄──►│   Kinematics    │◄──►│   Controller    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌─────────────────┐
                       │ Trajectory      │
                       │ Planner         │
                       └─────────────────┘
```

### 1. 編譯項目

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
```
## 2. 使用方法

```bash

# 啟動控制節點
ros2 launch delta_robot_isaacsim delta_robot.launch.py

# 發送移除像素座標
ros2 topic pub /removed_cords std_msgs/msg/UInt16MultiArray "{
  layout: {
    dim: [
      {label: 'group', size: 2, stride: 4},
      {label: 'coordinate', size: 2, stride: 2}
    ],
    data_offset: 0
  },
  data: [320, 240, 400, 300]
}"
```

## 3. 配置參數

### 機器人幾何參數

```python
self.RD_RF = 300     # 上臂長度 (mm)
self.RD_RE = 730     # 下臂長度 (mm)
self.RD_F = 334.641  # 固定平台半徑 (mm)
self.RD_E = 207.846  # 移動平台半徑 (mm)
```

### 控制參數

```python
self.max_velocity = math.pi / 16.0      # 最大角速度 (rad/s)
self.control_frequency = 200.0          # 控制頻率 (Hz)
```

### 角度限制

```python
min_angle_deg = -38.05  # 最小角度限制
max_angle_deg = 90.05   # 最大角度限制
```