### 1. 編譯項目

```bash
cd ~/delta_robot_isaacsim
colcon build                                        #編譯所有功能包
colcon build --packages-select delta_robot_isaacsim #編譯手臂控制功能包
colcon build --packages-select yolo                 #編譯視覺辨識功能包
source install/setup.bash
```
## 2. 使用方法

```bash

# 啟動控制節點
ros2 launch delta_robot_isaacsim delta_robot.launch.py

# 啟動視覺辨識節點
ros2 launch yolo yolo_detection.launch.py

# 發送移除像素座標(只有編譯手臂控制功能包匙時使用)
ros2 topic pub /removed_cords std_msgs/msg/UInt16MultiArray "{
  layout: {
    dim: [
      {label: 'group', size: 6, stride: 12},
      {label: 'coordinate', size: 2, stride: 2}
    ],
    data_offset: 0
  },
  data: [500, 360, 640, 360, 780, 360, 640, 120, 640, 360, 640, 480]
}" --once
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