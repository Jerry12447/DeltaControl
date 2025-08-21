工作區建立

colcon build
source install/setup.bash

ros2 launch delta_robot_isaacsim delta_robot_isaacsim.py


# 發送移除座標
ros2 topic pub /removed_cords std_msgs/msg/UInt16MultiArray "{
  layout: {
    dim: [
      {label: 'group', size: 2, stride: 4},
      {label: 'coordinate', size: 2, stride: 2}
    ],
    data_offset: 0
  },
  data: [320, 240, 400, 300]
}"# DeltaControl
# DeltaControl
