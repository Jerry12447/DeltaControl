#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
角度控制模組
實現角度計算、發布和保持功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math
import time
from typing import List, Optional
# 將相對導入改為絕對導入
from trajectory_planner import TrajectoryPlanner

class AngleController(Node):
    """角度控制器"""
    
    def __init__(self, 
                 joint_names: List[str] = None,
                 control_frequency: float = 100.0):
        """
        初始化角度控制器
        
        Args:
            joint_names: 關節名稱列表
            control_frequency: 控制頻率 (Hz)
        """
        super().__init__('angle_controller')
        
        # 設定關節名稱
        if joint_names is None:
            self.joint_names = [
                'part_reducer1________377', 
                'part_reducer2________334', 
                'part_reducer3________362'
            ]
        else:
            self.joint_names = joint_names
        
        # 初始化軌跡規劃器
        self.trajectory_planner = TrajectoryPlanner(control_frequency)
        
        # 當前角度狀態
        self.current_angles = [0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0]
        
        # 控制參數
        self.max_velocity = math.pi / 2.0  # 90度/秒
        self.max_acceleration = math.pi / 4.0  # 45度/秒²
        
        # 創建發布者
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/target', 
            10
        )
        
        # 創建訂閱者
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            '/target_angles',
            self.target_callback,
            10
        )
        
        # 控制狀態
        self.is_executing = False
        self.hold_mode = False
        
        # 創建定時器
        self.timer = self.create_timer(
            1.0 / control_frequency, 
            self.control_loop
        )
        
        self.get_logger().info(f"角度控制器已初始化，控制頻率: {control_frequency}Hz")
    
    def target_callback(self, msg: Float32MultiArray):
        """
        處理目標角度消息
        
        Args:
            msg: Float32MultiArray 包含目標角度 [θ1, θ2, θ3]
        """
        if len(msg.data) != 3:
            self.get_logger().error(f"目標角度數據長度錯誤: {len(msg.data)}，期望3")
            return
        
        # 更新目標角度
        self.target_angles = [float(angle) for angle in msg.data]
        self.get_logger().info(f"收到目標角度: {[f'{angle:.4f}' for angle in self.target_angles]}")
        
        # 開始執行軌跡
        self.execute_trajectory()
    
    def execute_trajectory(self):
        """執行軌跡到目標角度"""
        if self.is_executing:
            self.get_logger().warn("正在執行其他軌跡，請稍後再試")
            return
        
        self.is_executing = True
        self.hold_mode = False
        
        try:
            # 為每個關節生成軌跡
            trajectories = []
            max_time = 0.0
            
            for i in range(3):
                start_angle = self.current_angles[i]
                end_angle = self.target_angles[i]
                
                # 生成均速軌跡
                positions, times, total_time = self.trajectory_planner.generate_constant_velocity_trajectory(
                    start_angle, end_angle, self.max_velocity
                )
                
                trajectories.append({
                    'positions': positions,
                    'start_time': time.time(),
                    'total_time': total_time,
                    'completed': False
                })
                
                max_time = max(max_time, total_time)
            
            # 執行軌跡
            self._execute_trajectories(trajectories)
            
        except Exception as e:
            self.get_logger().error(f"執行軌跡時發生錯誤: {str(e)}")
        finally:
            self.is_executing = False
    
    def _execute_trajectories(self, trajectories: List[dict]):
        """
        執行多關節軌跡
        
        Args:
            trajectories: 軌跡列表
        """
        start_time = time.time()
        
        while not all(traj['completed'] for traj in trajectories):
            current_time = time.time() - start_time
            
            # 計算每個關節的當前位置
            current_positions = []
            all_completed = True
            
            for i, traj in enumerate(trajectories):
                if traj['completed']:
                    current_positions.append(self.target_angles[i])
                else:
                    # 計算當前位置
                    elapsed_time = current_time - (traj['start_time'] - start_time)
                    
                    if elapsed_time >= traj['total_time']:
                        # 軌跡完成
                        current_positions.append(self.target_angles[i])
                        traj['completed'] = True
                    else:
                        # 插值計算位置
                        time_ratio = elapsed_time / traj['total_time']
                        num_steps = len(traj['positions'])
                        step_index = int(time_ratio * (num_steps - 1))
                        
                        if step_index >= len(traj['positions']):
                            current_positions.append(self.target_angles[i])
                        else:
                            current_positions.append(traj['positions'][step_index])
                        
                        all_completed = False
            
            # 更新當前角度
            self.current_angles = current_positions
            
            # 發布關節狀態
            self._publish_joint_state()
            
            # 控制頻率
            time.sleep(1.0 / self.trajectory_planner.frequency)
        
        # 軌跡完成，發送精確的目標角度
        self._send_final_position()
        
        # 切換到保持模式
        self.hold_mode = True
        self.get_logger().info("軌跡執行完成，切換到保持模式")
    
    def _publish_joint_state(self):
        """發布關節狀態"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_angles
        joint_state.velocity = [0.0] * 3  # 暫時設為0
        joint_state.effort = [0.0] * 3    # 暫時設為0
        
        self.joint_state_pub.publish(joint_state)
    
    def _send_final_position(self):
        """發送最終精確位置"""
        self.current_angles = self.target_angles.copy()
        self._publish_joint_state()
        self.get_logger().info(f"發送最終位置: {[f'{angle:.6f}' for angle in self.current_angles]}")
    
    def control_loop(self):
        """主控制迴路"""
        if self.hold_mode:
            # 保持模式：持續發送當前位置
            self._publish_joint_state()
    
    def set_control_parameters(self, max_velocity: float, max_acceleration: float):
        """
        設定控制參數
        
        Args:
            max_velocity: 最大角速度（弧度/秒）
            max_acceleration: 最大角加速度（弧度/秒²）
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.get_logger().info(f"控制參數已更新: 最大速度={max_velocity:.2f} rad/s, 最大加速度={max_acceleration:.2f} rad/s²")
    
    def stop_and_hold(self):
        """停止運動並保持當前位置"""
        self.is_executing = False
        self.hold_mode = True
        self.get_logger().info("停止運動，切換到保持模式")
    
    def get_current_status(self) -> dict:
        """獲取當前狀態"""
        return {
            'current_angles': self.current_angles,
            'target_angles': self.target_angles,
            'is_executing': self.is_executing,
            'hold_mode': self.hold_mode,
            'max_velocity': self.max_velocity,
            'max_acceleration': self.max_acceleration
        } 