#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Delta Firmware 底層控制器
接收G1指令並執行笛卡爾空間插值，發布關節狀態給Isaac Sim
對應實際硬體中的Delta_Firmware
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32, Bool
from delta_firmware.delta_firmware_motion import DeltaFirmwareMotion
import math
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class DeltaFirmwareController(Node):
    
    def __init__(self):
        super().__init__('delta_firmware_controller')
        
        # 初始化運動控制器
        self.motion_controller = DeltaFirmwareMotion()
        
        # 創建回調組
        self.callback_group = ReentrantCallbackGroup()
        
        self.g1_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_firmware/g1',
            self.g1_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 新增M指令訂閱者
        self.m_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_firmware/m',
            self.m_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.angle_pub = self.create_publisher(
            JointState,
            '/target',
            10
        )
        
        # 批次完成狀態發布者
        self.batch_complete_pub = self.create_publisher(
            Bool,
            '/batch_execution_complete',
            10
        )
        
        # 新增夾爪控制發布者
        self.finger_pub = self.create_publisher(
            JointState,
            '/finger',
            10
        )
        
        # 初始化 joint state 消息（保持與原版本相同）
        self.joint_state = JointState()
        self.joint_state.name = ['part_reducer1________377', 'part_reducer2________334', 'part_reducer3________362']
        self.joint_state.position = [0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0, 0.0]
        
        # 初始化夾爪 joint state 消息
        self.finger_joint_state = JointState()
        self.finger_joint_state.name = ['finger_joint']
        self.finger_joint_state.position = [0.0]
        self.finger_joint_state.velocity = [0.0]
        self.finger_joint_state.effort = [0.0]
        
        # 控制狀態
        self.is_executing = False
        self.hold_mode = False
        self.startup_completed = False
        
        # 夾爪控制狀態
        self.finger_state = 0.0
        self.finger_target_state = 0.0
        self.finger_velocity = 0.01
        
        # 創建定時器
        self.timer = self.create_timer(
            1.0 / self.motion_controller.control_frequency, 
            self.control_loop
        )
        
        # 啟動時移動到待命位置
        self.startup_timer = self.create_timer(2.0, self.startup_sequence)
        
        self.get_logger().info("Delta Firmware 底層控制器已初始化完成（笛卡爾空間插值模式）")
    
    def publish_angles(self, angles):
        """發布馬達角度"""
        angle_values = [
            round(angles.Theta1, 6),
            round(angles.Theta2, 6),
            round(angles.Theta3, 6)
        ]
        
        if not self.motion_controller.check_angle_limits(angles):
            self.get_logger().error(f"角度超出範圍: θ1={math.degrees(angles.Theta1):.2f}°, θ2={math.degrees(angles.Theta2):.2f}°, θ3={math.degrees(angles.Theta3):.2f}°")
            return False
        
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = angle_values
        self.angle_pub.publish(self.joint_state)
        return True
    


    def g1_callback(self, msg):
        """處理G1線性移動命令（參考實際硬體）"""
        if len(msg.data) < 3:
            self.get_logger().error("G1命令數據格式錯誤：需要至少3個參數 [X, Y, Z]")
            return
        
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        velocity = msg.data[3] if len(msg.data) > 3 else None
        
        self.get_logger().info(f"執行G1 X{x:.2f} Y{y:.2f} Z{z:.2f} P{velocity}")
        self.execute_g1_motion(x, y, z, velocity)
    
    def m_callback(self, msg):
        """處理M指令（夾爪控制）"""
        if len(msg.data) < 1:
            self.get_logger().error("M指令數據格式錯誤：需要至少1個參數 [M_CODE]")
            return
        
        m_code = int(msg.data[0])
        
        if m_code == 0:
            self.get_logger().info("執行M0指令")
            self.execute_m0_motion()
        elif m_code == 1:
            self.get_logger().info("執行M1指令")
            self.execute_m1_motion()
        else:
            self.get_logger().error(f"未知的M指令代碼: {m_code}")


    def execute_g1_motion(self, x, y, z, velocity=None):
        """執行G1線性移動"""
        if self.is_executing:
            self.get_logger().warn("正在執行其他動作，等待完成後執行")
            # 等待當前動作完成
            while self.is_executing:
                time.sleep(0.01)
        
        self.is_executing = True
        
        # 檢查是否為待命位置（在執行前檢查）
        is_standby = self.is_standby_position(x, y, z)
        if is_standby:
            self.get_logger().info("檢測到待命位置指令，將在移動完成後發布批次完成信號")
        
        try:
            # 使用混合運動控制器執行G1移動
            result = self.motion_controller.g1_motion(x, y, z, velocity)
            
            if result is True:
                self.get_logger().info("G1移動完成")
                # 如果是待命位置，發布批次完成信號
                if is_standby:
                    self.publish_batch_complete()
                return True
            elif isinstance(result, tuple) and len(result) == 2:
                # 返回軌跡點和角度序列
                trajectory_points, trajectory_angles = result
                success = self.execute_trajectory_with_timing(trajectory_points, trajectory_angles, velocity or self.motion_controller.max_velocity)
                if success:
                    self.get_logger().info("G1移動完成")
                    # 如果是待命位置，發布批次完成信號
                    if is_standby:
                        self.publish_batch_complete()
                return success
            else:
                self.get_logger().error("G1移動失敗")
                return False
                
        except Exception as e:
            self.get_logger().error(f"執行G1移動時發生錯誤: {str(e)}")
            return False
        finally:
            self.is_executing = False

    def execute_m0_motion(self):
        """執行M0指令：開啟夾爪（0度）"""
        if self.is_executing:
            self.get_logger().warn("正在執行其他動作，等待完成後執行M0")
            while self.is_executing:
                time.sleep(0.01)
        
        self.is_executing = True
        try:
            result = self.motion_controller.m0_motion()
            if result:
                self.control_finger(0.0)
                #self.get_logger().info("M0指令執行完成")
                return True
            else:
                self.get_logger().error("M0指令執行失敗")
                return False
        except Exception as e:
            self.get_logger().error(f"執行M0指令時發生錯誤: {str(e)}")
            return False
        finally:
            self.is_executing = False

    def execute_m1_motion(self):
        """執行M1指令：關閉夾爪（45度）"""
        if self.is_executing:
            self.get_logger().warn("正在執行其他動作，等待完成後執行M1")
            while self.is_executing:
                time.sleep(0.01)
        
        self.is_executing = True
        try:
            result = self.motion_controller.m1_motion()
            if result:
                self.control_finger(45.0)
                #self.get_logger().info("M1指令執行完成：夾爪關閉（45度）")
                return True
            else:
                self.get_logger().error("M1指令執行失敗")
                return False
        except Exception as e:
            self.get_logger().error(f"執行M1指令時發生錯誤: {str(e)}")
            return False
        finally:
            self.is_executing = False

    def control_finger(self, angle_degrees):
        """控制夾爪角度"""
        finger_msg = JointState()
        finger_msg.header.stamp = self.get_clock().now().to_msg()
        finger_msg.name = ['finger_joint']
        finger_msg.position = [math.radians(angle_degrees)]
        self.finger_pub.publish(finger_msg)

    def execute_trajectory_with_timing(self, trajectory_points, trajectory_angles, velocity):
        """執行軌跡並控制時間（結合虛擬環境的200Hz控制）"""
        if not trajectory_points or not trajectory_angles:
            return False
        
        # 計算總時間
        total_distance = 0
        for i in range(len(trajectory_points) - 1):
            total_distance += self.motion_controller.calculate_cartesian_distance(
                trajectory_points[i], trajectory_points[i + 1]
            )
        
        total_time = total_distance / velocity
        dt = 1.0 / self.motion_controller.control_frequency  # 200Hz控制週期
        
        # 生成時間序列的軌跡點
        num_steps = int(total_time * self.motion_controller.control_frequency) + 1
        time_sequence = [i * dt for i in range(num_steps)]
        
        for t in time_sequence:
            # 根據時間插值找到對應的軌跡點
            if t >= total_time:
                current_angle = trajectory_angles[-1]
            else:
                # 找到對應的軌跡段
                segment_index = int(t / total_time * (len(trajectory_points) - 1))
                if segment_index >= len(trajectory_points) - 1:
                    current_angle = trajectory_angles[-1]
                else:
                    # 在軌跡段內進行插值
                    segment_t = (t / total_time * (len(trajectory_points) - 1)) - segment_index
                    current_angle = self.motion_controller.interpolate_angles(
                        trajectory_angles[segment_index], 
                        trajectory_angles[segment_index + 1], 
                        segment_t
                    )
            
            # 發布關節狀態
            self.publish_angles(current_angle)
            
            # 控制頻率
            time.sleep(dt)
        
        return True

    def move_to_standby_position(self):
        """移動到待命位置"""
        standby_point = self.motion_controller.position_manager.get_standby_position()
        self.get_logger().info(f"移動到待命位置: ({standby_point.X}, {standby_point.Y}, {standby_point.Z})")
        
        result = self.motion_controller.move_to_standby_position()
        if result is True:
            self.get_logger().info("成功移動到待命位置")
            return True
        elif isinstance(result, tuple) and len(result) == 2:
            trajectory_points, trajectory_angles = result
            success = self.execute_trajectory_with_timing(trajectory_points, trajectory_angles, self.motion_controller.max_velocity)
            if success:
                self.get_logger().info("成功移動到待命位置")
            return success
        else:
            self.get_logger().error("移動到待命位置失敗")
            return False

    def move_to_drop_position(self):
        """移動到丟棄位置"""
        drop_point = self.motion_controller.position_manager.get_drop_position()
        self.get_logger().info(f"移動到丟棄位置: ({drop_point.X}, {drop_point.Y}, {drop_point.Z})")
        
        result = self.motion_controller.move_to_drop_position()
        if result is True:
            self.get_logger().info("成功移動到丟棄位置")
            return True
        elif isinstance(result, tuple) and len(result) == 2:
            trajectory_points, trajectory_angles = result
            success = self.execute_trajectory_with_timing(trajectory_points, trajectory_angles, self.motion_controller.max_velocity)
            if success:
                self.get_logger().info("成功移動到丟棄位置")
            return success
        else:
            self.get_logger().error("移動到丟棄位置失敗")
            return False

    def is_standby_position(self, x, y, z):
        """檢查是否為待命位置"""
        standby_point = self.motion_controller.position_manager.get_standby_position()
        tolerance = 10.0  # 10mm容差
        
        return (abs(x - standby_point.X) < tolerance and 
                abs(y - standby_point.Y) < tolerance and 
                abs(z - standby_point.Z) < tolerance)

    def publish_batch_complete(self):
        """發布批次執行完成信號"""
        complete_msg = Bool()
        complete_msg.data = True
        self.batch_complete_pub.publish(complete_msg)
        self.get_logger().info("批次執行完成，通知YOLO進行推論")

    def control_finger(self, target_angle_degrees):
        """控制夾爪開合狀態"""
        target_angle_radians = math.radians(target_angle_degrees)
        
        if target_angle_radians != self.finger_target_state:
            self.finger_target_state = target_angle_radians
            
            self.finger_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.finger_joint_state.position = [target_angle_radians]
            self.finger_joint_state.velocity = [self.finger_velocity]
            self.finger_joint_state.effort = [0.0]
            
            self.finger_pub.publish(self.finger_joint_state)
            
            #state_text = "閉合" if target_angle_degrees == 0 else "打開"
            #self.get_logger().info(f"夾爪控制: {state_text} (角度: {target_angle_degrees}°)")
            
            self.finger_state = target_angle_radians

    def startup_sequence(self):
        """啟動序列：移動到待命位置"""
        if not self.startup_completed:
            self.get_logger().info("開始啟動序列：移動到待命位置...")
            
            if self.move_to_standby_position():
                self.get_logger().info("成功移動到待命位置")
                self.startup_completed = True
                self.hold_mode = True
            else:
                self.get_logger().error("啟動時移動到待命位置失敗")
            
            self.startup_timer.cancel()

    def control_loop(self):
        """主控制迴路"""
        if self.hold_mode and not self.is_executing:
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.angle_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaFirmwareController()  
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中斷信號，正在關閉...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            pass


if __name__ == '__main__':
    main()
