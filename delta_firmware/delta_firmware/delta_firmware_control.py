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
        
        self.motion_controller = DeltaFirmwareMotion()
        self.callback_group = ReentrantCallbackGroup()
        
        # ========= 訂閱 =========
        # G1 線性移動
        self.g1_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_firmware/g1',
            self.g1_callback,
            10,
            callback_group=self.callback_group
        )
        
        # G28 回待命位置
        self.g28_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_firmware/g28',
            self.g28_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ========= 發布 =========
        # Delta 三軸馬達目標角度
        self.angle_pub = self.create_publisher(
            JointState,
            '/target',
            10
        )
        
        # 夾爪關節
        self.finger_pub = self.create_publisher(
            JointState,
            '/finger',
            10
        )
        
        # 批次完成狀態
        self.batch_complete_pub = self.create_publisher(
            Bool,
            '/batch_execution_complete',
            10
        )
        
        # 單個指令執行完成狀態
        self.command_complete_pub = self.create_publisher(
            Bool,
            '/delta_firmware/command_complete',
            10
        )
        
        # ========= JointState 初始化 =========
        self.joint_state = JointState()
        self.joint_state.name = [
            'part_reducer1________377',
            'part_reducer2________334',
            'part_reducer3________362'
        ]
        # 使用 Motion 模組內部的 current_angle 做同步
        self.joint_state.position = [
            self.motion_controller.current_angle.Theta1,
            self.motion_controller.current_angle.Theta2,
            self.motion_controller.current_angle.Theta3
        ]
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
        
        # 主控制迴圈
        self.timer = self.create_timer(
            1.0 / self.motion_controller.control_frequency,
            self.control_loop
        )
        
        # 啟動時執行 G28（回待命位置）並初始化夾爪為打開狀態
        self.startup_timer = self.create_timer(2.0, self.startup_sequence)
        
        self.get_logger().info("Delta Firmware 控制器初始化完成")
    # ==============================
    # 角度發布 / 檢查
    # ==============================
    def publish_angles(self, angles):
        """發布馬達角度到 /target"""
        if not self.motion_controller.check_angle_limits(angles):
            self.get_logger().error(
                f"角度超出範圍: "
                f"θ1={math.degrees(angles.Theta1):.2f}°, "
                f"θ2={math.degrees(angles.Theta2):.2f}°, "
                f"θ3={math.degrees(angles.Theta3):.2f}°"
            )
            return False
        
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [
            round(angles.Theta1, 6),
            round(angles.Theta2, 6),
            round(angles.Theta3, 6)
        ]
        self.angle_pub.publish(self.joint_state)
        return True
    
    # ==============================
    # G-code Callback
    # ==============================
    def g1_callback(self, msg: Float32MultiArray):
        """
        G1 線性移動指令格式: [x, y, z, velocity, w]
        w: 夾爪角度(度)，0.0=開夾, 33.0=關夾, None=不改變狀態
        """
        if len(msg.data) < 3:
            self.get_logger().error("G1 命令數據格式錯誤：需要至少 3 個參數 [X, Y, Z]")
            return
        
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        velocity = msg.data[3] if len(msg.data) > 3 else None
        
        # W 參數：夾爪角度（度）
        # None (NaN) 表示不改變夾爪狀態，使用當前狀態
        w_angle = msg.data[4] if len(msg.data) > 4 else None
        if w_angle is not None and math.isnan(w_angle):
            w_angle = None
        
        self.get_logger().info(f"執行 G1 X{x:.2f} Y{y:.2f} Z{z:.2f} V{velocity} W{w_angle}")
        self.execute_g1_motion(x, y, z, velocity, w_angle)
    
    def g28_callback(self, msg: Float32MultiArray):
        """處理 G28 回待命位置命令"""
        self.get_logger().info("收到 G28 ")
        self.execute_g28_motion()

    # ==============================
    # G1 / G28 執行邏輯
    # ==============================
    def execute_g1_motion(self, x, y, z, velocity=None, w_angle=None):
        """
        執行 G1 線性移動（與原固件一致）
        w_angle: 夾爪角度(度)，0.0=開夾, 33.0=關夾, None=不改變狀態
        狀態更新在移動開始之前執行
        """
        if self.is_executing:
            self.get_logger().warn("正在執行其他動作")
            while self.is_executing:
                time.sleep(0.01)
        
        self.is_executing = True
        
        is_standby = self.is_standby_position(x, y, z)
        if is_standby:
            self.get_logger().info("目標為待命位置，移動完成後將發布批次完成信號")
        
        try:
            # 1. 先更新夾爪狀態
            if w_angle is not None:
                # 檢查是否需要更新狀態
                target_angle_rad = math.radians(w_angle)
                if abs(self.finger_target_state - target_angle_rad) > 0.01: 
                    self.get_logger().info(f"更新夾爪狀態: W{w_angle:.1f}° ")
                    self.control_finger(w_angle)
            
            # 2. 執行線性移動
            result = self.motion_controller.g1_motion(x, y, z, velocity)
            
            success = False
            if result is True:
                self.get_logger().info("G1 移動完成（距離為 0 或極小）")
                success = True
            elif isinstance(result, tuple) and len(result) == 2:
                traj_points, traj_angles = result
                success = self.execute_trajectory_with_timing(
                    traj_points,
                    traj_angles,
                    velocity or self.motion_controller.max_velocity
                )
                if success:
                    self.get_logger().info("G1 軌跡執行完成")
                else:
                    self.get_logger().error("G1 軌跡執行失敗")
            else:
                # 顯示具體的返回值信息以便調試
                result_type = type(result).__name__
                result_value = result
                if result is False:
                    self.get_logger().error(
                        f"G1 移動失敗：目標點不可達或超出工作範圍 "
                        f"(X{x:.2f} Y{y:.2f} Z{z:.2f})"
                    )
                else:
                    self.get_logger().error(
                        f"G1 移動失敗：返回值不合法 - 類型: {result_type}, 值: {result_value} "
                        f"(X{x:.2f} Y{y:.2f} Z{z:.2f})"
                    )
                return False
            
            # 發布單個指令完成狀態
            self.publish_command_complete(success)
            
            # 批次完成通知
            if success and is_standby:
                self.publish_batch_complete()
            
            return success
        
        except Exception as e:
            self.get_logger().error(f"執行 G1 移動時發生錯誤: {e}")
            return False
        finally:
            self.is_executing = False
    
    def execute_g28_motion(self):
        """執行 G28：回待命位置"""
        if self.is_executing:
            self.get_logger().warn("正在執行其他動作，等待完成後再執行 G28")
            while self.is_executing:
                time.sleep(0.01)
        
        self.is_executing = True
        standby = self.motion_controller.position_manager.get_standby_position()
        
        try:
            self.get_logger().info(
                f"執行 G28，回待命位置: "
                f"({standby.X:.2f}, {standby.Y:.2f}, {standby.Z:.2f})"
            )
            
            result = self.motion_controller.g28_motion(self.motion_controller.max_velocity)
            
            if result is True:
                self.get_logger().info("G28 完成")
                self.publish_command_complete(True)
                self.publish_batch_complete()
                return True
            
            if isinstance(result, tuple) and len(result) == 2:
                traj_points, traj_angles = result
                success = self.execute_trajectory_with_timing(
                    traj_points,
                    traj_angles,
                    self.motion_controller.max_velocity
                )
                if success:
                    self.get_logger().info("G28 軌跡執行完成")
                    self.publish_command_complete(True)
                    self.publish_batch_complete()
                else:
                    self.get_logger().error("G28 軌跡執行失敗")
                    self.publish_command_complete(False)
                return success

            result_type = type(result).__name__
            result_value = result
            if result is False:
                self.get_logger().error(
                    f"G28 移動失敗：待命位置不可達或超出工作範圍 "
                    f"({standby.X:.2f}, {standby.Y:.2f}, {standby.Z:.2f})"
                )
            else:
                self.get_logger().error(
                    f"G28 移動失敗：返回值不合法 - 類型: {result_type}, 值: {result_value} "
                    f"({standby.X:.2f}, {standby.Y:.2f}, {standby.Z:.2f})"
                )
            self.publish_command_complete(False)
            return False
        
        except Exception as e:
            self.get_logger().error(f"執行 G28 時發生錯誤: {e}")
            self.publish_command_complete(False)
            return False
        finally:
            self.is_executing = False

    def control_finger(self, target_angle_degrees: float):
        """控制夾爪開合狀態（角度以度為單位）"""
        target_angle_radians = math.radians(target_angle_degrees)
        
        if target_angle_radians != self.finger_target_state:
            self.finger_target_state = target_angle_radians
            
            self.finger_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.finger_joint_state.position = [target_angle_radians]
            self.finger_joint_state.velocity = [self.finger_velocity]
            self.finger_joint_state.effort = [0.0]
            
            self.finger_pub.publish(self.finger_joint_state)
            self.finger_state = target_angle_radians

    # ==============================
    # 軌跡執行（時間控制）
    # ==============================
    def execute_trajectory_with_timing(self, traj_points, traj_angles, velocity):
        """執行軌跡並控制時間"""
        if not traj_points or not traj_angles:
            return False
        
        total_distance = 0.0
        for i in range(len(traj_points) - 1):
            total_distance += self.motion_controller.calculate_cartesian_distance(
                traj_points[i],
                traj_points[i + 1]
            )
        
        total_time = total_distance / velocity
        dt = 1.0 / self.motion_controller.control_frequency
        
        num_steps = int(total_time * self.motion_controller.control_frequency) + 1
        time_sequence = [i * dt for i in range(num_steps)]
        
        for t in time_sequence:
            if t >= total_time:
                current_angle = traj_angles[-1]
            else:
                segment_index = int(t / total_time * (len(traj_points) - 1))
                if segment_index >= len(traj_points) - 1:
                    current_angle = traj_angles[-1]
                else:
                    segment_t = (t / total_time * (len(traj_points) - 1)) - segment_index
                    current_angle = self.motion_controller.interpolate_angles(
                        traj_angles[segment_index],
                        traj_angles[segment_index + 1],
                        segment_t
                    )
            
            self.publish_angles(current_angle)
            time.sleep(dt)
        
        return True

    # ==============================
    # 啟動 / 狀態維持 / 批次完成
    # ==============================
    def move_to_standby_position(self):
        """保留舊接口：實際上等價於 G28"""
        return self.execute_g28_motion()
    
    def is_standby_position(self, x, y, z):
        """檢查是否為待命位置（用於判斷批次是否完成）"""
        standby = self.motion_controller.position_manager.get_standby_position()
        tol = 5.0  # 5 mm 容差
        return (
            abs(x - standby.X) < tol and
            abs(y - standby.Y) < tol and
            abs(z - standby.Z) < tol
        )
    
    def publish_command_complete(self, success: bool):
        """發布單個指令執行完成狀態"""
        msg = Bool()
        msg.data = success
        self.command_complete_pub.publish(msg)
        if success:
            self.get_logger().debug("指令執行完成")
        else:
            self.get_logger().warn("指令執行失敗")
    
    def publish_batch_complete(self):
        """發布批次執行完成信號（仍保留原有 YOLO 觸發用途）"""
        msg = Bool()
        msg.data = True
        self.batch_complete_pub.publish(msg)
        self.get_logger().info("批次執行完成，已通知 YOLO 進行推論")
    
    def startup_sequence(self):
        """
        啟動序列：
        - 執行 G28 回待命位置
        - 初始化夾爪狀態為打開（W=0.0）
        """
        if self.startup_completed:
            return
        
        self.get_logger().info("開始啟動序列...")
        
        if self.execute_g28_motion():
            self.get_logger().info("Done!")
            self.control_finger(0.0)
            self.startup_completed = True
            self.hold_mode = True
        else:
            self.get_logger().error("啟動時 G28 失敗，請檢查運動範圍或參數")
        
        self.startup_timer.cancel()
    
    def control_loop(self):
        """主控制迴路：在 hold_mode 時持續輸出當前關節狀態"""
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
            except Exception:
                pass


if __name__ == '__main__':
    main()
