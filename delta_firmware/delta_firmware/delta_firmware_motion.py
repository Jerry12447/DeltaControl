#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Delta Firmware 運動規劃核心
實現笛卡爾空間插值算法和逆運動學計算
對應實際硬體中的Motion.cpp和DeltaKinematics.cpp
"""

import math
import numpy as np
from dataclasses import dataclass
import time
from typing import Tuple, List


@dataclass
class Point:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0

@dataclass
class Angle:
    Theta1: float = 0.0
    Theta2: float = 0.0
    Theta3: float = 0.0


class PositionManager:
    """位置管理類別，管理待命位置和丟棄位置"""
    
    def __init__(self):
        # 待命位置座標 
        self.standby_position = Point(300.0, 300.0, -600.0)
        # 丟棄位置座標 
        self.drop_position = Point(300.0, -300.0, -700.0)
    
    def get_standby_position(self):
        """獲取待命位置"""
        return self.standby_position
    
    def get_drop_position(self):
        """獲取丟棄位置"""
        return self.drop_position
    
    def is_standby_position(self, point):
        """檢查是否為待命位置"""
        return (abs(point.X - self.standby_position.X) < 0.1 and
                abs(point.Y - self.standby_position.Y) < 0.1 and
                abs(point.Z - self.standby_position.Z) < 0.1)
    
    def is_drop_position(self, point):
        """檢查是否為丟棄位置"""
        return (abs(point.X - self.drop_position.X) < 0.1 and
                abs(point.Y - self.drop_position.Y) < 0.1 and
                abs(point.Z - self.drop_position.Z) < 0.1)


class DeltaKinematics:
    """Delta機器人運動學計算（保持與原版本相同）"""
    
    def __init__(self):
        # 三角函數常數
        self.tan30 = 1 / math.sqrt(3)
        self.sin30 = 0.5
        self.cos30 = math.sqrt(3) / 2
        self.tan60 = math.sqrt(3)
        self.sin120 = math.sqrt(3) / 2
        self.cos120 = -0.5
        
        # 機器人參數
        self.RD_RF = 300     # 上臂長度，300mm
        self.RD_RE = 700     # 下臂長度，700mm
        self.RD_F = 334.641  # 固定平台半徑，334.641mm
        self.RD_E = 207.846  # 移動平台半徑，207.846mm
        
        # 計算常用值
        self.RD_RF_Pow2 = self.RD_RF * self.RD_RF
        self.RD_RE_Pow2 = self.RD_RE * self.RD_RE
        self._y1_ = -0.5 * self.tan30 * self.RD_F
        self._y0_ = 0.5 * self.tan30 * self.RD_E

    def angle_theta_calculations(self, x0, y0, z0):
        """計算單個馬達的角度"""
        y0 -= self._y0_
        y1 = self._y1_

        # z = a + b*y
        a = (x0*x0 + y0*y0 + z0*z0 + self.RD_RF_Pow2 - self.RD_RE_Pow2 - y1*y1) / (2.0*z0)
        b = (y1 - y0) / z0

        d = -(a + b*y1)*(a + b*y1) + self.RD_RF_Pow2*(b*b + 1.0)

        if d < 0:
            return None

        yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1.0)
        zj = a + b*yj

        theta = math.degrees(math.atan(-zj/(y1 - yj)))
        if yj > y1:
            theta += 180.0

        return theta

    def inverse_kinematics(self, point):
        """逆運動學：從末端位置計算馬達角度"""
        theta1 = self.angle_theta_calculations(point.X, point.Y, point.Z)
        if theta1 is None:
            return None

        theta2 = self.angle_theta_calculations(
            point.X * self.cos120 + point.Y * self.sin120,
            point.Y * self.cos120 - point.X * self.sin120,
            point.Z
        )
        if theta2 is None:
            return None

        theta3 = self.angle_theta_calculations(
            point.X * self.cos120 - point.Y * self.sin120,
            point.Y * self.cos120 + point.X * self.sin120,
            point.Z
        )
        if theta3 is None:
            return None

        # 配合isaacsim將度數轉換為弧度
        angle = Angle(
            math.radians(theta1),
            math.radians(theta2), 
            math.radians(theta3)
        )
        return angle


class DeltaFirmwareMotion:
    """Delta Firmware 運動規劃核心
    實現笛卡爾空間插值算法和逆運動學計算
    對應實際硬體中的Motion.cpp和DeltaKinematics.cpp
    """
    
    def __init__(self):
        self.kinematics = DeltaKinematics()
        self.position_manager = PositionManager()
        
        # 硬體參數（參考實際韌體）
        self.mm_per_linear_segment = 1.0  # 線性插值每段長度（mm）
        self.mm_per_arc_segment = 1.0     # 圓弧插值每段長度（mm）
        
        # 虛擬環境參數
        self.control_frequency = 100.0    # 控制頻率決定軌跡平滑程度
        self.max_velocity = 50.0          # 最大速度 mm/s
        
        # 夾爪控制參數
        self.current_gripper_angle = 0.0  # 當前夾爪角度（度）
        
        # 當前狀態
        self.current_point = Point(0.0, 0.0, -650.0)  # 當前位置
        self.current_angle = Angle(
            math.radians(-38.0), 
            math.radians(-38.0), 
            math.radians(-38.0)
        )
        
        # 角度限制（保持與原版本相同）
        self.min_angle_deg = -38.05
        self.max_angle_deg = 90.05
    
    def check_angle_limits(self, angle):
        """檢查角度是否在限制範圍內"""
        theta1_deg = math.degrees(angle.Theta1)
        theta2_deg = math.degrees(angle.Theta2)
        theta3_deg = math.degrees(angle.Theta3)
        
        if (theta1_deg < self.min_angle_deg or theta1_deg > self.max_angle_deg or
            theta2_deg < self.min_angle_deg or theta2_deg > self.max_angle_deg or
            theta3_deg < self.min_angle_deg or theta3_deg > self.max_angle_deg):
            return False
        return True
    
    def calculate_cartesian_distance(self, point1, point2):
        """計算兩點間的笛卡爾距離"""
        x_offset = point1.X - point2.X
        y_offset = point1.Y - point2.Y
        z_offset = point1.Z - point2.Z
        
        distance = math.sqrt(x_offset*x_offset + y_offset*y_offset + z_offset*z_offset)
        
        # 如果距離很小，設為0
        if distance < 0.2 and distance > -0.2:
            distance = 0
            
        return distance
    
    def linear_interpolate(self, start_point, end_point, t):
        """線性插值計算中間點"""
        interpolated_point = Point()
        interpolated_point.X = start_point.X - ((start_point.X - end_point.X) * t)
        interpolated_point.Y = start_point.Y - ((start_point.Y - end_point.Y) * t)
        interpolated_point.Z = start_point.Z - ((start_point.Z - end_point.Z) * t)
        return interpolated_point
    
    
    def g1_motion(self, x, y, z, velocity=None):
        """G1線性移動"""
        if velocity is None:
            velocity = self.max_velocity
            
        target_point = Point(x, y, z)
        return self.linear_interpolation_motion(self.current_point, target_point, velocity)
    
    def m0_motion(self):
        """M0指令：開啟夾爪（0度）"""
        self.current_gripper_angle = 0.0
        return True
    
    def m1_motion(self):
        """M1指令：關閉夾爪（45度）"""
        self.current_gripper_angle = 45.0
        return True
    
    def linear_interpolation_motion(self, start_point, target_point, velocity):
        """線性插值運動"""
        # 1. 計算笛卡爾距離
        distance = self.calculate_cartesian_distance(start_point, target_point)
        
        if distance == 0:
            return True
        
        # 2. 計算逆運動學檢查目標點是否可達
        target_angle = self.kinematics.inverse_kinematics(target_point)
        if target_angle is None:
            return False
        
        if not self.check_angle_limits(target_angle):
            return False
        
        # 3. 根據距離和每段長度計算分段數（參考實際韌體）
        number_segments = int(distance / self.mm_per_linear_segment)
        if number_segments < 1:
            number_segments = 1
        
        # 4. 計算每段的實際長度
        mm_per_seg = distance / number_segments
        
        # 5. 生成軌跡點序列
        trajectory_points = []
        trajectory_angles = []
        
        last_angle = self.current_angle
        
        for i in range(1, number_segments + 1):
            t = i / number_segments
            # 線性插值計算中間點
            interpolated_point = self.linear_interpolate(start_point, target_point, t)
            
            # 計算逆運動學
            current_angle = self.kinematics.inverse_kinematics(interpolated_point)
            if current_angle is None:
                return False
            
            if not self.check_angle_limits(current_angle):
                return False
            
            trajectory_points.append(interpolated_point)
            trajectory_angles.append(current_angle)
            last_angle = current_angle
        
        # 6. 更新當前位置和角度
        self.current_point = target_point
        self.current_angle = target_angle
        
        return trajectory_points, trajectory_angles
    
    def execute_trajectory_with_timing(self, trajectory_points, trajectory_angles, velocity):
        """執行軌跡並控制時間（結合虛擬環境的200Hz控制）"""
        if not trajectory_points or not trajectory_angles:
            return False
        
        # 計算總時間
        total_distance = 0
        for i in range(len(trajectory_points) - 1):
            total_distance += self.calculate_cartesian_distance(
                trajectory_points[i], trajectory_points[i + 1]
            )
        
        total_time = total_distance / velocity
        dt = 1.0 / self.control_frequency  
        
        # 生成時間序列的軌跡點
        time_sequence = np.linspace(0, total_time, int(total_time * self.control_frequency) + 1)
        
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
                    current_angle = self.interpolate_angles(
                        trajectory_angles[segment_index], 
                        trajectory_angles[segment_index + 1], 
                        segment_t
                    )
            
            # 這裡可以添加發布關節狀態的邏輯
            # self.publish_joint_state(current_angle)
            
            # 控制頻率
            time.sleep(dt)
        
        return True
    
    def interpolate_angles(self, angle1, angle2, t):
        """在兩個角度之間進行線性插值"""
        interpolated_angle = Angle()
        interpolated_angle.Theta1 = angle1.Theta1 + (angle2.Theta1 - angle1.Theta1) * t
        interpolated_angle.Theta2 = angle1.Theta2 + (angle2.Theta2 - angle1.Theta2) * t
        interpolated_angle.Theta3 = angle1.Theta3 + (angle2.Theta3 - angle1.Theta3) * t
        return interpolated_angle
    
    def move_to_standby_position(self, velocity=None):
        """移動到待命位置"""
        if velocity is None:
            velocity = self.max_velocity
            
        standby_point = self.position_manager.get_standby_position()
        return self.g1_motion(standby_point.X, standby_point.Y, standby_point.Z, velocity)
    
    def move_to_drop_position(self, velocity=None):
        """移動到丟棄位置"""
        if velocity is None:
            velocity = self.max_velocity
            
        drop_point = self.position_manager.get_drop_position()
        return self.g1_motion(drop_point.X, drop_point.Y, drop_point.Z, velocity)
    
    def get_current_position(self):
        """獲取當前位置"""
        return self.current_point
    
    def get_current_angle(self):
        """獲取當前角度"""
        return self.current_angle
    
    def get_current_gripper_angle(self):
        """獲取當前夾爪角度"""
        return self.current_gripper_angle
