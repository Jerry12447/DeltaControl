#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
軌跡規劃模組
實現均速軌跡規劃，計算總時間和生成高頻位置序列
"""

import numpy as np
import math
from typing import List, Tuple

class TrajectoryPlanner:
    """均速軌跡規劃器"""
    
    def __init__(self, frequency: float = 200.0):
        """
        初始化軌跡規劃器
        
        Args:
            frequency: 控制頻率 (Hz)，預設200Hz
        """
        self.frequency = frequency
        self.dt = 1.0 / frequency  # 時間步長
        
    def wrap_to_pi(self, angle: float) -> float:
        """
        將角度包裝到 [-π, π] 範圍內
        
        Args:
            angle: 輸入角度（弧度）
            
        Returns:
            包裝後的角度（弧度）
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_total_distance(self, theta_goal: float, theta_now: float) -> float:
        """
        計算總角距
        
        Args:
            theta_goal: 目標角度（弧度）
            theta_now: 當前角度（弧度）
            
        Returns:
            總角距（弧度）
        """
        return self.wrap_to_pi(theta_goal - theta_now)
    
    def generate_constant_velocity_trajectory(self, 
                                        theta_start: float, 
                                        theta_end: float, 
                                        velocity: float) -> Tuple[List[float], List[float], float]:
        """
        生成均速軌跡
        
        Args:
            theta_start: 起始角度（弧度）
            theta_end: 結束角度（弧度）
            velocity: 角速度（rad/s）
            
        Returns:
            (位置序列, 時間序列, 總時間)
        """
        # 計算總角距
        total_distance = self.calculate_total_distance(theta_end, theta_start)
        
        # 計算總時間
        total_time = abs(total_distance) / velocity
        
        # 生成時間序列
        num_steps = int(total_time * self.frequency) + 1
        time_sequence = np.linspace(0, total_time, num_steps)
        
        # 生成均速位置序列
        position_sequence = []
        for t in time_sequence:
            if t <= 0:
                pos = theta_start
            elif t >= total_time:
                pos = theta_end
            else:
                # 均速運動
                pos = theta_start + (total_distance * t / total_time)
            position_sequence.append(pos)
        
        return position_sequence, time_sequence.tolist(), total_time 