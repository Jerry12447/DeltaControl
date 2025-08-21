#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控制模組
整合軌跡規劃和角度控制功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import math
from typing import List, Optional

# 將相對導入改為絕對導入
from trajectory_planner import TrajectoryPlanner
from angle_controller import AngleController

class MainController(Node):
    """主控制器"""
    
    def __init__(self):
        super().__init__('main_controller')
        
        # 初始化組件
        self.trajectory_planner = TrajectoryPlanner(frequency=200.0)
        self.angle_controller = AngleController(control_frequency=200.0)
        
        # 創建服務
        self.start_service = self.create_service(
            Trigger, 
            '/start_trajectory', 
            self.start_trajectory_callback
        )
        
        self.stop_service = self.create_service(
            Trigger, 
            '/stop_trajectory', 
            self.stop_trajectory_callback
        )
        
        # 創建參數設定服務
        self.set_params_service = self.create_service(
            Trigger,
            '/set_control_params',
            self.set_control_params_callback
        )
        
        # 控制參數
        self.default_max_velocity = math.pi / 16.0      # 90度/秒
        self.default_max_acceleration = math.pi / 32.0  # 45度/秒²
        
        # 測試軌跡點（使用數學計算轉換為弧度）
        self.test_trajectory = [
            [0.0, 0.0, 0.0],                    # 起始位置
            [math.radians(50.0), math.radians(50.0), math.radians(50.0)],     # 60.0° 60.0° 60.0°
            [math.radians(45.0), math.radians(45.0), math.radians(45.0)],     # 45.0° 30.0° 50.0°
            [math.radians(60.0), math.radians(50.0), math.radians(40.0)],     # 60.0° 50.0° 40.0°
            [math.radians(20.0), math.radians(20.0), math.radians(30.0)],     # 60.0° 60.0° 60.0°
            [math.radians(50.0), math.radians(40.0), math.radians(10.0)],     # 60.0° 60.0° 60.0°
        ]
        
        self.get_logger().info("主控制器已初始化")
        self.get_logger().info("可用的服務:")
        self.get_logger().info("  /start_trajectory - 開始執行測試軌跡")
        self.get_logger().info("  /stop_trajectory - 停止軌跡執行")
        self.get_logger().info("  /set_control_params - 設定控制參數")
    
    def start_trajectory_callback(self, request, response):
        """開始執行軌跡的回調函數"""
        try:
            self.get_logger().info("開始執行測試軌跡")
            
            # 執行測試軌跡
            success = self.execute_test_trajectory()
            
            if success:
                response.success = True
                response.message = "軌跡執行成功"
            else:
                response.success = False
                response.message = "軌跡執行失敗"
                
        except Exception as e:
            response.success = False
            response.message = f"執行軌跡時發生錯誤: {str(e)}"
            self.get_logger().error(f"執行軌跡時發生錯誤: {str(e)}")
        
        return response
    
    def stop_trajectory_callback(self, request, response):
        """停止軌跡執行的回調函數"""
        try:
            self.angle_controller.stop_and_hold()
            response.success = True
            response.message = "軌跡已停止"
            self.get_logger().info("軌跡執行已停止")
            
        except Exception as e:
            response.success = False
            response.message = f"停止軌跡時發生錯誤: {str(e)}"
            self.get_logger().error(f"停止軌跡時發生錯誤: {str(e)}")
        
        return response
    
    def set_control_params_callback(self, request, response):
        """設定控制參數的回調函數"""
        try:
            # 設定預設控制參數
            self.angle_controller.set_control_parameters(
                self.default_max_velocity,
                self.default_max_acceleration
            )
            
            response.success = True
            response.message = "控制參數已設定"
            self.get_logger().info("控制參數已設定")
            
        except Exception as e:
            response.success = False
            response.message = f"設定控制參數時發生錯誤: {str(e)}"
            self.get_logger().error(f"設定控制參數時發生錯誤: {str(e)}")
        
        return response
    
    def execute_test_trajectory(self) -> bool:
        """
        執行測試軌跡
        
        Returns:
            執行是否成功
        """
        try:
            for i, target_angles in enumerate(self.test_trajectory):
                self.get_logger().info(f"執行軌跡點 {i+1}/{len(self.test_trajectory)}: {[f'{angle:.4f}' for angle in target_angles]}")
                
                # 設定目標角度
                self.angle_controller.target_angles = target_angles
                
                # 執行軌跡
                self.angle_controller.execute_trajectory()
                
                # 等待軌跡完成
                while self.angle_controller.is_executing:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.get_logger().info(f"軌跡點 {i+1} 執行完成")
            
            self.get_logger().info("所有軌跡點執行完成")
            return True
            
        except Exception as e:
            self.get_logger().error(f"執行測試軌跡時發生錯誤: {str(e)}")
            return False
    
    def get_status(self) -> dict:
        """獲取系統狀態"""
        return {
            'trajectory_planner': {
                'frequency': self.trajectory_planner.frequency,
                'dt': self.trajectory_planner.dt
            },
            'angle_controller': self.angle_controller.get_current_status(),
            'test_trajectory': self.test_trajectory
        }

def main(args=None):
    rclpy.init(args=args)
    
    controller = MainController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()