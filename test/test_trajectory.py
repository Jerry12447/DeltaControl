#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
測試腳本
測試軌跡規劃和角度控制功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import math
import time

# 將相對導入改為絕對導入
from trajectory_planner import TrajectoryPlanner
from angle_controller import AngleController

class TrajectoryTester(Node):
    """軌跡測試器"""
    
    def __init__(self):
        super().__init__('trajectory_tester')
        
        # 創建客戶端
        self.start_client = self.create_client(Trigger, '/start_trajectory')
        self.stop_client = self.create_client(Trigger, '/stop_trajectory')
        self.params_client = self.create_client(Trigger, '/set_control_params')
        
        # 創建發布者
        self.target_pub = self.create_publisher(
            Float32MultiArray,
            '/target_angles',
            10
        )
        
        self.get_logger().info("軌跡測試器已初始化")
        self.get_logger().info("等待服務可用...")
        
        # 等待服務可用
        while not (self.start_client.wait_for_service(timeout_sec=1.0) and
                  self.stop_client.wait_for_service(timeout_sec=1.0) and
                  self.params_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info("等待服務...")
        
        self.get_logger().info("所有服務已可用")
    
    def test_single_angle(self, target_angles: list):
        """
        測試單個角度設定
        
        Args:
            target_angles: 目標角度列表 [θ1, θ2, θ3]
        """
        self.get_logger().info(f"測試目標角度: {[f'{angle:.4f}' for angle in target_angles]}")
        
        # 發布目標角度
        msg = Float32MultiArray()
        msg.data = target_angles
        self.target_pub.publish(msg)
        
        # 等待執行
        time.sleep(2.0)
    
    def test_trajectory_service(self):
        """測試軌跡服務"""
        self.get_logger().info("測試軌跡服務...")
        
        # 設定控制參數
        request = Trigger.Request()
        future = self.params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("控制參數設定成功")
        else:
            self.get_logger().error(f"控制參數設定失敗: {future.result().message}")
            return False
        
        # 開始執行軌跡
        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("軌跡執行成功")
        else:
            self.get_logger().error(f"軌跡執行失敗: {future.result().message}")
            return False
        
        return True
    
    def test_stop_service(self):
        """測試停止服務"""
        self.get_logger().info("測試停止服務...")
        
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("停止服務測試成功")
        else:
            self.get_logger().error(f"停止服務測試失敗: {future.result().message}")
            return False
        
        return True
    
    def run_comprehensive_test(self):
        """執行綜合測試"""
        self.get_logger().info("開始執行綜合測試...")
        
        # 測試1: 單個角度設定
        self.get_logger().info("=== 測試1: 單個角度設定 ===")
        test_angles = [0.5, 0.3, 0.4]
        self.test_single_angle(test_angles)
        
        # 測試2: 軌跡服務
        self.get_logger().info("=== 測試2: 軌跡服務 ===")
        if not self.test_trajectory_service():
            self.get_logger().error("軌跡服務測試失敗")
            return False
        
        # 測試3: 停止服務
        self.get_logger().info("=== 測試3: 停止服務 ===")
        if not self.test_stop_service():
            self.get_logger().error("停止服務測試失敗")
            return False
        
        self.get_logger().info("所有測試完成！")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    tester = TrajectoryTester()
    
    try:
        # 執行綜合測試
        success = tester.run_comprehensive_test()
        
        if success:
            tester.get_logger().info("測試成功完成！")
        else:
            tester.get_logger().error("測試失敗！")
        
        # 保持運行一段時間
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        tester.get_logger().info("測試被用戶中斷")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()