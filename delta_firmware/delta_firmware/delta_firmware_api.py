#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class DeltaFirmwareAPI(Node):
    
    def __init__(self):
        super().__init__('delta_firmware_api')
        
        # 創建回調組
        self.callback_group = ReentrantCallbackGroup()
        
        # 訂閱座標轉換結果（原有格式）
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_x/target_array',
            self.coordinate_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 發布G1指令給混合版本控制器
        self.g1_pub = self.create_publisher(
            Float32MultiArray,
            '/delta_firmware/g1',
            10
        )
        
        # 發布M指令給混合版本控制器
        self.m_pub = self.create_publisher(
            Float32MultiArray,
            '/delta_firmware/m',
            10
        )
        
        
        # 移除完成狀態發布相關代碼
        
        # 簡化批次控制
        self.standby_position = (300.0, 300.0, -600.0)  # 待命位置
        
        # 控制參數
        self.default_velocity = 200.0  # 預設速度 100 mm/s
        
        self.get_logger().info("Delta Firmware API 已啟動")
        self.get_logger().info(f"預設速度: {self.default_velocity} mm/s")
    
    def coordinate_callback(self, msg):
        """處理座標轉換結果，轉換為G1指令並發送給底層控制器"""
        if len(msg.data) % 3 != 0:
            self.get_logger().error("座標數據格式錯誤：點的數量必須是3的倍數")
            return
        
        # 解析座標點
        points = []
        for i in range(0, len(msg.data), 3):
            point = {
                'x': msg.data[i],
                'y': msg.data[i+1], 
                'z': msg.data[i+2]
            }
            points.append(point)
        
        self.get_logger().info(f"收到 {len(points)} 個座標點，轉換為G1指令")
        
        # 轉換為G1指令並發送
        self.convert_and_send_g1_commands(points)
    
    def convert_and_send_g1_commands(self, points):
        """簡化版本：根據點數推斷夾爪狀態，只處理丟棄位置"""
        
        # 檢查是否為3的倍數（每個目標點3個動作）
        if len(points) % 3 != 0:
            self.get_logger().error(f"點數 {len(points)} 不是3的倍數，無法處理")
            return
        
        num_targets = len(points) // 3
        
        # 檢查是否有目標點
        if num_targets == 0:
            self.get_logger().warn("沒有目標點，跳過處理")
            return
        
        #self.get_logger().info(f"開始處理 {num_targets} 個目標點")
        
        for target_idx in range(num_targets):
            #self.get_logger().info(f"處理第 {target_idx + 1} 個目標點")
            
            # 處理每個目標的3個動作
            for action_idx in range(3):
                point = points[target_idx * 3 + action_idx]
                x, y, z = point['x'], point['y'], point['z']
                
                # 根據動作索引推斷夾爪狀態
                if action_idx == 0:  # 上方位置
                    gripper_state = 0
                    action_name = "上方位置"
                elif action_idx == 1:  # 目標位置
                    gripper_state = 1
                    action_name = "目標位置"
                elif action_idx == 2:  # 拔起位置
                    gripper_state = 1
                    action_name = "拔起位置"
                
                # 發送指令
                self.send_g1_and_m_commands(x, y, z, gripper_state, action_name)
            
            # 添加丟棄位置（使用固定高度）
            self.send_g1_and_m_commands(300.0, -300.0, -700.0, 0, "丟棄位置")
            
            # 檢查是否為最後一個移除點
            if target_idx == num_targets - 1:
                # 最後一個移除點完成後，移動到待命位置
                self.send_g1_command(self.standby_position[0], self.standby_position[1], self.standby_position[2], "回到待命位置")
        
        #self.get_logger().info(f"所有指令發送完成！處理了 {num_targets} 個目標點")
        
    def send_g1_and_m_commands(self, x, y, z, gripper_state, action_name):
        """發送G1和M指令"""
        # 生成G1指令
        g1_command = Float32MultiArray()
        g1_command.data = [x, y, z, self.default_velocity]
        
        # 生成M指令
        m_command = Float32MultiArray()
        m_command.data = [float(gripper_state)]
        
        # 同步發送
        self.g1_pub.publish(g1_command)
        self.m_pub.publish(m_command)
        
        #self.get_logger().info(f"{action_name}: G1({x}, {y}, {z}) M{gripper_state}")
        time.sleep(0.3)
        
    
    def send_g1_command(self, x, y, z, action_name):
        """發送G1指令（不包含M指令）"""
        g1_command = Float32MultiArray()
        g1_command.data = [x, y, z, self.default_velocity]
        
        self.g1_pub.publish(g1_command)
        self.get_logger().info(f"{action_name}: G1({x}, {y}, {z})")
        time.sleep(0.3)
    
    # 移除完成狀態相關方法
    
    
    def set_default_velocity(self, velocity):
        """設定預設速度"""
        self.default_velocity = velocity
        self.get_logger().info(f"預設速度已設定為: {velocity} mm/s")


def main(args=None):
    rclpy.init(args=args)
    node = DeltaFirmwareAPI()
    
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
