#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading
from queue import Queue


class DeltaFirmwareAPI(Node):
    
    def __init__(self):
        super().__init__('delta_firmware_api')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/delta_x/target_array',
            self.coordinate_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.g1_pub = self.create_publisher(
            Float32MultiArray,
            '/delta_firmware/g1',
            10
        )

        self.g28_pub = self.create_publisher(
            Float32MultiArray,
            '/delta_firmware/g28',
            10
        )
        self.standby_position = (397.53, 195.66, -700.0)   # 待命位置
        self.drop_position = (207.4, -380.5, -700.0)     # 丟棄位置
        
        # 控制參數
        self.default_velocity = 200.0  # 預設速度 mm/s
        
        # 批次佇列管理
        self.batch_queue = Queue()  
        self.command_wait_event = threading.Event()  
        self.last_command_success = True  
        
        self.command_complete_sub = self.create_subscription(
            Bool,
            '/delta_firmware/command_complete',
            self.command_complete_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 批次處理執行緒
        self.is_processing = False
        self.processing_thread = threading.Thread(
            target=self.process_batch_queue,
            daemon=True
        )
        self.processing_thread.start()
        self.is_processing = True
        
        self.get_logger().info("Delta Firmware API 已啟動")
        self.get_logger().info(f"預設速度: {self.default_velocity} mm/s")
        self.get_logger().info("批次處理執行緒已啟動")
    
    # ==============================
    # 座標序列處理
    # ==============================
    def coordinate_callback(self, msg):
        """處理座標轉換結果，將批次加入佇列由執行緒處理"""
        if len(msg.data) % 3 != 0:
            self.get_logger().error("座標數據格式錯誤：點的數量必須是 3 的倍數")
            return
        
        # 解析座標點
        points = []
        for i in range(0, len(msg.data), 3):
            point = {
                'x': msg.data[i],
                'y': msg.data[i + 1],
                'z': msg.data[i + 2]
            }
            points.append(point)
        
        self.get_logger().info(f"收到 {len(points)} 個座標點，加入批次佇列")
        self.batch_queue.put(points)
        self.get_logger().info(f"批次已加入佇列，當前佇列大小: {self.batch_queue.qsize()}")
    
    def command_complete_callback(self, msg: Bool):
        """接收指令執行完成狀態"""
        self.last_command_success = msg.data
        self.command_wait_event.set()  
    
    def process_batch_queue(self):
        """執行緒處理批次佇列，避免阻塞主執行緒"""
        self.get_logger().info("批次處理執行緒已啟動")
        
        loop_count = 0
        while self.is_processing:
            try:
                loop_count += 1
                if loop_count % 100 == 0:
                    queue_size = self.batch_queue.qsize()
                    self.get_logger().info(f"批次處理執行緒運行中，佇列大小: {queue_size}")
                if not self.batch_queue.empty():
                    points = self.batch_queue.get(timeout=1.0)
                    self.get_logger().info(f"開始處理批次，包含 {len(points)} 個座標點")
                    
                    # 處理批次
                    success = self.execute_batch(points)
                    
                    if success:
                        self.get_logger().info("批次處理完成")
                    else:
                        self.get_logger().error("批次處理失敗")
                    
                    # 標記任務完成
                    self.batch_queue.task_done()
                else:
                    time.sleep(0.1)  
                    
            except Exception as e:
                self.get_logger().error(f"批次處理執行緒錯誤: {e}")
                time.sleep(0.1)
        
        self.get_logger().info("批次處理執行緒已停止")
    
    def execute_batch(self, points):
        """執行批次序列：每組完成後移動到丟棄位置，批次完成後返回待命位置"""
        if len(points) % 3 != 0:
            self.get_logger().error(f"Invalid point count: {len(points)} must be multiple of 3")
            return False
        
        num_targets = len(points) // 3
        
        if num_targets == 0:
            self.get_logger().warn("沒有目標點，跳過處理")
            return False
        
        success_count = 0
        
        for t in range(num_targets):
            # Extract 3 points for this target
            group = points[t * 3:(t + 1) * 3]
            
            self.get_logger().info(f"處理目標組 {t + 1}/{num_targets}")
            
            # 1) First Z (upper layer) → OPEN gripper
            p0 = group[0]
            if not self.send_g1_and_wait(p0['x'], p0['y'], p0['z'], 0, "上方 (開夾)"):
                self.get_logger().error(f"目標組 {t + 1} 的第一個點執行失敗")
                continue
            
            # 2) Second Z (down layer) → CLOSE gripper 
            p1 = group[1]
            if not self.send_g1_and_wait(p1['x'], p1['y'], p1['z'], 0, "下降抓取 (關夾)"):
                self.get_logger().error(f"目標組 {t + 1} 的第二個點執行失敗")
                continue
            
            # 3) Third Z (upper layer again) 
            p2 = group[2]
            if not self.send_g1_and_wait(p2['x'], p2['y'], p2['z'], 1, "上升 (保持關夾)"):
                self.get_logger().error(f"目標組 {t + 1} 的第三個點執行失敗")
                continue
            
            # 每組完成後移動到丟棄位置並開夾
            if not self.send_g1_and_wait(
                self.drop_position[0],
                self.drop_position[1],
                self.drop_position[2],
                1, 
                f"丟棄位置 (開夾) - 目標組 {t + 1}"
            ):
                self.get_logger().error(f"目標組 {t + 1} 移動到丟棄位置失敗")
                continue
            
            success_count += 1
            self.get_logger().info(f"目標組 {t + 1} 處理完成")
        
        # 批次完成後返回待命位置
        self.get_logger().info("所有目標組處理完成，返回待命位置")
        if self.send_g1_and_wait(
            self.standby_position[0],
            self.standby_position[1],
            self.standby_position[2],
            0,  
            "回到待命位置"
        ):
            self.get_logger().info(f"批次執行完成！成功處理 {success_count}/{num_targets} 個目標組")
            return True
        else:
            self.get_logger().error("返回待命位置失敗")
            return False
    
    def send_g1_and_wait(self, x, y, z, gripper_state, action_name):
        """發送 G1 指令並等待執行完成"""
        self.command_wait_event.clear()
        if gripper_state is None:
            self.send_g1_command(x, y, z, action_name)
        else:
            self.send_g1_and_m_commands(x, y, z, gripper_state, action_name)
        
        # 等待執行完成（最多等待 30 秒）
        if self.command_wait_event.wait(timeout=30.0):
            return self.last_command_success
        else:
            self.get_logger().error(f"{action_name} 執行超時")
            return False
        
    # ==============================
    # 低階指令封裝
    # ==============================
    def send_g1_and_m_commands(self, x, y, z, gripper_state, action_name):
        """
        發送 G1 指令，格式與原固件一致: [x, y, z, velocity, w]
        w: 夾爪角度（度），0.0=開夾, 33.0=關夾
        """
        # 將 gripper_state (0/1) 轉換為角度值
        w_angle = 0.0 if gripper_state == 0 else 25.0
        
        g1_command = Float32MultiArray()
        g1_command.data = [x, y, z, self.default_velocity, w_angle]
        
        self.g1_pub.publish(g1_command)
        self.get_logger().debug(f"{action_name}: G1({x:.2f}, {y:.2f}, {z:.2f}) W{w_angle:.1f}")
    
    def send_g1_command(self, x, y, z, action_name):
        """
        只發送 G1 指令（不改變夾爪狀態，W=None 表示使用當前狀態）
        G1 指令格式: [x, y, z, velocity, w]
        w=None 時表示不改變夾爪狀態
        """
        g1_command = Float32MultiArray()
        # W=None 用於表示不改變夾爪狀態，在 Control 層會檢查
        g1_command.data = [x, y, z, self.default_velocity, float('nan')]
        
        self.g1_pub.publish(g1_command)
        self.get_logger().debug(f"{action_name}: G1({x:.2f}, {y:.2f}, {z:.2f})")

    def send_g28_command(self):
        """
        發送 G28 指令（回待命位置）。
        """
        msg = Float32MultiArray()
        msg.data = []
        self.g28_pub.publish(msg)
        self.get_logger().info("Send G28")
    
    # ==============================
    # 速度設定
    # ==============================
    def set_default_velocity(self, velocity):
        """設定預設速度"""
        self.default_velocity = float(velocity)
        self.get_logger().info(f"Set speed to {velocity} mm/s")


def main(args=None):
    rclpy.init(args=args)
    node = DeltaFirmwareAPI()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中斷信號，正在關閉...")
    finally:
        # 停止批次處理執行緒
        node.is_processing = False
        
        # 等待佇列清空
        if not node.batch_queue.empty():
            node.get_logger().info("等待剩餘批次處理完成...")
            node.batch_queue.join()
        
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
