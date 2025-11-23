#!/usr/bin/env python3
"""
單幀影像辨識工具
訂閱影像話題，取一張frame進行YOLO辨識，計算辨識框中心點
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np


class SingleFrameDetectionNode(Node):
    """單幀影像辨識節點"""
    
    def __init__(self):
        super().__init__('single_frame_detection_node')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 宣告參數
        self.declare_parameter('model_path', 'models/best1.pt')
        self.declare_parameter('confidence_threshold', 0.1)
        self.declare_parameter('iou_threshold', 0.7)
        self.declare_parameter('target_class', '')  # 空字串表示不限制類別
        self.declare_parameter('input_topic', '/rgb')
        self.declare_parameter('device', 'cpu')
        
        # 讀取參數
        self._load_parameters()
        
        # 載入YOLO模型
        self._load_yolo_model()
        
        # 訂閱影像話題
        input_topic = self.get_parameter('input_topic').value
        self.image_subscriber = self.create_subscription(
            Image, input_topic, self.image_callback, 10)
        
        # 標誌：是否已處理過一幀
        self.frame_processed = False
        
        self.get_logger().info(f"單幀辨識節點已啟動，訂閱話題: {input_topic}")
        if self.target_class:
            self.get_logger().info(f"目標類別限制: {self.target_class}")
        else:
            self.get_logger().info("不限制類別，將處理所有辨識結果")
        self.get_logger().info("等待接收第一幀影像...")
    
    def _load_parameters(self):
        """載入參數"""
        model_path_param = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.device = self.get_parameter('device').value
        
        # 獲取功能包路徑
        pkg_share = get_package_share_directory('yolo')
        self.model_path = os.path.join(pkg_share, model_path_param)
    
    def _load_yolo_model(self):
        """載入YOLO模型"""
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"成功載入YOLO模型: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"無法載入YOLO模型: {e}")
            raise
    
    def image_callback(self, msg):
        """影像回調：只處理第一幀"""
        if self.frame_processed:
            return  # 已處理過，不再處理
        
        try:
            # 標記為已處理
            self.frame_processed = True
            
            # 轉換影像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.get_logger().info("收到影像，開始辨識...")
            
            # 執行YOLO辨識
            results = self.model(
                cv_image, 
                verbose=False,
                conf=self.confidence_threshold,
                iou=self.iou_threshold
            )
            
            # 處理辨識結果
            detected_plants = self._process_detection_results(results[0], cv_image)
            
            # 顯示結果
            self._display_results(detected_plants)
            
            self.get_logger().info("辨識完成，節點將停止處理新影像")
            
        except Exception as e:
            self.get_logger().error(f"影像處理錯誤: {e}")
            self.frame_processed = False  # 允許重試
    
    def _process_detection_results(self, result, image):
        """處理辨識結果並計算中心點"""
        detected_plants = []
        names = result.names
        
        self.get_logger().info("=== 辨識結果 ===")
        
        for i, box in enumerate(result.boxes):
            cls_id = int(box.cls[0])
            label = names[cls_id]
            conf = float(box.conf[0])
            
            # 如果 target_class 為空字串，則處理所有類別；否則只處理指定類別
            if not self.target_class or label == self.target_class:
                # 獲取邊界框座標
                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                
                # 計算中心點
                center_x = int((xmin + xmax) / 2)
                center_y = int((ymin + ymax) / 2)
                
                # 儲存辨識結果
                plant_info = {
                    'id': len(detected_plants),
                    'label': label,
                    'confidence': conf,
                    'bbox': [int(xmin), int(ymin), int(xmax), int(ymax)],
                    'center': [center_x, center_y]
                }
                detected_plants.append(plant_info)
                
                # 在影像上繪製
                cv2.rectangle(image, 
                            (int(xmin), int(ymin)), 
                            (int(xmax), int(ymax)), 
                            (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(image, f"{label}: {conf:.2f}", 
                          (int(xmin), int(ymin) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, f"({center_x}, {center_y})", 
                          (center_x + 10, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # 儲存標註後的影像
        output_path = '/tmp/detection_result.jpg'
        cv2.imwrite(output_path, image)
        self.get_logger().info(f"標註結果已儲存至: {output_path}")
        
        return detected_plants
    
    def _display_results(self, detected_plants):
        """顯示辨識結果"""
        if not detected_plants:
            self.get_logger().warn("未偵測到目標物件")
            return
        
        self.get_logger().info(f"總共偵測到 {len(detected_plants)} 個物件:")
        self.get_logger().info("=" * 60)
        
        for plant in detected_plants:
            self.get_logger().info(f"物件 {plant['id'] + 1}:")
            self.get_logger().info(f"  類別: {plant['label']}")
            self.get_logger().info(f"  信心值: {plant['confidence']:.3f}")
            self.get_logger().info(f"  邊界框: [{plant['bbox'][0]}, {plant['bbox'][1]}, {plant['bbox'][2]}, {plant['bbox'][3]}]")
            self.get_logger().info(f"  中心點: ({plant['center'][0]}, {plant['center'][1]})")
            self.get_logger().info("-" * 60)
        
        # 輸出中心點列表（方便複製）
        centers = [plant['center'] for plant in detected_plants]
        self.get_logger().info(f"中心點列表: {centers}")


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    node = SingleFrameDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中斷信號")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            pass


if __name__ == '__main__':
    main()
