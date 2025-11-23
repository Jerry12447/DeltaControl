"""
整合YOLO辨識與疏苗算法
單一節點處理：影像輸入 → YOLO辨識 → 疏苗算法 → 結果發布
"""

import rclpy
import numpy as np
import cv2
import os
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from typing import List, Dict, Tuple

from yolo.utils import ThinningUtils, create_plant_data, extract_coords_from_plant_data, extract_confidences_from_plant_data


class IntegratedThinningNode(Node):
    """整合YOLO辨識與疏苗算法的節點"""
    
    def __init__(self):
        super().__init__("integrated_thinning_node")
        
        # 宣告參數
        self._declare_parameters()
        
        # 初始化YOLO模型
        self._init_yolo_model()
        
        # 初始化疏苗工具
        self._init_thinning_utils()
        
        # 初始化變數
        self.bridge = CvBridge()
        
        # 建立訂閱者和發布者
        self._setup_ros_interface()
        
        self.get_logger().info("整合疏苗節點已啟動（觸發模式）")
        self.get_logger().info("等待批次完成信號觸發推論...")
    
    def _declare_parameters(self):
        """宣告ROS2參數"""
        # YOLO參數
        self.declare_parameter('model_path', 'models/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_class', 'Crop')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('verbose', False)
        
        # 疏苗參數
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('min_plant_distance_mm', 150.0)
        self.declare_parameter('pixel_to_mm_ratio', 0.9864)
        
        # 影像參數
        self.declare_parameter('input_topic', '/rgb')
    
    def _init_yolo_model(self):
        """初始化YOLO模型"""
        # 讀取YOLO參數
        model_path_param = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.device = self.get_parameter('device').value
        self.verbose = self.get_parameter('verbose').value
        
        # 載入YOLO模型
        pkg_share = get_package_share_directory('yolo')
        model_path = os.path.join(pkg_share, model_path_param)
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"成功載入YOLO模型")
        except Exception as e:
            self.get_logger().error(f"無法載入YOLO模型: {e}")
            raise
    
    def _init_thinning_utils(self):
        """初始化疏苗工具"""
        # 讀取疏苗參數
        min_plant_distance_mm = self.get_parameter('min_plant_distance_mm').value
        pixel_to_mm_ratio = self.get_parameter('pixel_to_mm_ratio').value
        
        # 計算像素距離
        min_plant_distance_pixels = min_plant_distance_mm / pixel_to_mm_ratio
        
        # 初始化疏苗工具
        self.thinning_utils = ThinningUtils(min_plant_distance=min_plant_distance_pixels)
        
        # 儲存參數
        self.min_confidence = self.get_parameter('min_confidence').value
        self.min_plant_distance_pixels = min_plant_distance_pixels
        self.pixel_to_mm_ratio = pixel_to_mm_ratio
        
        self.get_logger().info(f"疏苗參數: 最小株距={min_plant_distance_mm}mm, 像素距離={min_plant_distance_pixels:.2f}px")
    
    def _setup_ros_interface(self):
        """設置ROS2介面"""
        # 影像緩存變數
        self.current_image = None
        self.image_available = False
        
        self.batch_complete_sub = self.create_subscription(
            Bool, '/batch_execution_complete', self.batch_complete_callback, 10)
        
        input_topic = self.get_parameter('input_topic').value
        self.image_subscriber = self.create_subscription(
            Image, input_topic, self.image_callback, 10)
        
        # 發布移除座標
        self.removal_publisher = self.create_publisher(
            Float32MultiArray, 'removed_cords', 10)
        # 發布保留座標
        self.keep_publisher = self.create_publisher(
            Float32MultiArray, 'thinning/keep_coords', 10)
    
    def image_callback(self, msg):
        """影像回調：僅緩存最新影像"""
        self.current_image = msg
        self.image_available = True
        # 不進行推論，僅更新影像緩存
    
    def batch_complete_callback(self, msg):
        """批次完成回調：觸發疏苗推論"""
        if msg.data and self.image_available:
            self.get_logger().info("收到批次完成信號，開始疏苗推論")
            self._run_thinning_pipeline()
        elif not self.image_available:
            self.get_logger().warn("沒有可用影像，跳過疏苗推論")
        else:
            self.get_logger().info("收到批次完成信號，但信號為False")
    
    def _run_thinning_pipeline(self):
        """執行完整的疏苗流程"""
        try:
            # 1. 轉換影像格式
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, "bgr8")
            
            # 2. YOLO辨識
            detected_plants = self._run_yolo_detection(cv_image)
            
            if not detected_plants:
                self.get_logger().warn("未偵測到植株")
                return
            
            # 3. 疏苗算法
            keep_plants, remove_plants = self._run_thinning_algorithm(detected_plants)
            
            # 4. 發布結果
            self._publish_results(keep_plants, remove_plants)
            
        except Exception as e:
            self.get_logger().error(f"疏苗流程錯誤: {e}")
    
    def _run_yolo_detection(self, image):
        """執行YOLO辨識"""
        # YOLO推論
        results = self.model(image, verbose=self.verbose, 
                           conf=self.confidence_threshold, 
                           iou=self.iou_threshold)
        
        detected_plants = []
        names = results[0].names
        
        # 處理偵測結果
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]
            
            if label == self.target_class:
                # 提取座標和信心值
                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                cx = int((xmin + xmax) / 2)
                cy = int((ymin + ymax) / 2)
                conf = float(box.conf[0])
                
                # 創建植株資料
                plant_data = create_plant_data(
                    coords=[cx, cy],
                    confidence=conf,
                    plant_id=len(detected_plants)
                )
                detected_plants.append(plant_data)
        
        self.get_logger().info(f"偵測到 {len(detected_plants)} 株植株")
        return detected_plants
    
    def _run_thinning_algorithm(self, detected_plants):
        """執行疏苗算法"""
        # 1. 信心值過濾
        filtered_plants = self.thinning_utils.filter_plants_by_confidence(
            detected_plants, self.min_confidence)
        
        if not filtered_plants:
            self.get_logger().warn("信心值過濾後無植株")
            return [], detected_plants
        
        # 2. 提取座標和信心值
        plant_coords = extract_coords_from_plant_data(filtered_plants)
        plant_confidences = extract_confidences_from_plant_data(filtered_plants)
        
        # 3. 計算距離矩陣
        distance_matrix = self.thinning_utils.calculate_distance_matrix(plant_coords)
        
        # 4. 找出過密群組
        dense_groups = self.thinning_utils.find_dense_groups(distance_matrix)
        
        # 5. 疏除決策
        keep_indices, remove_indices = self.thinning_utils.make_thinning_decision(
            dense_groups, plant_confidences)
        
        # 6. 分離保留和疏除的植株
        keep_plants = [filtered_plants[i] for i in keep_indices]
        remove_plants = [filtered_plants[i] for i in remove_indices]
        
        self.get_logger().info(f"疏苗結果: 保留 {len(keep_plants)} 株，疏除 {len(remove_plants)} 株")
        return keep_plants, remove_plants
    
    def _publish_results(self, keep_plants, remove_plants):
        """發布疏苗結果"""
        # 發布保留的植株座標
        if keep_plants:
            keep_coords = []
            for plant in keep_plants:
                # 確保座標轉換為浮點數
                coords = [float(coord) for coord in plant['coords']]
                keep_coords.extend(coords)
            
            keep_msg = Float32MultiArray()
            keep_msg.layout.dim.append(MultiArrayDimension(label="group", size=len(keep_plants), stride=len(keep_coords)))
            keep_msg.layout.dim.append(MultiArrayDimension(label="coordinate", size=2, stride=2))
            keep_msg.data = keep_coords
            self.keep_publisher.publish(keep_msg)
            self.get_logger().info(f"發布保留座標: {keep_coords}")
        
        # 發布疏除的植株座標
        if remove_plants:
            remove_coords = []
            for plant in remove_plants:
                # 確保座標轉換為浮點數
                coords = [float(coord) for coord in plant['coords']]
                remove_coords.extend(coords)
            
            remove_msg = Float32MultiArray()
            remove_msg.layout.dim.append(MultiArrayDimension(label="group", size=len(remove_plants), stride=len(remove_coords)))
            remove_msg.layout.dim.append(MultiArrayDimension(label="coordinate", size=2, stride=2))
            remove_msg.data = remove_coords
            self.removal_publisher.publish(remove_msg)
            self.get_logger().info(f"發布疏除座標: {remove_coords}")


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    node = IntegratedThinningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()
