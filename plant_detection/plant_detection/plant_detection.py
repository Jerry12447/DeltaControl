from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from .algorithm import ThinningAlgorithm


class PlantDetectionNode(Node):
    def __init__(self):
        super().__init__('plant_detection_node')
        
        # 宣告 ROS2 參數
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('image_topic', '/agri_bot/D455f/color/image_raw')
        
        # 疏除演算法參數
        self.declare_parameter('enable_thinning', True)
        self.declare_parameter('thinning_threshold_cm', 12.0)  # 疏苗距離（公分）
        self.declare_parameter('pixel_to_cm_original', 0.09874)  # 1280*720解析度下的像素到公分轉換
        
        #YOLO模型參數
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        image_topic = self.get_parameter('image_topic').value
        
        # 取得疏除參數
        self.enable_thinning = self.get_parameter('enable_thinning').value
        self.thinning_threshold_cm = self.get_parameter('thinning_threshold_cm').value
        self.pixel_to_cm = self.get_parameter('pixel_to_cm_original').value  # 使用1280*720的轉換比例
        
        # 計算疏除閾值（像素單位）
        self.threshold_px = self.thinning_threshold_cm / self.pixel_to_cm
        
        # 初始化疏苗算法模組
        if self.enable_thinning:
            self.thinning_algorithm = ThinningAlgorithm(threshold_px=self.threshold_px)
        else:
            self.thinning_algorithm = None
        
        # 載入 YOLO 模型
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f'無法載入模型: {str(e)}')
            raise
        
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        # 創建發布者：發布被疏除的作物和所有 weed 的座標
        self.removed_cords_publisher = self.create_publisher(
            Float32MultiArray, '/removed_cords', 10)
        
        # 標誌：是否已經執行過辨識和疏除
        self.processed = False

    def image_callback(self, msg):
        # 如果已經處理過，直接返回
        if self.processed:
            return
        
        try:
            # 轉換 ROS2 Image 訊息為 OpenCV 格式
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 執行 YOLO 辨識（使用模型預設尺寸）
            results = self.model.predict(
                image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                show=True,
                verbose=False
            )
            
            # 提取檢測結果
            detections = self._extract_detections(results)
            
            # 記錄檢測結果
            if detections:
                # 找出所有作物的索引（在 detections 中的位置）
                crop_indices_in_detections = [i for i, d in enumerate(detections) if d["class"] == 0]
                
                kept_count = 0
                removed_count = 0
                evaluation_scores = {}  # 儲存評估分數 {detection_index: score}
                
                if self.enable_thinning and len(crop_indices_in_detections) > 0 and self.thinning_algorithm is not None:
                    crop_detections = [detections[i] for i in crop_indices_in_detections]
                    
                    # 使用疏苗算法模組執行疏除（返回的是 crop_detections 中的索引和評估分數）
                    kept_indices_in_crop, removed_indices_in_crop, crop_evaluation_scores = self.thinning_algorithm.apply_thinning(crop_detections)

                    for idx, crop_idx in enumerate(crop_indices_in_detections):
                        # 映射評估分數（從 crop_detections 的索引映射到 detections 的索引）
                        evaluation_scores[crop_idx] = crop_evaluation_scores.get(idx, 0.0)
                        
                        if idx in kept_indices_in_crop:
                            detections[crop_idx]['thinning_status'] = 'kept'
                            kept_count += 1
                        else:
                            detections[crop_idx]['thinning_status'] = 'removed'
                            removed_count += 1
                
                self._print_results(detections, kept_count, removed_count, evaluation_scores)
                
                
                self._publish_removed_cords(detections)
            else:
                print("No detections found")
            
            self.processed = True
                
        except Exception as e:
            self.get_logger().error(f'圖像處理錯誤: {str(e)}')
            self.processed = True #避免重複嘗試
    
    def _extract_detections(self, yolo_results):
        """
        從 YOLO 結果中提取檢測資訊
        
        :param yolo_results: YOLO 預測結果
        :return: 檢測結果列表，每個元素包含座標、類別、信心度等資訊
        """
        detections = []
        
        for result in yolo_results:
            if result.boxes is not None and len(result.boxes) > 0:
                names = result.names
                
                # 檢查是否有關鍵點輸出
                has_keypoints = hasattr(result, 'keypoints') and result.keypoints is not None
                
                
                for i, box in enumerate(result.boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                    # 計算邊界框中心像素座標（備用中心點）
                    x_center_bbox = int((x1 + x2) / 2)
                    y_center_bbox = int((y1 + y2) / 2)
                    x_center = x_center_bbox
                    y_center = y_center_bbox
                    center_source = 'bbox'  # 標記中心點來源：'keypoint' 或 'bbox'
                    
                    # 嘗試從關鍵點獲取中心點
                    if has_keypoints and result.keypoints is not None:
                        try:
                            if len(result.keypoints) > i:
                                keypoints_obj = result.keypoints[i]
                                keypoint_valid = False
                                kp_x, kp_y = 0, 0
                                if hasattr(keypoints_obj, 'xy'):
                                    kp_xy = keypoints_obj.xy.cpu().numpy()
                                    kp_xy_flat = kp_xy.flatten()
                                    if len(kp_xy_flat) >= 2:
                                        kp_x = int(kp_xy_flat[0])
                                        kp_y = int(kp_xy_flat[1])
                                        keypoint_valid = True
                                
                                #備用 data 屬性
                                if not keypoint_valid and hasattr(keypoints_obj, 'data'):
                                    kp_data = keypoints_obj.data.cpu().numpy()
                                    kp_data_flat = kp_data.flatten()
                                    if len(kp_data_flat) >= 3 and kp_data_flat[2] > 0:  # visibility > 0
                                        kp_x = int(kp_data_flat[0])
                                        kp_y = int(kp_data_flat[1])
                                        keypoint_valid = True
                                    elif len(kp_data_flat) >= 2:  # 沒有 visibility
                                        kp_x = int(kp_data_flat[0])
                                        kp_y = int(kp_data_flat[1])
                                        keypoint_valid = True
                                
                                # 驗證關鍵點是否有效
                                if keypoint_valid:
                                    if (kp_x > 0 and kp_y > 0 and 
                                        x1 - 50 <= kp_x <= x2 + 50 and 
                                        y1 - 50 <= kp_y <= y2 + 50):
                                        x_center = kp_x
                                        y_center = kp_y
                                        center_source = 'keypoint'
                                    else:
                                        self.get_logger().debug(
                                            f"檢測 {i}: 關鍵點座標 ({kp_x}, {kp_y}) 無效，使用邊界框中心"
                                        )
                        except Exception as e:
                            self.get_logger().debug(f"讀取關鍵點時發生錯誤: {str(e)}，使用邊界框中心")
                            # 確保使用邊界框中心作為備用
                            x_center = x_center_bbox
                            y_center = y_center_bbox
                            center_source = 'bbox'
                    
                    w = int(x2 - x1)
                    h = int(y2 - y1)
                    cls = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())
                    
                    detections.append({
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [x_center, y_center],  # 使用的中心像素座標 [x, y]
                        'center_bbox': [x_center_bbox, y_center_bbox],  # 邊界框中心（備用）
                        'center_source': center_source,  # 中心點來源：'keypoint' 或 'bbox'
                        'size': [w, h],
                        'class': cls,
                        'class_name': names[cls] if cls < len(names) else 'unknown',
                        'confidence': conf
                    })
        
        return detections
    '''    
    def _print_results(self, detections, kept_count, removed_count, evaluation_scores=None):
        """
        顯示結果：顯示類別、關鍵點中心和疏除統計
        
        :param detections: 檢測結果列表
        :param kept_count: 保留的作物數量
        :param removed_count: 移除的作物數量
        :param evaluation_scores: 評估分數字典 {detection_index: score}
        """
        # 計算統計資訊
        crop_count = sum(1 for d in detections if d["class"] == 0)
        
        # 顯示每個檢測的詳細資訊
        print("\n=== 檢測結果 ===")
        for i, det in enumerate(detections):
            class_name = det.get('class_name', 'unknown')
            center = det['center']
            center_source = det.get('center_source', 'bbox')
            center_bbox = det.get('center_bbox', center)  # 備用邊界框中心
            thinning_status = det.get('thinning_status', 'N/A')
            
            # 獲取評估分數（如果是作物且有評估分數）
            if det.get("class") == 0 and evaluation_scores is not None and i in evaluation_scores:
                eval_score = evaluation_scores[i]
                # 格式化輸出：類別、座標、來源、評估分數、疏除狀態
                source_info = f"來源={center_source}"
                if center_source == 'keypoint' and center_bbox != center:
                    source_info += f" (備用:({center_bbox[0]}, {center_bbox[1]}))"
                
                if thinning_status != 'N/A':
                    print(f"[{i+1}] {class_name}: 中心點=({center[0]}, {center[1]}), {source_info}, 評估分數={eval_score:.3f}, 狀態={thinning_status}")
                else:
                    print(f"[{i+1}] {class_name}: 中心點=({center[0]}, {center[1]}), {source_info}, 評估分數={eval_score:.3f}")
            else:
                # 非作物或沒有評估分數，顯示信心值
                confidence = det.get('confidence', 0.0)
                source_info = f"來源={center_source}"
                if center_source == 'keypoint' and center_bbox != center:
                    source_info += f" (備用:({center_bbox[0]}, {center_bbox[1]}))"
                print(f"[{i+1}] {class_name}: 中心點=({center[0]}, {center[1]}), {source_info}, 信心值={confidence:.3f}")
        
        # 疏除結果統計
        if self.enable_thinning and crop_count > 0:
            keep_ratio = (kept_count / crop_count) * 100 if crop_count > 0 else 0
            print(f"\n=== 疏除統計 ===")
            print(f"總作物數: {crop_count}")
            print(f"保留: {kept_count}, 移除: {removed_count}, 保留率: {keep_ratio:.1f}%")
            
            # 輸出可移除的作物座標（方便複製）
            removed_crops = [det for det in detections 
                           if det.get("class") == 0 and det.get('thinning_status') == 'removed']
            if removed_crops:
                print(f"\n=== 可移除的作物座標 ===")
                for det in removed_crops:
                    center = det['center']
                    print(f"({center[0]}, {center[1]})")
        
        print("=" * 50)
    '''
    
    def _publish_removed_cords(self, detections: List[dict]):
        """
        發布被疏除的作物和所有 weed 的座標到 /removed_cords topic
        
        Args:
            detections: 檢測結果列表
        """
        removed_coords = []
        
        # 收集被疏除的作物座標
        for det in detections:
            if det.get("class") == 0 and det.get('thinning_status') == 'removed':
                center = det['center']
                removed_coords.append([float(center[0]), float(center[1])])
        
        # 收集所有 weed 的座標
        for det in detections:
            if det.get("class") == 1:  # Weed
                center = det['center']
                removed_coords.append([float(center[0]), float(center[1])])
        
        if removed_coords:
            flat_coords = []
            for coord in removed_coords:
                flat_coords.extend(coord)
            msg = Float32MultiArray()
            msg.layout.dim.append(MultiArrayDimension(
                label="group",
                size=len(removed_coords),
                stride=len(flat_coords)
            ))
            msg.layout.dim.append(MultiArrayDimension(
                label="coordinate",
                size=2,
                stride=2
            ))
            
            msg.data = flat_coords
            self.removed_cords_publisher.publish(msg)
            self.get_logger().info(
                f"發布 {len(removed_coords)} 個需要移除的座標到 /removed_cords"
            )
        else:
            self.get_logger().info("沒有需要移除的座標")

def main(args=None):
    rclpy.init(args=args)
    plant_detection_node = PlantDetectionNode()
    rclpy.spin(plant_detection_node)
  
    plant_detection_node.destroy_node()
    rclpy.shutdown()