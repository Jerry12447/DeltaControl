from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
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
        self.declare_parameter('hybrid_weight_conflict', 0.6)  # Hybrid 策略：衝突數權重
        self.declare_parameter('hybrid_weight_distance', 0.4)  # Hybrid 策略：距離權重
        
        #YOLO模型參數
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        image_topic = self.get_parameter('image_topic').value
        
        # 取得疏除參數
        self.enable_thinning = self.get_parameter('enable_thinning').value
        self.thinning_threshold_cm = self.get_parameter('thinning_threshold_cm').value
        self.pixel_to_cm = self.get_parameter('pixel_to_cm_original').value  # 使用1280*720的轉換比例
        self.hybrid_weight_conflict = self.get_parameter('hybrid_weight_conflict').value
        self.hybrid_weight_distance = self.get_parameter('hybrid_weight_distance').value
        
        # 計算疏除閾值（像素單位）
        self.threshold_px = self.thinning_threshold_cm / self.pixel_to_cm
        
        # 初始化疏苗算法模組
        if self.enable_thinning:
            self.thinning_algorithm = ThinningAlgorithm(
                threshold_px=self.threshold_px,
                weight_conflict=self.hybrid_weight_conflict,
                weight_distance=self.hybrid_weight_distance
            )
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
                
                # 對作物進行疏除處理
                kept_count = 0
                removed_count = 0
                if self.enable_thinning and len(crop_indices_in_detections) > 0 and self.thinning_algorithm is not None:
                    crop_detections = [detections[i] for i in crop_indices_in_detections]
                    
                    # 使用疏苗算法模組執行疏除（返回的是 crop_detections 中的索引）
                    kept_indices_in_crop, removed_indices_in_crop = self.thinning_algorithm.apply_thinning(crop_detections)
                    
                    # 將結果映射回 detections 列表
                    for idx, crop_idx in enumerate(crop_indices_in_detections):
                        if idx in kept_indices_in_crop:
                            detections[crop_idx]['thinning_status'] = 'kept'
                            kept_count += 1
                        else:
                            detections[crop_idx]['thinning_status'] = 'removed'
                            removed_count += 1
                
                # 終端顯示結果
                self._print_results(detections, kept_count, removed_count)
            else:
                print("No detections found")
            
            # 標記為已處理，後續圖像將不再處理
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
                # 從 YOLO 結果中獲取類別名稱（模型內建）
                names = result.names
                
                # 檢查是否有關鍵點（keypoints）輸出
                has_keypoints = hasattr(result, 'keypoints') and result.keypoints is not None
                
                for i, box in enumerate(result.boxes):
                    # 提取邊界框座標
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                    # 計算邊界框中心像素座標（當沒有關鍵點時使用）
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    
                    # 嘗試從關鍵點獲取中心點
                    if has_keypoints and result.keypoints is not None:
                        try:
                            # YOLO keypoints 是 Keypoints 對象，可以通過索引訪問
                            if len(result.keypoints) > i:
                                keypoints_obj = result.keypoints[i]
                                
                                # 方法1：使用 xy 屬性（推薦，更直接）
                                if hasattr(keypoints_obj, 'xy'):
                                    kp_xy = keypoints_obj.xy.cpu().numpy()
                                    # xy 形狀可能是 [1, 1, 2] 或 [1, 2]，需要處理不同維度
                                    kp_xy_flat = kp_xy.flatten()
                                    if len(kp_xy_flat) >= 2:
                                        x_center = int(kp_xy_flat[0])
                                        y_center = int(kp_xy_flat[1])
                                
                                # 方法2：使用 data 屬性（備用）
                                elif hasattr(keypoints_obj, 'data'):
                                    kp_data = keypoints_obj.data.cpu().numpy()
                                    # data 形狀是 [1, 1, 3] (x, y, visibility)
                                    kp_data_flat = kp_data.flatten()
                                    if len(kp_data_flat) >= 3 and kp_data_flat[2] > 0:  # visibility > 0
                                        x_center = int(kp_data_flat[0])
                                        y_center = int(kp_data_flat[1])
                                    elif len(kp_data_flat) >= 2:  # 沒有 visibility
                                        x_center = int(kp_data_flat[0])
                                        y_center = int(kp_data_flat[1])
                        except Exception as e:
                            self.get_logger().warn(f"讀取關鍵點時發生錯誤: {str(e)}")
                            import traceback
                            self.get_logger().warn(traceback.format_exc())
                    
                    # 提取寬高
                    w = int(x2 - x1)
                    h = int(y2 - y1)
                    # 提取類別和信心度
                    cls = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())
                    
                    detections.append({
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [x_center, y_center],  # 中心像素座標 [x, y]（優先使用關鍵點，無關鍵點時使用邊界框中心）
                        'size': [w, h],
                        'class': cls,
                        'class_name': names[cls] if cls < len(names) else 'unknown',
                        'confidence': conf
                    })
        
        return detections
    
    def _print_results(self, detections, kept_count, removed_count):
        """
        顯示結果：顯示類別、關鍵點中心和疏除統計
        
        :param detections: 檢測結果列表
        :param kept_count: 保留的作物數量
        :param removed_count: 移除的作物數量
        """
        # 計算統計資訊
        crop_count = sum(1 for d in detections if d["class"] == 0)
        
        # 顯示每個檢測的詳細資訊
        print("\n=== 檢測結果 ===")
        for i, det in enumerate(detections):
            class_name = det.get('class_name', 'unknown')
            center = det['center']
            confidence = det.get('confidence', 0.0)
            thinning_status = det.get('thinning_status', 'N/A')
            
            # 格式化輸出：類別、座標、信心值、疏除狀態
            if thinning_status != 'N/A':
                print(f"[{i+1}] {class_name}: 中心點=({center[0]}, {center[1]}), 信心值={confidence:.3f}, 狀態={thinning_status}")
            else:
                print(f"[{i+1}] {class_name}: 中心點=({center[0]}, {center[1]}), 信心值={confidence:.3f}")
        
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
    

def main(args=None):
    rclpy.init(args=args)
    plant_detection_node = PlantDetectionNode()
    rclpy.spin(plant_detection_node)
  
    plant_detection_node.destroy_node()
    rclpy.shutdown()