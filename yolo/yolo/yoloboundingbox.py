import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import cv2
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class YoloV8CropOnceNode(Node):
    def __init__(self):
        super().__init__('yolov8_crop_once_node')

        # 宣告參數
        self.declare_parameter('model_path', 'models/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_class', 'Crop')
        self.declare_parameter('input_topic', '/rgb')
        self.declare_parameter('output_topic', '/removed_cords')
        self.declare_parameter('execution_complete_topic', 'delta_execution_complete')
        self.declare_parameter('verbose', False)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('save_images', True)
        self.declare_parameter('data_folder', 'data')

        # 讀取參數
        model_path_param = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.target_class = self.get_parameter('target_class').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        execution_topic = self.get_parameter('execution_complete_topic').value
        self.verbose = self.get_parameter('verbose').value
        self.device = self.get_parameter('device').value
        self.save_images = self.get_parameter('save_images').value
        self.data_folder = self.get_parameter('data_folder').value

        # 獲取功能包路徑並構建模型完整路徑
        pkg_share = get_package_share_directory('yolo')
        model_path = os.path.join(pkg_share, model_path_param)
        
        # 載入 YOLO 模型
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"成功載入 YOLO 模型: {model_path}")
        except Exception as e:
            self.get_logger().error(f"無法載入 YOLO 模型: {e}")
            raise

        # 建立 cv_bridge
        self.bridge = CvBridge()

        # 建立發布者
        self.publisher_ = self.create_publisher(UInt16MultiArray, output_topic, 10)

        # 訂閱輸入影像話題
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10)

        # 訂閱執行完成話題
        self.execution_subscription = self.create_subscription(
            Bool,
            execution_topic,
            self.execution_complete_callback,
            10)

        # 旗標：是否需要執行推論
        self.need_inference = True  # 啟動時等待第一次觸發
        
        # 建立數據儲存目錄
        if self.save_images:
            self.data_path = os.path.join(pkg_share, self.data_folder)
            os.makedirs(self.data_path, exist_ok=True)
            self.get_logger().info(f"圖片儲存目錄: {self.data_path}")

        self.get_logger().info(f"YOLO 節點已啟動，監聽話題: {input_topic}, 發布話題: {output_topic}")

    def process_image(self, frame):
        """處理影像並進行推論的通用函數"""
        # 進行 YOLO 推論
        results = self.model(frame, verbose=self.verbose, conf=self.confidence_threshold, iou=self.iou_threshold)

        names = results[0].names
        coords_list = []

        # 篩選目標類別並繪製檢測框
        annotated_frame = frame.copy()
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]
            if label == self.target_class:
                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                cx = int((xmin + xmax) / 2)
                cy = int((ymin + ymax) / 2)
                coords_list.extend([cx, cy])
                
                # 繪製檢測框
                cv2.rectangle(annotated_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(annotated_frame, f"{label}: {box.conf[0]:.2f}", 
                           (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)  

        # 建立訊息
        msg_out = UInt16MultiArray()

        # 為符合訂閱格式設定 layout
        group_size = len(coords_list) // 2  # 像素座標組數 
        msg_out.layout.dim.append(MultiArrayDimension(label="group", size=group_size, stride=len(coords_list)))
        msg_out.layout.dim.append(MultiArrayDimension(label="coordinate", size=2, stride=2))
        msg_out.layout.data_offset = 0

        # 設定 data
        msg_out.data = coords_list

        # 發布結果
        self.publisher_.publish(msg_out)
        self.get_logger().info(f"已發布 Crop 中心點到 /removed_cords: {coords_list}")
        
        # 儲存圖片結果
        if self.save_images:
            self.save_annotated_image(annotated_frame, coords_list)
        
        return coords_list

    def image_callback(self, msg):
        """影像回調函數，持續接收影像"""
        # ROS2 Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 檢查是否需要執行推論
        if self.need_inference:
            # 進行推論
            coords_list = self.process_image(frame)
            self.get_logger().info(f"推論結果已發布到 /removed_cords: {coords_list}")
            self.need_inference = False

    def save_annotated_image(self, annotated_frame, coords_list):
        """儲存帶有檢測結果的圖片"""
        try:
            # 生成時間戳記文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 精確到毫秒
            filename = f"yolo_detection_{timestamp}.jpg"
            filepath = os.path.join(self.data_path, filename)
            
            # 儲存圖片
            cv2.imwrite(filepath, annotated_frame)
            
            # 記錄儲存資訊
            coords_str = ", ".join([f"({coords_list[i]}, {coords_list[i+1]})" for i in range(0, len(coords_list), 2)])
            self.get_logger().info(f"已儲存檢測結果圖片: {filename}, 檢測到 {len(coords_list)//2} 個作物, 座標: {coords_str}")
            
        except Exception as e:
            self.get_logger().error(f"儲存圖片失敗: {e}")

    def execution_complete_callback(self, msg):
        """當收到 delta_execution_complete 時觸發推論"""
        if msg.data:
            self.get_logger().info("收到執行完成信號，下次收到影像時進行推論...")
            self.need_inference = True


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8CropOnceNode()  
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中斷信號，正在關閉...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            # 忽略關閉時的錯誤
            pass

if __name__ == '__main__':
    main()