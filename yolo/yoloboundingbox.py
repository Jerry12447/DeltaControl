import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloV8CropOnceNode(Node):
    def __init__(self):
        super().__init__('yolov8_crop_once_node')

        # 初始化 YOLOv8 模型
        self.model = YOLO("./yolo/best.pt")

        # 建立 cv_bridge
        self.bridge = CvBridge()

        # 建立發布者
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/removed_cords', 10)

        # 訂閱 /rgb 話題
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10)

        # 旗標：只跑一次
        self.processed = False

    def listener_callback(self, msg):
        if self.processed:
            return   # 已處理過就不再執行

        # ROS2 Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 進行 YOLO 推論 (關閉 log)
        results = self.model(frame, verbose=False)

        names = results[0].names
        coords_list = []

        # 篩選 "Crop" 類別
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]
            if label == "Crop":
                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                cx = int((xmin + xmax) / 2)
                cy = int((ymin + ymax) / 2)
                coords_list.extend([cx, cy])  

        # 建立訊息
        msg_out = UInt16MultiArray()

        # 設定 layout
        group_size = len(coords_list) // 2  # 有多少組 (一組是 (cx, cy))
        msg_out.layout.dim.append(MultiArrayDimension(label="group", size=group_size, stride=len(coords_list)))
        msg_out.layout.dim.append(MultiArrayDimension(label="coordinate", size=2, stride=2))
        msg_out.layout.data_offset = 0

        # 設定 data
        msg_out.data = coords_list

        # 發布結果
        self.publisher_.publish(msg_out)
        self.get_logger().info(f"已發布 Crop 中心點到 /removed_cords: {coords_list}")

        # 設為已處理
        self.processed = True


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8CropOnceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
