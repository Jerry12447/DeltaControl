#!/usr/bin/env python3
# =============================================================================
# 使用流程 (How to Use)
#
# 1) 執行節點：
#    $ python3 yolo_rgbd_mean_depth_node.py
#    （需先啟動 Isaac Sim模擬，且要有 /rgb_yolo, /depth_yolo, /camera_info 三個主題）
#
# 2) 觸發一次推論：
#    $ ros2 service call /run_inference std_srvs/srv/Trigger {}
#    - 程式會臨時訂閱 /rgb_yolo, /depth_yolo, /camera_info
#    - 收到一組完整的 RGB + Depth + CameraInfo 後，自動執行 YOLO 偵測
#    - 偵測到的物件會輸出：
#        * ROS 主題：/center_points (geometry_msgs/PointStamped)
#        * JSON 檔：yolo_results_<timestamp>.json
#        * 標註圖：yolo_center_check_<timestamp>.png
#    - 完成後自動退訂主題，釋放資源
#
# 3) 查詢最近一次結果（不用重跑推論）：
#    $ ros2 service call /get_latest_results std_srvs/srv/Trigger {}
#    - 回傳 JSON 格式字串，包含上次所有偵測的中心點與 3D 座標
#
# 4) 輸出路徑：
#    DATA_DIR = /home/stanleyc/IsaacSim_To_Ros2_WS/aftermid_0519/data
#    - 偵測結果與圖片會自動存到這個資料夾
#
# 備註：
# - 每次觸發只會處理一次資料，流程是「觸發 → 推論 → 存檔/發布 → 退訂」
# - 3D 座標 (X, Y, Z) 是相機座標系下的值，如需轉到 world 或 base_link 需再加 TF
# =============================================================================


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import json
import time
import os

# 使用絕對路徑，基於腳本檔案的位置
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "best.pt")                      # 模型檔案路徑
DATA_DIR = os.path.join(SCRIPT_DIR, "data")                           # 資料輸出目錄

class YoloRGBDMeanDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_rgbd_mean_depth_node')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)  

        self.rgb_sub = None
        self.depth_sub = None
        self.caminfo_sub = None

        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.rgb_header = None

        self.pub_center = self.create_publisher(PointStamped, '/center_points', 10)
        self.latest_results = []
        self.waiting_for_data = False

        self.create_service(Trigger, 'run_inference', self.trigger_callback)
        self.create_service(Trigger, 'get_latest_results', self.result_callback)

        self.get_logger().info('🟢 YOLO RGBD MeanDepth Node ready！')

        # 檢查資料夾存在
        os.makedirs(DATA_DIR, exist_ok=True)

    def trigger_callback(self, request, response):
        self.get_logger().info('🔵 開始動態訂閱 RGB/Depth/CameraInfo...')
        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.latest_results = []
        self.waiting_for_data = True

        self.rgb_sub = self.create_subscription(Image, '/rgb', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_callback, 10)
        self.caminfo_sub = self.create_subscription(CameraInfo, '/camera_info', self.caminfo_callback, 10)

        response.success = True
        response.message = "啟動一次感知（資料齊全時自動處理/退訂）"
        return response

    def rgb_callback(self, msg):
        if not self.waiting_for_data:
            return
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.rgb_header = msg.header
        self.check_and_process()

    def depth_callback(self, msg):
        if not self.waiting_for_data:
            return
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.check_and_process()

    def caminfo_callback(self, msg):
        if not self.waiting_for_data:
            return
        self.camera_info = msg
        self.check_and_process()

    def check_and_process(self):
        if self.rgb_img is not None and self.depth_img is not None and self.camera_info is not None:
            self.waiting_for_data = False
            self.process_inference()
            self.unsubscribe_all()

    def unsubscribe_all(self):
        if self.rgb_sub:
            self.destroy_subscription(self.rgb_sub)
            self.rgb_sub = None
        if self.depth_sub:
            self.destroy_subscription(self.depth_sub)
            self.depth_sub = None
        if self.caminfo_sub:
            self.destroy_subscription(self.caminfo_sub)
            self.caminfo_sub = None
        self.get_logger().info('🟠 已退訂所有感知主題，釋放資源！')

    def process_inference(self):
        results = self.model(self.rgb_img)[0]
        centers_info = []
        window_radius = 5

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0].item())
            cls = int(box.cls[0])
            label = self.model.names[cls].lower()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            min_x = max(cx - window_radius, 0)
            max_x = min(cx + window_radius, self.depth_img.shape[1] - 1)
            min_y = max(cy - window_radius, 0)
            max_y = min(cy + window_radius, self.depth_img.shape[0] - 1)
            window = self.depth_img[min_y:max_y+1, min_x:max_x+1]

            valid_depths = window[np.isfinite(window) & (window > 0)]
            if valid_depths.size == 0:
                self.get_logger().warn(f"{label} 中心({cx},{cy}) 未取到有效深度，跳過。")
                continue
            mean_depth = float(np.median(valid_depths))

            X, Y, Z = self.deproject_pixel_to_3d(cx, cy, mean_depth, self.camera_info)
            center_info = {
                'label': label,
                'confidence': conf,
                'u': cx,
                'v': cy,
                'mean_depth': mean_depth,
                'X': X,
                'Y': Y,
                'Z': Z
            }
            centers_info.append(center_info)

            point_msg = PointStamped()
            point_msg.header = self.rgb_header
            point_msg.header.frame_id = "sim_camera"
            point_msg.point.x = X
            point_msg.point.y = Y
            point_msg.point.z = Z
            self.pub_center.publish(point_msg)

            self.get_logger().info(
                f"{label}: (u,v)=({cx},{cy}) mean_depth={mean_depth:.3f} -> (X,Y,Z)=({X:.3f},{Y:.3f},{Z:.3f})"
            )

        self.latest_results = centers_info

        # ==== [重點] 自動儲存 json 與圖片到指定目錄 ====
        timestamp = int(time.time())
        json_path = f"{DATA_DIR}/yolo_results_{timestamp}.json"
        img_path = f"{DATA_DIR}/yolo_center_check_{timestamp}.png"

        with open(json_path, 'w') as f:
            json.dump(centers_info, f, indent=2)
        self.get_logger().info(f'已儲存感知 json 檔：{json_path}')

        img_draw = self.rgb_img.copy()
        for c in centers_info:
            color = (0,255,0) if c['label']=="crop" else (0,0,255)
            cv2.circle(img_draw, (c['u'], c['v']), 5, color, -1)
            txt = f"{c['label']},d={c['mean_depth']:.2f}"
            cv2.putText(img_draw, txt, (c['u']+5, c['v']-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.imwrite(img_path, img_draw)
        self.get_logger().info(f'已儲存可視化圖片：{img_path}')

    def deproject_pixel_to_3d(self, u, v, depth, cam_info):
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z

    def result_callback(self, request, response):
        result_json = json.dumps(self.latest_results, indent=2)
        response.success = True
        response.message = result_json
        return response

def main(args=None):
    rclpy.init(args=args)
    node = YoloRGBDMeanDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 手動中止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
