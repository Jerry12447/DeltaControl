#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class ExtrinsicCalibrationNode(Node):
    def __init__(self):
        super().__init__('extrinsic_calibration_node')

        # 初始化 CV Bridge
        self.bridge = CvBridge()

        # 棋盤格參數 - 根據實際棋盤格調整
        # 對於 640x640 解析度，建議使用：
        # - (9, 6): 9列6行內部角點，每個格子約 64x91 像素
        # - (10, 7): 10列7行內部角點，每個格子約 58x80 像素（當前棋盤格）
        # - (7, 5): 7列5行內部角點，每個格子約 80x107 像素
        # 注意：格式為 (列數, 行數)，即 (cols, rows)
        # 根據 Standard.png，實際棋盤格為 11列×8行，內部角點為 (10, 7)
        self._Chessboard = (6, 4)  # 棋盤格的長寬數量（內部角點數量）
        self._Chessboard_squareSize = 50  # 棋盤格小格的長度 mm

        # 相機參數變數
        self.camera_matrix = None
        self.dist_coeffs = None
        self.current_image = None
        self.camera_info_received = False

        # 訂閱器
        self.image_sub = self.create_subscription(
            Image, '/rgb', self.image_callback, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.get_logger().info('外參標定節點已啟動')
        self.get_logger().info('請確保棋盤格在視野中，然後按 Enter 開始標定')

    def camera_info_callback(self, msg):
        """接收相機內參與畸變參數"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('已接收相機內參與畸變參數')
            self.get_logger().info(f'相機內參矩陣: \n{self.camera_matrix}')

    def image_callback(self, msg):
        """接收圖像數據"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'圖像轉換失敗: {e}')

    def text_print(self, frame, text):
        """在畫面中顯示文字"""
        word_color = (0, 255, 255)  # 字的顏色
        position = (0, 20)          # 字的位置
        cv2.rectangle(frame, (0, 0), (655, 25), (0, 0, 0), cv2.FILLED)
        cv2.putText(frame, text, position,
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, word_color, 1)

    def standard_catch(self):
        """拍攝標定用棋盤格"""
        if self.current_image is None:
            self.get_logger().warn('尚未接收到圖像數據')
            return None

        # 等待圖像數據穩定
        wait_count = 0
        max_wait = 100  # 最大等待次數
        while self.current_image is None and wait_count < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)  # 增加等待時間到 0.1 秒
            wait_count += 1
            if wait_count % 10 == 0:  # 每10次顯示一次等待信息
                self.get_logger().info(f'等待圖像數據... ({wait_count}/{max_wait})')

        if self.current_image is None:
            self.get_logger().error('等待圖像數據超時')
            return None

        while True:
            if self.current_image is not None:
                frame = self.current_image.copy()
                text = 'Place the Calibration board and press Enter'
                self.text_print(frame, text)
                cv2.imshow("Standard", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == 13:  # Enter 鍵
                    # 再次等待一小段時間確保圖像穩定
                    for _ in range(5):
                        rclpy.spin_once(self, timeout_sec=0.05)
                    
                    standard_image = self.current_image.copy()
                    cv2.imwrite("Standard.png", standard_image)
                    self.get_logger().info('標定圖像已儲存為 Standard.png')
                    cv2.destroyAllWindows()
                    return standard_image
                elif key == 27:  # ESC 鍵
                    cv2.destroyAllWindows()
                    return None

            # 讓 ROS2 回調函數有機會執行，增加等待時間
            rclpy.spin_once(self, timeout_sec=0.05)  # 從 0.01 增加到 0.05 秒

    def calculate_extrinsics(self):
        """計算外參 - 改用手動標記四個關鍵點"""
        if not self.camera_info_received:
            self.get_logger().error('尚未接收到相機內參，無法進行標定')
            return False

        # 拍攝標定圖像
        std_img = self.standard_catch()
        if std_img is None:
            self.get_logger().info('標定已取消')
            return False

        # 轉換為灰階
        gray = cv2.cvtColor(std_img, cv2.COLOR_BGR2GRAY)
        
        # 診斷信息
        self.get_logger().info(f'圖像尺寸: {std_img.shape}')
        self.get_logger().info(f'尋找棋盤格角點，參數: {self._Chessboard} (列, 行)')
        self.get_logger().info(f'預期內部角點: {self._Chessboard[0]}列 × {self._Chessboard[1]}行')
        
        # 保存原始圖像用於後續處理
        original_gray = gray.copy()
        original_color = std_img.copy()
        crop_region = None
        
        # 嘗試自動裁剪背景，只保留棋盤格區域
        cropped_img, crop_region = self._auto_crop_chessboard(gray, std_img)
        if cropped_img is not None:
            self.get_logger().info(f'成功裁剪背景，裁剪區域: {crop_region}')
            self.get_logger().info(f'裁剪後圖像尺寸: {cropped_img.shape}')
            # 使用裁剪後的圖像進行檢測
            gray = cropped_img
            std_img_cropped = std_img[crop_region[1]:crop_region[3], crop_region[0]:crop_region[2]]
            cv2.imwrite('debug_cropped.png', std_img_cropped)
            self.get_logger().info('裁剪後的圖像已儲存為 debug_cropped.png')
            # 保存裁剪後的彩色圖像用於驗證
            std_img = std_img_cropped
        else:
            self.get_logger().warn('無法自動裁剪背景，使用完整圖像')
        
        # 儲存原始灰階圖像用於診斷
        cv2.imwrite('debug_gray.png', gray)
        self.get_logger().info('灰階圖像已儲存為 debug_gray.png')

        # 準備多種預處理圖像（包括更激進的方法）
        processed_images = [
            (gray, "原始灰階"),
            (cv2.equalizeHist(gray), "直方圖均衡化"),
            (cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(gray), "CLAHE增強"),
            (cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8)).apply(gray), "CLAHE增強(強)"),
            (cv2.GaussianBlur(gray, (5, 5), 0), "高斯模糊"),
            (cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2), "自適應二值化"),
            (cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1], "OTSU二值化"),
            (cv2.bilateralFilter(gray, 9, 75, 75), "雙邊濾波"),
        ]
        
        # 儲存預處理圖像
        for idx, (proc_img, name) in enumerate(processed_images[1:], 1):
            cv2.imwrite(f'debug_preprocess_{idx}_{name.replace(" ", "_")}.png', proc_img)

        # 嘗試多種檢測方法
        detection_flags = [
            (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS, "標準方法"),
            (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE, "無濾波"),
            (cv2.CALIB_CB_ADAPTIVE_THRESH, "僅自適應閾值"),
            (cv2.CALIB_CB_FAST_CHECK, "快速檢查"),
            (cv2.CALIB_CB_NORMALIZE_IMAGE, "僅正規化"),
            (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK, "自適應+快速"),
        ]
        
        # 嘗試使用 findChessboardCornersSB（如果 OpenCV 版本支持）
        try:
            has_sb = hasattr(cv2, 'findChessboardCornersSB')
        except:
            has_sb = False
        
        ret = False
        corners = None
        used_method = None
        used_image = None
        
        # 對每種預處理圖像嘗試所有檢測方法
        for proc_img, img_name in processed_images:
            if ret:
                break
            self.get_logger().info(f'\n--- 使用預處理: {img_name} ---')
            
            # 先嘗試標準的 findChessboardCorners
            for flags, method_name in detection_flags:
                self.get_logger().info(f'  嘗試方法: {method_name}')
                ret, corners = cv2.findChessboardCorners(proc_img, self._Chessboard, flags)
                if ret:
                    self.get_logger().info(f'✓ 使用 {img_name} + {method_name} 成功找到角點')
                    used_method = method_name
                    used_image = img_name
                    gray = proc_img  # 使用成功的預處理圖像
                    break
                else:
                    self.get_logger().warn(f'  ✗ {method_name} 未找到角點')
            
            # 如果標準方法失敗，嘗試 findChessboardCornersSB（如果支持）
            if not ret and has_sb:
                self.get_logger().info(f'  嘗試方法: findChessboardCornersSB (新方法)')
                try:
                    ret, corners = cv2.findChessboardCornersSB(proc_img, self._Chessboard, 
                                                              cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
                    if ret:
                        self.get_logger().info(f'✓ 使用 {img_name} + findChessboardCornersSB 成功找到角點')
                        used_method = "findChessboardCornersSB"
                        used_image = img_name
                        gray = proc_img
                        break
                except Exception as e:
                    self.get_logger().warn(f'  ✗ findChessboardCornersSB 失敗: {e}')

        if ret:
            self.get_logger().info('成功找到棋盤格角點')
            # 儲存除錯影像
            debug_img = std_img.copy()
            cv2.putText(debug_img, f'Chessboard: {self._Chessboard}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imwrite('debug_chessboard.png', debug_img)
            self.get_logger().info('除錯影像已儲存為 debug_chessboard.png')

            # 精細化角點位置
            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)

            # 如果使用了裁剪，將角點座標轉換回原始圖像座標系
            if crop_region is not None:
                x_offset, y_offset = crop_region[0], crop_region[1]
                # 調整所有角點座標到原始圖像座標系
                corners_original = corners.copy()
                for i in range(len(corners_original)):
                    corners_original[i][0][0] += x_offset
                    corners_original[i][0][1] += y_offset
                self.get_logger().info(f'已將角點座標轉換回原始圖像座標系（偏移: {x_offset}, {y_offset}）')
            else:
                corners_original = corners
                x_offset, y_offset = 0, 0

            # 選擇四個關鍵角點進行標定（使用裁剪後的座標）
            key_corners = self.select_key_corners(corners, std_img)
            
            # 將關鍵角點座標轉換回原始圖像座標系
            key_corners_original = None
            if key_corners is not None:
                key_corners_original = key_corners.copy()
                for i in range(len(key_corners_original)):
                    key_corners_original[i][0] += x_offset
                    key_corners_original[i][1] += y_offset
                
                # 輸出原始圖像座標系的精確像素座標
                self.get_logger().info('\n=== 四個角點在原始圖像中的精確像素座標 ===')
                labels = ['點1: 左上角 (紅色)', '點2: 右上角 (綠色)', 
                         '點3: 左下角 (藍色)', '點4: 右下角 (黃色)']
                for i, (corner, label) in enumerate(zip(key_corners_original, labels)):
                    self.get_logger().info(f'{label}: ({corner[0]:.2f}, {corner[1]:.2f})')
                self.get_logger().info('=' * 50)

            if key_corners is not None and key_corners_original is not None:
                # 手動輸入這四個點的實際Delta座標
                world_points = self.input_delta_coordinates()

                if world_points is not None:
                    # 使用四個關鍵點計算外參（使用裁剪後的座標）
                    success, rvecs, tvecs = cv2.solvePnP(
                        world_points, key_corners, self.camera_matrix, self.dist_coeffs)

                    if success:
                        # 驗證標定結果（傳入原始圖像座標系的角點用於顯示）
                        if self.verify_calibration_result(key_corners, key_corners_original, world_points, rvecs, tvecs, crop_region):
                            # 儲存外參到 YAML 檔案
                            self.save_extrinsics_to_yaml(rvecs, tvecs)
                            self.get_logger().info('外參標定完成')
                            return True
                        else:
                            self.get_logger().error('標定結果驗證失敗，請重新標定')
                            return False
                    else:
                        self.get_logger().error('solvePnP 計算失敗')
                        return False
                else:
                    self.get_logger().info('座標輸入已取消')
                    return False
            else:
                self.get_logger().info('角點選擇已取消')
                return False
        else:
            self.get_logger().error('無法找到棋盤格角點，嘗試其他可能的參數組合...')
            self.get_logger().error(f'原始參數: {self._Chessboard} (列, 行)')
            
            # 嘗試其他常見的參數組合
            alternative_params = [
                (9, 6), (8, 6), (11, 8), (7, 5), (6, 4),
                (10, 6), (9, 7), (8, 5), (11, 7), (12, 8)
            ]
            
            # 移除重複的參數
            alternative_params = [p for p in alternative_params if p != self._Chessboard]
            
            self.get_logger().info('嘗試其他參數組合...')
            for alt_params in alternative_params:
                self.get_logger().info(f'  嘗試參數: {alt_params} (列, 行)')
                ret, corners = cv2.findChessboardCorners(
                    gray, alt_params, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
                if ret:
                    self.get_logger().warn(f'⚠ 使用參數 {alt_params} 找到了角點！')
                    self.get_logger().warn(f'⚠ 請確認實際棋盤格參數是否為 {alt_params}')
                    self.get_logger().warn(f'⚠ 如果是，請將 _Chessboard 改為 {alt_params}')
                    # 儲存找到的角點圖像
                    debug_img = std_img.copy()
                    cv2.drawChessboardCorners(debug_img, alt_params, corners, ret)
                    cv2.putText(debug_img, f'Found: {alt_params}', (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imwrite('debug_found_alternative.png', debug_img)
                    self.get_logger().info('已儲存找到的角點圖像: debug_found_alternative.png')
                    break
            
            if not ret:
                self.get_logger().error('所有參數組合都無法找到角點')
                self.get_logger().error('可能的原因:')
                self.get_logger().error('1. 棋盤格參數不匹配（檢查實際棋盤格的列數和行數）')
                self.get_logger().error('2. 圖像對比度不足（檢查 debug_gray.png）')
                self.get_logger().error('3. 棋盤格在圖像中太小或太大')
                self.get_logger().error('4. 光照條件不佳或反光')
                self.get_logger().error('5. 棋盤格有變形或不在同一平面')
                
                # 嘗試檢測圖像中的邊緣來幫助診斷
                edges = cv2.Canny(gray, 50, 150)
                cv2.imwrite('debug_edges.png', edges)
                self.get_logger().info('邊緣檢測圖像已儲存為 debug_edges.png（用於診斷）')
                
                # 生成手動驗證圖像：顯示預期的角點位置
                # 如果使用了裁剪後的圖像，使用裁剪後的圖像；否則使用原始圖像
                verify_color = std_img if crop_region is None else original_color
                verify_gray = gray if crop_region is None else original_gray
                self._create_manual_verification_image(verify_color, verify_gray, crop_region)
            
            return False
    
    def _auto_crop_chessboard(self, gray_img, color_img):
        """自動裁剪背景，只保留棋盤格區域"""
        self.get_logger().info('嘗試自動裁剪背景...')
        
        # 使用邊緣檢測找到棋盤格邊界
        edges = cv2.Canny(gray_img, 50, 150)
        
        # 形態學操作：閉合操作連接邊緣
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None
        
        # 找到最大的輪廓（應該是棋盤格）
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 獲取邊界矩形
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # 添加邊距（10%）
        margin_x = int(w * 0.1)
        margin_y = int(h * 0.1)
        
        # 確保不超出圖像邊界
        x = max(0, x - margin_x)
        y = max(0, y - margin_y)
        w = min(gray_img.shape[1] - x, w + 2 * margin_x)
        h = min(gray_img.shape[0] - y, h + 2 * margin_y)
        
        # 裁剪圖像
        cropped_gray = gray_img[y:y+h, x:x+w]
        
        # 驗證裁剪後的圖像大小是否合理（至少是原圖的 30%）
        if cropped_gray.size < gray_img.size * 0.3:
            self.get_logger().warn('裁剪區域太小，可能不正確，使用完整圖像')
            return None, None
        
        return cropped_gray, (x, y, x+w, y+h)
    
    def _create_manual_verification_image(self, color_img, gray_img, crop_region=None):
        """創建手動驗證圖像，幫助用戶確認參數"""
        self.get_logger().info('\n=== 生成手動驗證圖像 ===')
        
        # 如果使用了裁剪，在原始圖像上繪製
        if crop_region is not None:
            self.get_logger().info('使用裁剪區域在原始圖像上繪製驗證標記')
            x1, y1, x2, y2 = crop_region
            # 在原始圖像的裁剪區域內進行處理
            cropped_gray = gray_img[y1:y2, x1:x2]
            cropped_color = color_img[y1:y2, x1:x2]
        else:
            cropped_gray = gray_img
            cropped_color = color_img
            x1, y1 = 0, 0
        
        # 嘗試檢測邊緣
        edges = cv2.Canny(cropped_gray, 50, 150)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 創建驗證圖像（使用完整圖像）
        verify_img = color_img.copy()
        
        # 在裁剪區域內繪製邊緣
        if crop_region is not None:
            # 調整輪廓座標到原始圖像座標系
            adjusted_contours = [cnt + np.array([x1, y1]) for cnt in contours]
            cv2.drawContours(verify_img, adjusted_contours, -1, (0, 255, 0), 1)
        else:
            cv2.drawContours(verify_img, contours, -1, (0, 255, 0), 1)
        
        # 計算預期的角點位置
        h, w = cropped_gray.shape
        cols, rows = self._Chessboard
        
        # 估算棋盤格區域（假設佔圖像的 70%）
        board_w = int(w * 0.7)
        board_h = int(h * 0.7)
        board_x = (w - board_w) // 2
        board_y = (h - board_h) // 2
        
        # 計算每個格子的尺寸
        square_w = board_w / (cols + 1)
        square_h = board_h / (rows + 1)
        
        # 繪製預期的角點位置（調整到原始圖像座標系）
        expected_corners = []
        for i in range(rows):
            for j in range(cols):
                x = int(x1 + board_x + (j + 1) * square_w)
                y = int(y1 + board_y + (i + 1) * square_h)
                expected_corners.append((x, y))
                cv2.circle(verify_img, (x, y), 5, (255, 0, 0), -1)  # 藍色圓點
        
        # 繪製棋盤格邊界（調整到原始圖像座標系）
        cv2.rectangle(verify_img, (x1 + board_x, y1 + board_y), 
                     (x1 + board_x + board_w, y1 + board_y + board_h), (0, 0, 255), 2)
        
        # 如果使用了裁剪，繪製裁剪邊界
        if crop_region is not None:
            cv2.rectangle(verify_img, (x1, y1), (x2, y2), (255, 255, 0), 2)  # 黃色邊界
        
        # 添加文字說明
        cv2.putText(verify_img, f'Expected: {cols}x{rows} corners', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(verify_img, 'Blue dots: Expected corner positions', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(verify_img, 'Red box: Estimated board area', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(verify_img, 'Green lines: Detected edges', (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imwrite('debug_manual_verification.png', verify_img)
        self.get_logger().info('手動驗證圖像已儲存為 debug_manual_verification.png')
        self.get_logger().info('請檢查藍色圓點是否與實際角點位置對齊')
        self.get_logger().info(f'如果對齊，參數 {self._Chessboard} 是正確的')
        self.get_logger().info('如果未對齊，請手動數一下實際的角點數量並調整參數')

    def select_key_corners(self, corners, img):
        """選擇四個關鍵角點"""
        # 選擇棋盤格的四個角落點
        # corners 是按照棋盤格從左到右、從上到下的順序排列
        # 注意：棋盤格是 (cols, rows)
        rows, cols = self._Chessboard[1], self._Chessboard[0]

        key_indices = [
            0,                    # 左上角
            cols - 1,            # 右上角
            (rows - 1) * cols,   # 左下角
            rows * cols - 1      # 右下角
        ]

        key_corners = np.array([corners[i][0]
                                for i in key_indices], dtype=np.float32)

        # 在圖像上標記這四個點
        display_img = img.copy()
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0),
                  (255, 255, 0)]  # 紅、綠、藍、黃
        labels = ['Point 1 (Top-Left)', 'Point 2 (Top-Right)',
                  'Point 3 (Bottom-Left)', 'Point 4 (Bottom-Right)']

        for i, (corner, color, label) in enumerate(zip(key_corners, colors, labels)):
            cv2.circle(
                display_img, (int(corner[0]), int(corner[1])), 8, color, -1)
            cv2.putText(display_img, f'{i+1}',
                        (int(corner[0])+15, int(corner[1])-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # 顯示說明
        y_offset = 30
        for i, label in enumerate(labels):
            cv2.putText(display_img, label, (10, y_offset + i*25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2)

        cv2.putText(display_img, 'Press ENTER to confirm, ESC to cancel',
                    (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Select Key Corners', display_img)
        key = cv2.waitKey(0)
        cv2.destroyAllWindows()

        if key == 13:  # Enter鍵
            return key_corners
        else:
            return None

    def input_delta_coordinates(self):
        """輸入四個點的實際Delta座標"""
        print("\n=== 請用Delta手臂測量四個標記點的實際座標 ===")
        print("點1: 左上角 (紅色)")
        print("點2: 右上角 (綠色)")
        print("點3: 左下角 (藍色)")
        print("點4: 右下角 (黃色)")
        print("\n請依序輸入四個點的Delta座標 (單位: mm)")

        world_points = []

        for i in range(4):
            try:
                print(f"\n點{i+1}的座標:")
                x = float(input(f"  X座標 (mm): "))
                y = float(input(f"  Y座標 (mm): "))
                z = float(input(f"  Z座標 (mm, 通常為0): "))

                world_points.append([x, y, z])
                print(f"  已記錄: ({x:.1f}, {y:.1f}, {z:.1f})")

            except ValueError:
                print("輸入格式錯誤，標定已取消")
                return None
            except KeyboardInterrupt:
                print("\n標定已取消")
                return None

        return np.array(world_points, dtype=np.float32)

    def verify_calibration_result(self, image_points, image_points_original, world_points, rvecs, tvecs, crop_region=None):
        """驗證標定結果"""
        # 將世界座標投影回像素座標
        projected_points, _ = cv2.projectPoints(
            world_points, rvecs, tvecs, self.camera_matrix, self.dist_coeffs)
        
        # 如果使用了裁剪，將投影像素也轉換回原始圖像座標系
        if crop_region is not None:
            x_offset, y_offset = crop_region[0], crop_region[1]
            projected_points_original = projected_points.copy()
            for i in range(len(projected_points_original)):
                projected_points_original[i][0][0] += x_offset
                projected_points_original[i][0][1] += y_offset
        else:
            projected_points_original = projected_points

        # 計算重投影誤差
        total_error = 0
        print(f"\n=== 標定結果驗證 ===")
        print(f"{'點':<5} {'原始像素(裁剪後)':<20} {'原始像素(原始圖像)':<25} {'投影像素(原始圖像)':<25} {'誤差(像素)':<10}")
        print("-" * 90)

        for i in range(len(image_points)):
            # 使用裁剪後的座標計算誤差（用於驗證標定精度）
            error = np.linalg.norm(image_points[i] - projected_points[i][0])
            total_error += error
            print(f"點{i+1:<4} ({image_points[i][0]:.1f}, {image_points[i][1]:.1f})   "
                  f"({image_points_original[i][0]:.2f}, {image_points_original[i][1]:.2f})   "
                  f"({projected_points_original[i][0][0]:.2f}, {projected_points_original[i][0][1]:.2f})   "
                  f"{error:.2f}")
        
        # 輸出原始圖像座標系的精確座標
        print(f"\n=== 四個角點在原始圖像中的精確像素座標（用於後續使用）===")
        labels = ['點1: 左上角', '點2: 右上角', '點3: 左下角', '點4: 右下角']
        for i, (corner, label) in enumerate(zip(image_points_original, labels)):
            print(f"{label}: ({corner[0]:.2f}, {corner[1]:.2f})")
        print("=" * 60)

        mean_error = total_error / len(image_points)
        print(f"\n平均重投影誤差: {mean_error:.2f} 像素")

        # 如果平均誤差小於2像素，認為標定成功
        if mean_error < 2.0:
            print("✓ 標定結果良好")
            return True
        else:
            print("✗ 標定誤差過大，建議重新標定")
            return False

    def save_extrinsics_to_yaml(self, rvecs, tvecs):
        """儲存外參到 YAML 檔案"""
        # 將旋轉向量轉換為旋轉矩陣
        rotation_matrix, _ = cv2.Rodrigues(rvecs)

        extrinsic_data = {
            'extrinsic_calibration': {
                'rotation_vector': rvecs.flatten().tolist(),
                'translation_vector': tvecs.flatten().tolist(),
                'rotation_matrix': rotation_matrix.tolist(),
                'chessboard_size': list(self._Chessboard),
                'square_size_mm': self._Chessboard_squareSize
            }
        }

        yaml_filename = 'camera_extrinsics.yaml'
        with open(yaml_filename, 'w') as yaml_file:
            yaml.dump(extrinsic_data, yaml_file, default_flow_style=False)

        self.get_logger().info(f'外參已儲存至 {yaml_filename}')
        self.get_logger().info(f'旋轉向量: {rvecs.flatten()}')
        self.get_logger().info(f'平移向量: {tvecs.flatten()}')


def main(args=None):
    rclpy.init(args=args)

    calibration_node = ExtrinsicCalibrationNode()

    try:
        # 等待相機資訊
        while not calibration_node.camera_info_received:
            rclpy.spin_once(calibration_node, timeout_sec=1.0)
            calibration_node.get_logger().info('等待相機內參資訊...')

        # 等待圖像數據開始接收
        calibration_node.get_logger().info('等待圖像數據開始接收...')
        wait_count = 0
        max_image_wait = 50
        while calibration_node.current_image is None and wait_count < max_image_wait:
            rclpy.spin_once(calibration_node, timeout_sec=0.2)  # 增加等待時間
            wait_count += 1
            if wait_count % 10 == 0:
                calibration_node.get_logger().info(f'等待圖像數據... ({wait_count}/{max_image_wait})')

        if calibration_node.current_image is None:
            calibration_node.get_logger().error('圖像數據接收超時，請檢查相機連接')
            return

        calibration_node.get_logger().info('圖像數據開始接收，準備開始標定...')
        
        # 開始標定
        calibration_node.calculate_extrinsics()

    except KeyboardInterrupt:
        calibration_node.get_logger().info('程式已中斷')
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
