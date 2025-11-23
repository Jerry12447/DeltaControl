#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Delta機器人正運動學計算（獨立版本）
從三個馬達角度計算末端位置（x, y, z）

這是一個獨立的Python模組，不依賴ROS2或其他功能包。

角度定義與逆運動學一致：
- 0度：手臂平舉（邏輯零點）
- -38度：極限開關位置（機械零點/歸零點）

關於歸零點補償的說明：
1. 歸零點補償是角度定義的問題，不影響幾何計算
2. 逆運動學計算的角度是相對於0度（平舉）的邏輯角度
3. 正運動學使用相同的角度定義，直接使用逆運動學返回的角度即可
4. 幾何計算（三球面交點）本身不受歸零點補償影響
5. 重要的是：正運動學輸入的角度必須與逆運動學輸出的角度定義一致
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class Point:
    """三維座標點"""
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0


@dataclass
class Angle:
    """三個馬達角度"""
    Theta1: float = 0.0
    Theta2: float = 0.0
    Theta3: float = 0.0


class DeltaForwardKinematics:
    """Delta機器人正運動學計算類
    
    使用三球面交點法計算末端位置：
    1. 根據三個馬達角度計算三個上臂關節位置
    2. 以三個關節為中心，下臂長度為半徑建立三個球面
    3. 求解三個球面的交點即為末端位置
    
    參數說明（與逆運動學保持一致）：
    - RD_RF: 上臂長度，300mm
    - RD_RE: 下臂長度，700mm
    - RD_F: 固定平台半徑，334.641mm
    - RD_E: 移動平台半徑，207.846mm
    """
    
    def __init__(self, 
                 RD_RF: float = 300.0,
                 RD_RE: float = 700.0,
                 RD_F: float = 334.641,
                 RD_E: float = 207.846):
        """
        初始化Delta機器人正運動學計算器
        
        Args:
            RD_RF: 上臂長度（mm），默認300mm
            RD_RE: 下臂長度（mm），默認700mm
            RD_F: 固定平台半徑（mm），默認334.641mm
            RD_E: 移動平台半徑（mm），默認207.846mm
        """
        # 三角函數常數（與逆運動學保持一致）
        self.tan30 = 1 / math.sqrt(3)
        self.sin30 = 0.5
        self.cos30 = math.sqrt(3) / 2
        self.tan60 = math.sqrt(3)
        self.sin120 = math.sqrt(3) / 2
        self.cos120 = -0.5
        
        # 機器人參數
        self.RD_RF = RD_RF
        self.RD_RE = RD_RE
        self.RD_F = RD_F
        self.RD_E = RD_E
        
        # 計算常用值
        self.RD_RF_Pow2 = self.RD_RF * self.RD_RF
        self.RD_RE_Pow2 = self.RD_RE * self.RD_RE
        self._y1_ = -0.5 * self.tan30 * self.RD_F
        self._y0_ = 0.5 * self.tan30 * self.RD_E
    
    def calculate_joint_position(self, theta_rad: float, base_idx: int) -> Optional[np.ndarray]:
        """
        計算單個上臂關節位置
        
        根據逆運動學的邏輯：
        - 逆運動學中，角度是從 atan(-zj/(y1 - yj)) 計算的
        - y1 = -0.5 * tan30 * RD_F 是基座在局部Y方向的偏移
        - 上臂在局部Y-Z平面內旋轉，長度為RD_RF
        
        從角度直接計算關節位置：
        - 在局部座標系中，上臂在Y-Z平面內旋轉
        - 角度定義：0度時上臂水平（指向Y軸正方向），負角度時向下旋轉
        - 關節相對於基座的方向向量為：
          dy = RD_RF * cos(theta)  （Y方向分量）
          dz = -RD_RF * sin(theta)  （Z方向分量，負號表示向下為正）
        
        Args:
            theta_rad: 馬達角度（弧度），0度為平舉，-38度為極限開關
            base_idx: 基座索引（0, 1, 2）
            
        Returns:
            關節位置 [x, y, z]，如果計算失敗返回None
        """
        if base_idx < 0 or base_idx > 2:
            return None
        
        # 根據逆運動學的邏輯：
        # y1 = -0.5 * tan30 * RD_F 是基座在局部Y方向的偏移
        y1 = self._y1_
        
        # 從角度直接計算關節位置
        # 在局部座標系中，上臂在Y-Z平面內旋轉
        # 角度定義：0度時上臂水平（指向Y軸正方向），負角度時向下旋轉
        # 關節相對於基座的方向向量為：
        # dy = RD_RF * cos(theta)  （Y方向分量）
        # dz = -RD_RF * sin(theta)  （Z方向分量，負號表示向下為正）
        
        # 計算關節相對於基座的偏移
        dy = self.RD_RF * math.cos(theta_rad)  # Y方向分量
        dz = -self.RD_RF * math.sin(theta_rad)  # Z方向分量（向下為正）
        
        # 計算關節在局部座標系中的絕對位置
        # 基座在 (0, y1, 0)，所以關節在 (0, y1 + dy, dz)
        yj = y1 + dy
        zj = dz
        
        # 驗證：檢查上臂長度是否正確
        dist = math.sqrt(dy*dy + dz*dz)
        if abs(dist - self.RD_RF) > 1.0:  # 允許1mm誤差
            return None
        
        # 現在將局部座標轉換到全局座標系
        # 根據逆運動學，三個基座的座標變換是：
        # 基座1：直接使用 (x, y, z)
        # 基座2：使用 (x*cos120 + y*sin120, y*cos120 - x*sin120, z)
        # 基座3：使用 (x*cos120 - y*sin120, y*cos120 + x*sin120, z)
        
        # 局部座標系中關節位置為 (0, yj, zj)
        # 需要轉換到全局座標系
        
        if base_idx == 0:
            # 基座1：局部Y軸對應全局Y軸，局部Z軸對應全局Z軸
            # 全局位置是 (0, yj, zj)
            joint_pos = np.array([0.0, yj, zj])
        elif base_idx == 1:
            # 基座2：座標系旋轉120度
            # 從全局到局部的變換矩陣：
            # [x_local]   [cos120  sin120] [x_global]
            # [y_local] = [cos120 -sin120] [y_global]
            # 
            # 但更簡單的方法：直接求解線性方程組
            # x_local = 0 = x_global*cos120 + y_global*sin120
            # y_local = yj = y_global*cos120 - x_global*sin120
            # 
            # 從第一式：x_global = -y_global * sin120 / cos120
            # 代入第二式：yj = y_global*cos120 - (-y_global * sin120/cos120) * sin120
            #           yj = y_global * (cos120 + sin120²/cos120)
            #           yj = y_global * (cos120² + sin120²) / cos120 = y_global / cos120
            # 
            # 所以：y_global = yj * cos120
            #      x_global = -yj * cos120 * sin120 / cos120 = -yj * sin120
            y_global = yj * self.cos120
            x_global = -yj * self.sin120
            z_global = zj
            joint_pos = np.array([x_global, y_global, z_global])
        else:  # base_idx == 2
            # 基座3：座標系旋轉-120度
            # 從全局到局部的變換：
            # x_local = x_global*cos120 - y_global*sin120
            # y_local = y_global*cos120 + x_global*sin120
            # 
            # 局部座標 (0, yj)，求解：
            # 0 = x_global*cos120 - y_global*sin120  =>  x_global = y_global * sin120 / cos120
            # yj = y_global*cos120 + x_global*sin120
            # 
            # 代入：yj = y_global*cos120 + (y_global * sin120/cos120) * sin120
            #      yj = y_global * (cos120 + sin120²/cos120) = y_global / cos120
            # 
            # 所以：y_global = yj * cos120
            #      x_global = yj * cos120 * sin120 / cos120 = yj * sin120
            y_global = yj * self.cos120
            x_global = yj * self.sin120
            z_global = zj
            joint_pos = np.array([x_global, y_global, z_global])
        
        return joint_pos
    
    def forward_kinematics(self, angle: Angle) -> Optional[Point]:
        """
        正運動學：從三個馬達角度計算末端位置
        
        Args:
            angle: 三個馬達角度（弧度），使用Angle類
                  - Theta1, Theta2, Theta3 分別對應三個馬達
                  - 角度定義：0度為平舉，-38度為極限開關位置
                  
        Returns:
            末端位置 Point(X, Y, Z)，如果計算失敗返回None
            
        注意：
            - 歸零點補償不影響幾何計算，但角度定義必須與逆運動學一致
            - 如果逆運動學返回的角度是相對於0度（平舉）的，這裡也使用相同定義
        """
        # 計算三個上臂關節位置
        joint1 = self.calculate_joint_position(angle.Theta1, 0)
        joint2 = self.calculate_joint_position(angle.Theta2, 1)
        joint3 = self.calculate_joint_position(angle.Theta3, 2)
        
        if joint1 is None or joint2 is None or joint3 is None:
            return None
        
        # 使用三球面交點法計算末端位置
        # 三個球面方程：
        # (x - x1)² + (y - y1)² + (z - z1)² = RD_RE²
        # (x - x2)² + (y - y2)² + (z - z2)² = RD_RE²
        # (x - x3)² + (y - y3)² + (z - z3)² = RD_RE²
        
        # 簡化：先計算兩個球面的交線，再與第三個球面求交點
        # 使用數值方法求解
        
        # 方法：使用牛頓-拉夫遜法或直接解析求解
        # 這裡使用解析方法：通過兩個球面方程相減得到平面方程
        
        # 球面1和球面2相減
        # 2(x2-x1)x + 2(y2-y1)y + 2(z2-z1)z = (x2²+y2²+z2²) - (x1²+y1²+z1²)
        x1, y1, z1 = joint1[0], joint1[1], joint1[2]
        x2, y2, z2 = joint2[0], joint2[1], joint2[2]
        x3, y3, z3 = joint3[0], joint3[1], joint3[2]
        
        # 計算兩個球面交線所在的平面
        # 平面方程：ax + by + cz = d
        a12 = 2 * (x2 - x1)
        b12 = 2 * (y2 - y1)
        c12 = 2 * (z2 - z1)
        d12 = (x2*x2 + y2*y2 + z2*z2) - (x1*x1 + y1*y1 + z1*z1)
        
        # 球面1和球面3相減
        a13 = 2 * (x3 - x1)
        b13 = 2 * (y3 - y1)
        c13 = 2 * (z3 - z1)
        d13 = (x3*x3 + y3*y3 + z3*z3) - (x1*x1 + y1*y1 + z1*z1)
        
        # 求解兩個平面的交線，然後與球面1求交點
        # 使用線性代數方法求解
        
        # 構建線性方程組：兩個平面方程
        # a12*x + b12*y + c12*z = d12
        # a13*x + b13*y + c13*z = d13
        
        # 如果c12和c13不為0，可以從z表達x和y
        # 這裡使用更穩定的方法：直接求解三個方程（兩個平面 + 一個球面）
        
        # 方法：使用最小二乘法或直接求解
        # 構建矩陣方程 Ax = b
        
        # 如果兩個平面不平行，可以求解
        # 檢查係數矩陣的秩
        A = np.array([
            [a12, b12, c12],
            [a13, b13, c13]
        ])
        
        b = np.array([d12, d13])
        
        # 使用SVD求解（處理奇異情況）
        try:
            # 計算兩個平面的法向量叉積，得到交線方向向量
            n1 = np.array([a12, b12, c12])
            n2 = np.array([a13, b13, c13])
            
            # 交線方向向量
            direction = np.cross(n1, n2)
            
            # 如果方向向量為零，說明兩個平面平行或重合
            if np.linalg.norm(direction) < 1e-6:
                return None
            
            # 找到交線上的一個點（通過求解線性方程組）
            # 使用最小二乘法找到最接近原點的點
            A_full = np.vstack([A, direction])
            b_full = np.hstack([b, 0])
            
            # 使用偽逆求解
            point_on_line = np.linalg.pinv(A_full) @ b_full
            
            # 現在需要找到這個交線與球面1的交點
            # 球面1方程：(x - x1)² + (y - y1)² + (z - z1)² = RD_RE²
            # 交線參數方程：p = point_on_line + t * direction
            
            # 代入球面方程求解t
            # (p0 + t*d - j1) · (p0 + t*d - j1) = RD_RE²
            p0 = point_on_line
            d = direction / np.linalg.norm(direction)  # 歸一化方向向量
            j1 = joint1
            
            # (p0 - j1 + t*d) · (p0 - j1 + t*d) = RD_RE²
            diff = p0 - j1
            a_coeff = np.dot(d, d)  # 應該是1（已歸一化）
            b_coeff = 2 * np.dot(diff, d)
            c_coeff = np.dot(diff, diff) - self.RD_RE_Pow2
            
            # 求解二次方程：a*t² + b*t + c = 0
            discriminant = b_coeff * b_coeff - 4 * a_coeff * c_coeff
            
            if discriminant < 0:
                return None
            
            sqrt_disc = math.sqrt(discriminant)
            t1 = (-b_coeff + sqrt_disc) / (2 * a_coeff)
            t2 = (-b_coeff - sqrt_disc) / (2 * a_coeff)
            
            # 選擇合理的解（z應該為負值，因為工作空間在下方）
            pos1 = p0 + t1 * d
            pos2 = p0 + t2 * d
            
            # 選擇z值更負（更下方）的解，通常這是正確的工作位置
            if pos1[2] < pos2[2]:
                end_effector_pos = pos1
            else:
                end_effector_pos = pos2
            
            # 驗證：檢查是否滿足所有三個球面方程（允許一定誤差）
            dist1 = np.linalg.norm(end_effector_pos - joint1)
            dist2 = np.linalg.norm(end_effector_pos - joint2)
            dist3 = np.linalg.norm(end_effector_pos - joint3)
            
            tolerance = 1.0  # 允許1mm誤差
            if (abs(dist1 - self.RD_RE) > tolerance or
                abs(dist2 - self.RD_RE) > tolerance or
                abs(dist3 - self.RD_RE) > tolerance):
                # 如果誤差太大，嘗試另一個解
                if pos1[2] < pos2[2]:
                    end_effector_pos = pos2
                else:
                    end_effector_pos = pos1
                
                # 再次驗證
                dist1 = np.linalg.norm(end_effector_pos - joint1)
                dist2 = np.linalg.norm(end_effector_pos - joint2)
                dist3 = np.linalg.norm(end_effector_pos - joint3)
                
                if (abs(dist1 - self.RD_RE) > tolerance or
                    abs(dist2 - self.RD_RE) > tolerance or
                    abs(dist3 - self.RD_RE) > tolerance):
                    return None
            
            return Point(
                X=float(end_effector_pos[0]),
                Y=float(end_effector_pos[1]),
                Z=float(end_effector_pos[2])
            )
            
        except Exception as e:
            return None


def test_forward_kinematics():
    """測試正運動學計算 - 支持輸入四組角度"""
    fk = DeltaForwardKinematics()
    
    print("Delta機器人正運動學計算測試")
    print("=" * 60)
    print()
    
    # 定義四組測試角度（單位：度）
    test_angles = [
        {
            "name": "測試1",
            "theta1": -10.0,
            "theta2": -6.9,
            "theta3": -29.8,
            "description": "三個馬達都在-38度（極限開關位置）"
        },
        {
            "name": "測試2",
            "theta1": -33.3,
            "theta2": -3.5,
            "theta3": -0.7,
            "description": "不同角度組合"
        },
        {
            "name": "測試3",
            "theta1": -0.7,
            "theta2": -25.8,
            "theta3": -21,
            "description": "三個馬達都在30度"
        },
        {
            "name": "測試4",
            "theta1": -24.6,
            "theta2": -22.1,
            "theta3": 9.7,
            "description": "三個馬達都在0度（平舉）"
        }
    ]
    
    # 計算並顯示每組角度的結果
    for i, test_case in enumerate(test_angles, 1):
        angle = Angle(
            Theta1=math.radians(test_case["theta1"]),
            Theta2=math.radians(test_case["theta2"]),
            Theta3=math.radians(test_case["theta3"])
        )
        
        result = fk.forward_kinematics(angle)
        
        print(f"{test_case['name']}: {test_case['description']}")
        print(f"  輸入角度: Theta1={test_case['theta1']:.2f}°, "
              f"Theta2={test_case['theta2']:.2f}°, "
              f"Theta3={test_case['theta3']:.2f}°")
        
        if result:
            print(f"  末端位置: X={result.X:.2f} mm, "
                  f"Y={result.Y:.2f} mm, "
                  f"Z={result.Z:.2f} mm")
        else:
            print("  計算失敗：無法找到有效的末端位置")
        
        if i < len(test_angles):
            print()
    
    print("=" * 60)


if __name__ == "__main__":
    test_forward_kinematics()

