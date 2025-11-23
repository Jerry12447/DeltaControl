#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
像素到長度比例係數計算
使用多個點計算平均比例係數，提高精度
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict
import math

def calculate_pixel_to_length_ratio_multiple(pixel_coords: List[Tuple[float, float]], 
                                           real_coords: List[Tuple[float, float]]) -> Tuple[float, float, Dict]:
    """
    使用多個點計算平均比例係數
    
    Args:
        pixel_coords: 像素座標列表 [(u1,v1), (u2,v2), ...]
        real_coords: 現實座標列表 [(x1,y1), (x2,y2), ...] (單位: mm)
    
    Returns:
        average_ratio: 平均比例係數 (mm/pixel)
        std_ratio: 標準差
        details: 詳細計算結果
    """
    
    if len(pixel_coords) != len(real_coords):
        raise ValueError("像素座標和現實座標數量必須相同")
    
    if len(pixel_coords) < 2:
        raise ValueError("至少需要2個點才能計算比例")
    
    ratios = []
    distance_pairs = []
    
    print("=== 像素到長度比例計算 ===")
    print(f"輸入點數: {len(pixel_coords)}")
    print()
    
    # 計算所有點對之間的比例
    for i in range(len(pixel_coords)):
        for j in range(i+1, len(pixel_coords)):
            # 像素距離
            pixel_dx = pixel_coords[j][0] - pixel_coords[i][0]
            pixel_dy = pixel_coords[j][1] - pixel_coords[i][1]
            pixel_dist = np.sqrt(pixel_dx**2 + pixel_dy**2)
            
            # 現實距離
            real_dx = real_coords[j][0] - real_coords[i][0]
            real_dy = real_coords[j][1] - real_coords[i][1]
            real_dist = np.sqrt(real_dx**2 + real_dy**2)
            
            if pixel_dist > 0:  # 避免除零
                ratio = real_dist / pixel_dist
                ratios.append(ratio)
                
                # 記錄詳細信息
                distance_pairs.append({
                    'point1': i+1,
                    'point2': j+1,
                    'pixel_distance': pixel_dist,
                    'real_distance': real_dist,
                    'ratio': ratio
                })
                
                print(f"點{i+1} 到 點{j+1}:")
                print(f"  像素距離: {pixel_dist:.2f} pixels")
                print(f"  現實距離: {real_dist:.2f} mm")
                print(f"  比例係數: {ratio:.4f} mm/pixel")
                print()
    
    # 計算統計結果
    average_ratio = np.mean(ratios)
    std_ratio = np.std(ratios)
    min_ratio = np.min(ratios)
    max_ratio = np.max(ratios)
    
    # 計算變異係數
    cv_ratio = std_ratio / average_ratio if average_ratio > 0 else 0
    
    print("=== 統計結果 ===")
    print(f"平均比例係數: {average_ratio:.4f} mm/pixel")
    print(f"標準差: {std_ratio:.4f} mm/pixel")
    print(f"最小值: {min_ratio:.4f} mm/pixel")
    print(f"最大值: {max_ratio:.4f} mm/pixel")
    print(f"變異係數: {cv_ratio:.4f}")
    print()
    
    # 評估精度
    if cv_ratio < 0.05:
        precision_level = "優秀"
    elif cv_ratio < 0.1:
        precision_level = "良好"
    elif cv_ratio < 0.2:
        precision_level = "可接受"
    else:
        precision_level = "需要改善"
    
    print(f"精度評估: {precision_level}")
    print()
    
    # 詳細結果
    details = {
        'ratios': ratios,
        'distance_pairs': distance_pairs,
        'average_ratio': average_ratio,
        'std_ratio': std_ratio,
        'min_ratio': min_ratio,
        'max_ratio': max_ratio,
        'cv_ratio': cv_ratio,
        'precision_level': precision_level,
        'num_points': len(pixel_coords),
        'num_pairs': len(ratios)
    }
    
    return average_ratio, std_ratio, details

def visualize_ratio_distribution(details: Dict):
    """視覺化比例係數分布"""
    
    ratios = details['ratios']
    
    plt.figure(figsize=(12, 8))
    
    # 子圖1: 比例係數分布
    plt.subplot(2, 2, 1)
    plt.hist(ratios, bins=10, alpha=0.7, color='blue', edgecolor='black')
    plt.axvline(details['average_ratio'], color='red', linestyle='--', 
                label=f'average: {details["average_ratio"]:.4f}')
    plt.xlabel('ratio (mm/pixel)')
    plt.ylabel('frequency')
    plt.title('ratio distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 子圖2: 點對距離關係
    plt.subplot(2, 2, 2)
    pixel_distances = [pair['pixel_distance'] for pair in details['distance_pairs']]
    real_distances = [pair['real_distance'] for pair in details['distance_pairs']]
    
    plt.scatter(pixel_distances, real_distances, alpha=0.7, color='green')
    
    # 擬合直線
    z = np.polyfit(pixel_distances, real_distances, 1)
    p = np.poly1d(z)
    plt.plot(pixel_distances, p(pixel_distances), "r--", alpha=0.8)
    
    plt.xlabel('pixel distance (pixels)')
    plt.ylabel('world distance (mm)')
    plt.title('pixel vs world')
    plt.grid(True, alpha=0.3)
    
    # 子圖3: 比例係數箱線圖
    plt.subplot(2, 2, 3)
    plt.boxplot(ratios, vert=True)
    plt.ylabel('ratio (mm/pixel)')
    plt.title('ratio boxplot')
    plt.grid(True, alpha=0.3)
    
    # 子圖4: 誤差分析
    plt.subplot(2, 2, 4)
    errors = [abs(ratio - details['average_ratio']) for ratio in ratios]
    plt.plot(range(1, len(errors)+1), errors, 'o-', color='orange')
    plt.xlabel('point pair number')
    plt.ylabel('error (mm/pixel)')
    plt.title('error analysis')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def calculate_confidence_interval(ratios: List[float], confidence: float = 0.95) -> Tuple[float, float]:
    """計算信賴區間"""
    
    n = len(ratios)
    mean = np.mean(ratios)
    std = np.std(ratios, ddof=1)  # 樣本標準差
    
    # t分布臨界值（簡化計算，實際應使用scipy.stats.t）
    if confidence == 0.95:
        t_critical = 2.776 if n == 4 else 2.262 if n == 10 else 1.96
    else:
        t_critical = 1.96  # 簡化
    
    margin_error = t_critical * std / np.sqrt(n)
    
    lower_bound = mean - margin_error
    upper_bound = mean + margin_error
    
    return lower_bound, upper_bound

def main():
    """主函數 - 使用範例"""
    
    # 範例數據（您需要替換為實際數據）
    pixel_coords = [
        (210.96, 243.45),  # 點1
        (495.33, 242.39),  # 點2
        (211.13, 414.77),  # 點3
        (496.48, 413.92)   # 點4
    ]
    
    # 您需要測量這4個點對應的現實座標（單位：mm）
    real_coords = [
        (212.75, -93.7),  # 點1的現實座標
        (-21.27, 327.05),  # 點2的現實座標
        (-47.62, -224.96),  # 點3的現實座標
        (-260.65, 178.92)   # 點4的現實座標
    ]
    
    print("=== 像素到長度比例係數計算 ===")
    print("請將 real_coords 替換為您實際測量的座標")
    print()
    
    try:
        # 計算比例係數
        avg_ratio, std_ratio, details = calculate_pixel_to_length_ratio_multiple(
            pixel_coords, real_coords
        )
        
        # 計算信賴區間
        lower_bound, upper_bound = calculate_confidence_interval(details['ratios'])
        
        print("=== 最終結果 ===")
        print(f"平均比例係數: {avg_ratio:.4f} mm/pixel")
        print(f"95% 信賴區間: [{lower_bound:.4f}, {upper_bound:.4f}] mm/pixel")
        print(f"精度等級: {details['precision_level']}")
        print()
        
        # 視覺化結果
        print("正在生成視覺化圖表...")
        visualize_ratio_distribution(details)
        
        return avg_ratio, std_ratio, details
        
    except Exception as e:
        print(f"計算過程中發生錯誤: {e}")
        return None, None, None

if __name__ == "__main__":
    # 執行計算
    avg_ratio, std_ratio, details = main()
    
    if avg_ratio is not None:
        print(f"\n計算完成！")
        print(f"建議使用的比例係數: {avg_ratio:.4f} mm/pixel")
        print(f"標準差: {std_ratio:.4f} mm/pixel")