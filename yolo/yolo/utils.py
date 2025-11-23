"""
疏苗算法數學計算工具模組
包含距離計算、過密群判斷、疏除決策等數學運算功能
"""

import numpy as np
from typing import List, Tuple, Dict, Set
import logging


class ThinningUtils:
    """疏苗算法數學計算工具類別"""
    
    def __init__(self, min_plant_distance: float = 152.07):
        """
        初始化疏苗工具
        
        Args:
            min_plant_distance: 最小株距（像素），預設 152.07 pixels (15cm)
        """
        self.min_plant_distance = min_plant_distance
        self.logger = logging.getLogger(__name__)
    
    def calculate_distance_matrix(self, plant_coords: List[List[float]]) -> np.ndarray:
        """
        計算所有植株間的歐式距離矩陣（像素座標）
        
        Args:
            plant_coords: 植株像素座標列表，格式為 [[u1, v1], [u2, v2], ...]
            
        Returns:
            N×N 距離矩陣，其中 N 為植株數量
        """
        if not plant_coords:
            return np.array([])
        
        coords_array = np.array(plant_coords)
        n_plants = len(coords_array)
        
        # 初始化距離矩陣
        distance_matrix = np.zeros((n_plants, n_plants))
        
        # 計算每對植株間的歐式距離
        for i in range(n_plants):
            for j in range(n_plants):
                if i == j:
                    distance_matrix[i][j] = 0.0  # 對角線為0
                else:
                    # 計算歐式距離（像素距離）
                    dist = np.sqrt(
                        (coords_array[i][0] - coords_array[j][0])**2 + 
                        (coords_array[i][1] - coords_array[j][1])**2
                    )
                    distance_matrix[i][j] = dist
        
        self.logger.debug(f"距離矩陣計算完成，大小: {distance_matrix.shape}")
        return distance_matrix
    
    def find_dense_groups(self, distance_matrix: np.ndarray) -> List[Set[int]]:
        """
        根據距離矩陣找出過密群組
        
        Args:
            distance_matrix: N×N 距離矩陣
            
        Returns:
            過密群組列表，每個群組包含植株索引
        """
        if distance_matrix.size == 0:
            return []
        
        n_plants = distance_matrix.shape[0]
        visited = set()
        dense_groups = []
        
        for i in range(n_plants):
            if i in visited:
                continue
                
            # 找出與植株 i 過近的所有植株
            dense_group = {i}
            visited.add(i)
            
            # 使用廣度優先搜尋找出所有相關的過密植株
            queue = [i]
            while queue:
                current = queue.pop(0)
                for j in range(n_plants):
                    if (j not in visited and 
                        j != current and 
                        distance_matrix[current][j] < self.min_plant_distance):
                        dense_group.add(j)
                        visited.add(j)
                        queue.append(j)
            
            # 只保留包含多於一株植株的群組
            if len(dense_group) > 1:
                dense_groups.append(dense_group)
                self.logger.info(f"發現過密群組: {dense_group}，包含 {len(dense_group)} 株植株")
        
        return dense_groups
    
    def make_thinning_decision(self, dense_groups: List[Set[int]], 
                             plant_confidences: List[float]) -> Tuple[List[int], List[int]]:
        """
        對過密群組進行疏除決策（基於距離和信心值）
        
        Args:
            dense_groups: 過密群組列表
            plant_confidences: 每株植株的信心值列表
            
        Returns:
            (保留植株索引列表, 疏除植株索引列表)
        """
        keep_list = []
        remove_list = []
        
        # 處理每個過密群組
        for group_idx, group in enumerate(dense_groups):
            if len(group) <= 1:
                # 單株植株直接保留
                keep_list.extend(list(group))
                continue
            
            # 找出群組中信心值最高的植株
            group_confidences = [(idx, plant_confidences[idx]) for idx in group]
            group_confidences.sort(key=lambda x: x[1], reverse=True)
            
            # 信心值分析
            confidences = [conf for _, conf in group_confidences]
            avg_conf = np.mean(confidences)
            max_conf = max(confidences)
            min_conf = min(confidences)
            conf_std = np.std(confidences)
            
            self.logger.info(f"群組 {group_idx + 1} 信心值分析:")
            self.logger.info(f"  平均信心值: {avg_conf:.3f}")
            self.logger.info(f"  最高信心值: {max_conf:.3f}")
            self.logger.info(f"  最低信心值: {min_conf:.3f}")
            self.logger.info(f"  標準差: {conf_std:.3f}")
            
            # 決策策略：保留信心值最高的植株
            best_plant_idx = group_confidences[0][0]
            best_confidence = group_confidences[0][1]
            keep_list.append(best_plant_idx)
            
            # 其餘植株標記為疏除
            removed_plants = []
            for idx, conf in group_confidences[1:]:
                remove_list.append(idx)
                removed_plants.append((idx, conf))
            
            self.logger.info(f"群組 {group_idx + 1} 決策結果:")
            self.logger.info(f"  保留植株 {best_plant_idx} (信心值: {best_confidence:.3f})")
            for idx, conf in removed_plants:
                self.logger.info(f"  疏除植株 {idx} (信心值: {conf:.3f})")
        
        # 處理不在任何過密群組中的植株（直接保留）
        all_grouped_plants = set()
        for group in dense_groups:
            all_grouped_plants.update(group)
        
        isolated_plants = []
        for i in range(len(plant_confidences)):
            if i not in all_grouped_plants:
                keep_list.append(i)
                isolated_plants.append((i, plant_confidences[i]))
        
        if isolated_plants:
            self.logger.info("孤立植株（直接保留）:")
            for idx, conf in isolated_plants:
                self.logger.info(f"  植株 {idx} (信心值: {conf:.3f})")
        
        self.logger.info(f"疏除決策完成: 保留 {len(keep_list)} 株，疏除 {len(remove_list)} 株")
        return keep_list, remove_list
    
    def filter_plants_by_confidence(self, plant_data: List[Dict], 
                                  min_confidence: float = 0.5) -> List[Dict]:
        """
        根據信心值過濾植株資料
        
        Args:
            plant_data: 植株資料列表，每個元素包含 'coords', 'confidence' 等欄位
            min_confidence: 最小信心值閾值
            
        Returns:
            過濾後的植株資料列表
        """
        filtered_plants = []
        for i, plant in enumerate(plant_data):
            if plant.get('confidence', 0.0) >= min_confidence:
                filtered_plants.append(plant)
            else:
                self.logger.debug(f"植株 {i} 信心值 {plant.get('confidence', 0.0):.3f} 低於閾值，已過濾")
        
        self.logger.info(f"信心值過濾: {len(plant_data)} -> {len(filtered_plants)} 株植株")
        return filtered_plants
    
    def calculate_group_statistics(self, dense_groups: List[Set[int]], 
                                 plant_coords: List[List[float]]) -> Dict:
        """
        計算過密群組統計資訊
        
        Args:
            dense_groups: 過密群組列表
            plant_coords: 植株座標列表
            
        Returns:
            統計資訊字典
        """
        stats = {
            'total_groups': len(dense_groups),
            'total_plants_in_groups': sum(len(group) for group in dense_groups),
            'group_sizes': [len(group) for group in dense_groups],
            'average_group_size': 0,
            'max_group_size': 0,
            'min_group_size': 0
        }
        
        if dense_groups:
            stats['average_group_size'] = np.mean(stats['group_sizes'])
            stats['max_group_size'] = max(stats['group_sizes'])
            stats['min_group_size'] = min(stats['group_sizes'])
        
        return stats
    
    def validate_coordinates(self, coords: List[float]) -> bool:
        """
        驗證座標是否有效
        
        Args:
            coords: 座標列表 [x, y, z]
            
        Returns:
            座標是否有效
        """
        if len(coords) != 3:
            return False
        
        # 檢查是否為有限數值
        return all(np.isfinite(coord) for coord in coords)
    
    def calculate_plant_density(self, plant_coords: List[List[float]], 
                              area_bounds: Tuple[float, float, float, float]) -> float:
        """
        計算植株密度（株/平方公尺）
        
        Args:
            plant_coords: 植株座標列表
            area_bounds: 區域邊界 (x_min, x_max, y_min, y_max)
            
        Returns:
            植株密度
        """
        if not plant_coords:
            return 0.0
        
        x_min, x_max, y_min, y_max = area_bounds
        area = (x_max - x_min) * (y_max - y_min)
        
        if area <= 0:
            return 0.0
        
        density = len(plant_coords) / area
        return density


def create_plant_data(coords: List[float], confidence: float, 
                     plant_id: int = None) -> Dict:
    """
    建立標準化的植株資料結構
    
    Args:
        coords: 植株座標 [x, y, z]
        confidence: 信心值
        plant_id: 植株ID（可選）
        
    Returns:
        標準化的植株資料字典
    """
    return {
        'coords': coords,
        'confidence': confidence,
        'plant_id': plant_id,
        'is_keep': True,  # 預設保留
        'is_remove': False  # 預設不疏除
    }


def extract_coords_from_plant_data(plant_data: List[Dict]) -> List[List[float]]:
    """
    從植株資料中提取座標列表
    
    Args:
        plant_data: 植株資料列表
        
    Returns:
        座標列表
    """
    return [plant['coords'] for plant in plant_data]


def extract_confidences_from_plant_data(plant_data: List[Dict]) -> List[float]:
    """
    從植株資料中提取信心值列表
    
    Args:
        plant_data: 植株資料列表
        
    Returns:
        信心值列表
    """
    return [plant['confidence'] for plant in plant_data]
