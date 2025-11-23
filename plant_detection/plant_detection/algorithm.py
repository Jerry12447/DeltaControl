#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
疏苗算法模組
實現基於KDTree的分群和Hybrid疏除策略
"""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Tuple, Dict


class ThinningAlgorithm:
    """
    疏苗算法類
    實現基於KDTree的分群和Hybrid疏除策略
    """
    
    def __init__(self, 
                 threshold_px: float,
                 weight_conflict: float = 0.6,
                 weight_distance: float = 0.4):
        """
        初始化疏苗算法
        
        Args:
            threshold_px: 疏苗距離閾值（像素）
            weight_conflict: Hybrid策略中衝突數的權重 (0-1)
            weight_distance: Hybrid策略中距離的權重 (0-1)
        """
        self.threshold_px = threshold_px
        self.weight_conflict = weight_conflict
        self.weight_distance = weight_distance
    
    def apply_thinning(self, crop_detections: List[dict]) -> Tuple[List[int], List[int]]:
        """
        對作物檢測結果應用疏除演算法
        
        Args:
            crop_detections: 作物檢測結果列表，每個元素包含 'center' 鍵（中心點座標 [x, y]）
            
        Returns:
            (kept_indices, removed_indices): 保留和移除的索引列表（相對於 crop_detections）
        """
        if len(crop_detections) <= 1:
            return list(range(len(crop_detections))), []
        
        # 提取中心點座標
        points = np.array([det['center'] for det in crop_detections])
        
        # KDTree 分群
        clusters = self._cluster_with_kdtree(points, self.threshold_px)
        
        # 對每個群組執行疏除
        kept_indices = []
        removed_indices = []
        
        for cluster_id, cluster_indices in clusters.items():
            if len(cluster_indices) <= 1:
                # 單點群組，直接保留
                kept_indices.extend(cluster_indices)
            else:
                # 執行 Hybrid 疏除策略
                kept_in_cluster = self._thin_cluster(
                    cluster_indices, points, self.threshold_px,
                    weight_conflict=self.weight_conflict,
                    weight_distance=self.weight_distance
                )
                
                removed_in_cluster = [i for i in cluster_indices if i not in kept_in_cluster]
                kept_indices.extend(kept_in_cluster)
                removed_indices.extend(removed_in_cluster)
        
        return kept_indices, removed_indices
    
    def _cluster_with_kdtree(self, points: np.ndarray, threshold: float) -> Dict[int, List[int]]:
        """
        使用 KDTree 進行分群
        
        Args:
            points: 點座標陣列 (N, 2)
            threshold: 距離閾值（像素）
            
        Returns:
            群組字典 {群組ID: [點索引列表]}
        """
        tree = KDTree(points)
        
        # Union-Find 資料結構用於分群
        parent = list(range(len(points)))
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            root_x = find(x)
            root_y = find(y)
            if root_x != root_y:
                parent[root_y] = root_x
        
        # 找出所有距離小於閾值的點對並進行分群
        for i, p in enumerate(points):
            neighbors = tree.query_ball_point(p, threshold)
            for j in neighbors:
                if i < j:  # 避免重複
                    dist = np.linalg.norm(points[i] - points[j])
                    if dist < threshold:
                        union(i, j)
        
        # 找出每個點所屬的群組
        clusters = {}
        for i in range(len(points)):
            root = find(i)
            if root not in clusters:
                clusters[root] = []
            clusters[root].append(i)
        
        return clusters
    
    def _thin_cluster(self, 
                     cluster_indices: List[int], 
                     points: np.ndarray, 
                     threshold: float,
                     weight_conflict: float = 0.6,
                     weight_distance: float = 0.4) -> List[int]:
        """
        Args:
            cluster_indices: 群組內的點索引列表
            points: 所有點的座標陣列
            threshold: 距離閾值
            weight_conflict: 衝突數權重
            weight_distance: 距離權重
            
        Returns:
            保留的點索引列表
        """
        if len(cluster_indices) <= 1:
            return cluster_indices.copy()
        
        # 建立衝突列表和距離字典
        conflicts = {}
        distances = {}
        
        for i in cluster_indices:
            conflicts[i] = []
            distances[i] = {}
        
        # 計算群組內所有點對的距離
        for i in cluster_indices:
            for j in cluster_indices:
                if i < j:
                    dist = np.linalg.norm(points[i] - points[j])
                    distances[i][j] = dist
                    distances[j][i] = dist
                    if dist < threshold:
                        conflicts[i].append(j)
                        conflicts[j].append(i)
        
        # 當前保留的點集合
        remaining = set(cluster_indices)
        
        # 迭代移除點，直到沒有衝突
        while True:
            conflict_counts = {}
            avg_distances = {}
            
            for i in remaining:
                # 計算衝突數
                count = sum(1 for j in conflicts[i] if j in remaining)
                if count > 0:
                    conflict_counts[i] = count
                
                # 計算平均距離
                remaining_neighbors = [j for j in remaining if j != i]
                if remaining_neighbors:
                    avg_dist = np.mean([distances[i][j] for j in remaining_neighbors])
                    avg_distances[i] = avg_dist
            
            # 如果沒有衝突了，結束
            if not conflict_counts:
                break
            
            # Hybrid 策略：綜合評估衝突數和平均距離
            # 正規化衝突數
            if conflict_counts:
                max_conflict = max(conflict_counts.values())
                min_conflict = min(conflict_counts.values())
                conflict_range = max_conflict - min_conflict if max_conflict != min_conflict else 1
                normalized_conflicts = {
                    i: (count - min_conflict) / conflict_range
                    for i, count in conflict_counts.items()
                }
            else:
                normalized_conflicts = {}
            
            # 正規化平均距離（距離越小，分數越高）
            if avg_distances:
                max_dist = max(avg_distances.values())
                min_dist = min(avg_distances.values())
                dist_range = max_dist - min_dist if max_dist != min_dist else 1
                normalized_distances = {
                    i: 1.0 - (dist - min_dist) / dist_range
                    for i, dist in avg_distances.items()
                }
            else:
                normalized_distances = {}
            
            # 計算混合分數
            scores = {}
            for i in conflict_counts.keys():
                conflict_score = normalized_conflicts.get(i, 0) * weight_conflict
                distance_score = normalized_distances.get(i, 0) * weight_distance
                scores[i] = conflict_score + distance_score
            
            # 選擇分數最高的點移除
            max_score = max(scores.values())
            candidates = [i for i, score in scores.items() if score == max_score]
            to_remove = candidates[0]
            
            remaining.remove(to_remove)
        
        return sorted(list(remaining))

