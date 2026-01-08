#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
疏苗算法模組
實現基於KDTree的分群和最大獨立集疏除策略
"""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Tuple, Dict


class ThinningAlgorithm:
    """
    疏苗算法類
    實現基於KDTree的分群和貪心最大獨立集疏除策略
    """
    
    def __init__(self, threshold_px: float):
        """
        初始化疏苗算法
        
        Args:
            threshold_px: 疏苗距離閾值（像素）
        """
        self.threshold_px = threshold_px
    
    def apply_thinning(self, crop_detections: List[dict]) -> Tuple[List[int], List[int], Dict[int, float]]:
        """
        對作物檢測結果應用疏除演算法
        
        Args:
            crop_detections: 作物檢測結果列表，每個元素包含 'center' 鍵（中心點座標 [x, y]）
            
        Returns:
            (kept_indices, removed_indices, evaluation_scores): 
            - kept_indices: 保留的索引列表
            - removed_indices: 移除的索引列表
            - evaluation_scores: 每個點的評估分數字典 {index: score}
               注意：評估分數反映的是點在第一次迭代時的初始風險評估。
               在迭代疏除過程中，某些點可能最初分數很高（如1.0），但在移除其他點後
               失去衝突，最終被保留。這是正常的行為，因為評估分數記錄的是初始狀態。
        """
        if len(crop_detections) <= 1:
            # 單點或無點，評估分數為0
            evaluation_scores = {i: 0.0 for i in range(len(crop_detections))}
            return list(range(len(crop_detections))), [], evaluation_scores
        
        # 提取中心點座標
        points = np.array([det['center'] for det in crop_detections])
        
        # KDTree 分群
        clusters = self._cluster_with_kdtree(points, self.threshold_px)
        
        # 初始化評估分數字典（所有點預設為0）
        evaluation_scores = {i: 0.0 for i in range(len(crop_detections))}
        
        # 對每個群組執行疏除
        kept_indices = []
        removed_indices = []
        
        for cluster_id, cluster_indices in clusters.items():
            if len(cluster_indices) <= 1:
                # 單點群組，直接保留，評估分數為0（非密集群）
                kept_indices.extend(cluster_indices)
            else:
                # 執行最大獨立集疏除策略，同時計算評估分數
                kept_in_cluster, cluster_scores = self._thin_cluster(
                    cluster_indices, points, self.threshold_px
                )
                
                # 更新評估分數
                for idx, score in cluster_scores.items():
                    evaluation_scores[idx] = score
                
                removed_in_cluster = [i for i in cluster_indices if i not in kept_in_cluster]
                kept_indices.extend(kept_in_cluster)
                removed_indices.extend(removed_in_cluster)
        
        return kept_indices, removed_indices, evaluation_scores
    
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
                     threshold: float) -> Tuple[List[int], Dict[int, float]]:
        """
        使用貪心最大獨立集算法對密集群組進行疏除
        
        Args:
            cluster_indices: 群組內的點索引列表
            points: 所有點的座標陣列
            threshold: 距離閾值
            
        Returns:
            (保留的點索引列表, 評估分數字典): 
            - 保留的點索引列表（最大獨立集）
            - 評估分數字典 {index: score}，包含所有群組內點的評估分數
               分數表示點在衝突圖中的度數（衝突數），度數越高表示越密集
        """
        if len(cluster_indices) <= 1:
            # 單點群組，評估分數為0
            scores = {cluster_indices[0]: 0.0} if cluster_indices else {}
            return cluster_indices.copy(), scores
        
        # 建立衝突圖（鄰接列表）
        conflicts = {}
        degrees = {}  # 每個點的度數（衝突數）
        
        for i in cluster_indices:
            conflicts[i] = []
            degrees[i] = 0
        
        # 計算群組內所有點對的距離，建立衝突圖
        for i in cluster_indices:
            for j in cluster_indices:
                if i < j:
                    dist = np.linalg.norm(points[i] - points[j])
                    if dist < threshold:
                        conflicts[i].append(j)
                        conflicts[j].append(i)
                        degrees[i] += 1
                        degrees[j] += 1
        
        # 初始化評估分數（使用度數作為評估分數）
        all_scores = {}
        for i in cluster_indices:
            # 正規化度數到 [0, 1] 範圍
            max_degree = max(degrees.values()) if degrees.values() else 1
            all_scores[i] = degrees[i] / max_degree if max_degree > 0 else 0.0
        
        # 貪心最大獨立集算法
        # 策略：每次選擇度數最小的點加入獨立集，然後移除它和所有與它衝突的點
        independent_set = []  # 最大獨立集（保留的點）
        remaining = set(cluster_indices)
        
        while remaining:
            # 從剩餘點中選擇度數最小的點（如果度數相同，選擇第一個）
            min_degree = min(degrees[i] for i in remaining)
            candidates = [i for i in remaining if degrees[i] == min_degree]
            
            # 選擇第一個候選點加入獨立集
            selected = candidates[0]
            independent_set.append(selected)
            
            # 移除選中的點和所有與它衝突的點
            to_remove = {selected}
            to_remove.update(conflicts[selected])
            
            # 更新剩餘點集合和度數
            for node in to_remove:
                if node in remaining:
                    remaining.remove(node)
                    # 更新與該點相鄰的點的度數
                    for neighbor in conflicts[node]:
                        if neighbor in remaining:
                            degrees[neighbor] -= 1
        
        return sorted(independent_set), all_scores

