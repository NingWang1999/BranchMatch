#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <random>

// ===================== 预处理参数（基准值，实际使用自适应计算）=====================
// 这些是 V_BASE=0.008m 密度下的基准参数，实际参数由 DensityScaledParams 计算
struct PreprocessParams {
    // 体素滤波 (A类: × K)
    static constexpr float VOXEL_SIZE_BASE = 0.008f;
    
    // RANSAC地面检测 (固定参数)
    static constexpr int RANSAC_MAX_ITER = 10000;
    static constexpr float RANSAC_DIST_THRESH = 0.15f;
    
    // 统计滤波 (B类: ÷ K²)
    static constexpr float STAT_K_BASE = 20.0f;  // sor_mean_k = max(5, 20/K²)
    static constexpr float STAT_STD_MUL = 0.1f;  // 固定
    
    // 高度滤波 (现由 TreeHeightParams 自适应计算)
    // height_min = H_tree * 0.15, height_max = H_tree * 0.50
    
    // 聚类参数 (A类容差 × K，B类点数 ÷ K²)
    static constexpr float CLUSTER_TOL_BASE = 0.06f;
    static constexpr float CLUSTER_MIN_BASE = 1000.0f;
    static constexpr float CLUSTER_MAX_BASE = 1000000.0f;
    
    // 法线估计 (A类: × K)
    static constexpr float NORMAL_RADIUS_BASE = 0.03f;
    
    // MLS表面重建 (A类: × K)
    static constexpr int MLS_POLYNOMIAL_ORDER = 2;  // 固定
    static constexpr float MLS_RADIUS_BASE = 0.03f;
};

// ===================== 树枝分割参数（自适应版本）=====================
// 所有参数现在都由自适应算法计算，此结构体仅保留比例系数常量
struct BranchSegmentParams {
    // ===== 环形分割：基于主干半径的比例系数 =====
    // 基准：R_trunk=3.5cm时，r_min=7cm, r_max=14cm
    static constexpr float RING_RMIN_RATIO = 2.0f;   // r_min = R_trunk * 2.0
    static constexpr float RING_RMAX_RATIO = 4.0f;   // r_max = R_trunk * 4.0
    
    // ===== 高度区域：基于树高的比例系数 =====
    // 休眠期高纺锤形苹果树一级枝集中在主干中下部
    static constexpr float HEIGHT_ZMIN_RATIO = 0.15f;  // z_min = H_tree * 0.15
    static constexpr float HEIGHT_ZMAX_RATIO = 0.50f;  // z_max = H_tree * 0.50
    
    // ===== 聚类参数：密度自适应基准值 =====
    // 实际值由 DensityScaledParams 计算
    static constexpr float CLUSTER_TOL_BASE = 0.01f;  // cluster_tol = 0.01 * K
    static constexpr float CLUSTER_MIN_BASE = 150.0f; // cluster_min = max(20, 150/K²)
    static constexpr float CLUSTER_MAX_BASE = 1500.0f;// cluster_max = max(100, 1500/K²)
    
    // 计算环形分割半径参数
    static void computeRingParams(double trunk_radius, float& r_min, float& r_max) {
        r_min = static_cast<float>(trunk_radius) * RING_RMIN_RATIO;
        r_max = static_cast<float>(trunk_radius) * RING_RMAX_RATIO;
    }
};



// ===================== 树高自适应参数结构体 =====================
// 基于树高计算的相对高度阈值（比例来自 BranchSegmentParams）
struct TreeHeightParams {
    float H_tree = 2.0f;       // 树高 (z_max - z_min)
    float z_ground = 0.0f;     // 地面高度 (z_min)
    
    // 一级枝区域（相对于地面的高度）
    // 比例系数定义在 BranchSegmentParams::HEIGHT_ZMIN/ZMAX_RATIO
    float branch_z_min;        // H_tree * HEIGHT_ZMIN_RATIO (15%)
    float branch_z_max;        // H_tree * HEIGHT_ZMAX_RATIO (50%)
    
    // 打印参数信息
    void print() const {
        std::cout << "[树高参数] H_tree=" << H_tree << "m, 一级枝相对高度=[" 
                  << branch_z_min << ", " << branch_z_max << "]m (占比15%-50%)" << std::endl;
    }
};

// ===================== 密度自适应缩放参数结构体 =====================
// 根据尺度因子K计算出的所有动态参数（基准值来自 PreprocessParams 和 BranchSegmentParams）
struct DensityScaledParams {
    // 尺度因子
    float K = 1.0f;
    float d_mean = 0.008f;  // 平均点距
    
    // A. 距离/半径类参数（与K成正比）
    float voxel_size;           // PreprocessParams::VOXEL_SIZE_BASE * K
    float cluster_tolerance;    // PreprocessParams::CLUSTER_TOL_BASE * K
    float normal_radius;        // PreprocessParams::NORMAL_RADIUS_BASE * K
    float mls_radius;           // PreprocessParams::MLS_RADIUS_BASE * K
    float branch_cluster_tol;   // BranchSegmentParams::CLUSTER_TOL_BASE * K
    
    // B. 点数阈值类参数（与K²成反比）
    int cluster_min;            // max(20, BranchSegmentParams::CLUSTER_MIN_BASE / K²)
    int cluster_max;            // max(100, BranchSegmentParams::CLUSTER_MAX_BASE / K²)
    int sor_mean_k;             // max(5, PreprocessParams::STAT_K_BASE / K²)
    
    // 打印参数信息
    void print() const {
        std::cout << "\n======== 密度自适应参数 ========" << std::endl;
        std::cout << "[密度] 平均点距 d_mean = " << d_mean << " m" << std::endl;
        std::cout << "[密度] 尺度因子 K = " << K << std::endl;
        std::cout << "[A类-距离] voxel_size = " << voxel_size << std::endl;
        std::cout << "[A类-距离] cluster_tolerance = " << cluster_tolerance << std::endl;
        std::cout << "[A类-距离] normal_radius = " << normal_radius << std::endl;
        std::cout << "[A类-距离] mls_radius = " << mls_radius << std::endl;
        std::cout << "[A类-距离] branch_cluster_tol = " << branch_cluster_tol << std::endl;
        std::cout << "[B类-点数] cluster_min = " << cluster_min << std::endl;
        std::cout << "[B类-点数] cluster_max = " << cluster_max << std::endl;
        std::cout << "[B类-点数] sor_mean_k = " << sor_mean_k << std::endl;
        std::cout << "================================\n" << std::endl;
    }
};

// ===================== 自适应参数计算器 =====================
class AdaptiveParams {
public:
    // 基准分辨率常量
    static constexpr float V_BASE = 0.008f;
    
    // ===================== 计算平均点距（使用KD-Tree随机采样） =====================
    // 采样 sample_count 个点，计算每个点到最近邻的距离，返回平均值
    static float computeMeanPointDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                           int sample_count = 1000) {
        if (!cloud || cloud->size() < 10) {
            return V_BASE;  // 点云太小，返回基准值
        }
        
        // 构建KD-Tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        
        // 随机采样点的索引
        int actual_samples = std::min(sample_count, static_cast<int>(cloud->size()));
        std::vector<int> sample_indices(cloud->size());
        std::iota(sample_indices.begin(), sample_indices.end(), 0);
        
        // 随机打乱
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(sample_indices.begin(), sample_indices.end(), gen);
        
        // 计算采样点到最近邻的距离
        double total_distance = 0.0;
        int valid_count = 0;
        std::vector<int> k_indices(2);
        std::vector<float> k_distances(2);
        
        for (int i = 0; i < actual_samples; ++i) {
            int idx = sample_indices[i];
            const auto& pt = cloud->points[idx];
            
            // 跳过无效点
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            
            // 搜索2个最近邻（第一个是自己）
            if (kdtree.nearestKSearch(pt, 2, k_indices, k_distances) == 2) {
                // k_distances[1] 是到第二近邻（真正的最近邻）的平方距离
                if (k_distances[1] > 0) {
                    total_distance += std::sqrt(k_distances[1]);
                    valid_count++;
                }
            }
        }
        
        if (valid_count < 10) {
            return V_BASE;  // 有效点太少
        }
        
        return static_cast<float>(total_distance / valid_count);
    }
    
    // ===================== 计算尺度因子K =====================
    // K = R_actual / V_base, 其中 R_actual = max(d_mean, V_base)
    static float computeScaleFactor(float d_mean) {
        float R_actual = std::max(d_mean, V_BASE);
        float K = R_actual / V_BASE;
        return K;
    }
    
    // ===================== 根据尺度因子K计算所有密度缩放参数 =====================
    // 使用 PreprocessParams 和 BranchSegmentParams 中定义的基准常量
    static DensityScaledParams computeDensityScaledParams(float d_mean) {
        DensityScaledParams params;
        
        params.d_mean = d_mean;
        params.K = computeScaleFactor(d_mean);
        float K = params.K;
        float K2 = K * K;  // K的平方
        
        // A. 距离/半径类参数（与K成正比）
        params.voxel_size = PreprocessParams::VOXEL_SIZE_BASE * K;
        params.cluster_tolerance = PreprocessParams::CLUSTER_TOL_BASE * K;
        params.normal_radius = PreprocessParams::NORMAL_RADIUS_BASE * K;
        params.mls_radius = PreprocessParams::MLS_RADIUS_BASE * K;
        params.branch_cluster_tol = BranchSegmentParams::CLUSTER_TOL_BASE * K;
        
        // B. 点数阈值类参数（与K²成反比）
        params.cluster_min = std::max(20, static_cast<int>(BranchSegmentParams::CLUSTER_MIN_BASE / K2));
        params.cluster_max = std::max(100, static_cast<int>(BranchSegmentParams::CLUSTER_MAX_BASE / K2));
        params.sor_mean_k = std::max(5, static_cast<int>(PreprocessParams::STAT_K_BASE / K2));
        
        return params;
    }
    
    // ===================== 一站式：从点云直接计算缩放参数 =====================
    static DensityScaledParams computeParamsFromCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int sample_count = 1000) 
    {
        float d_mean = computeMeanPointDistance(cloud, sample_count);
        return computeDensityScaledParams(d_mean);
    }
    
    // ===================== 计算树高自适应参数 =====================
    // 从去地面后的点云计算树高，并返回一级枝区域的相对高度阈值
    // 使用 BranchSegmentParams 中定义的比例常量
    static TreeHeightParams computeTreeHeightParams(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float branch_ratio_min = BranchSegmentParams::HEIGHT_ZMIN_RATIO,
        float branch_ratio_max = BranchSegmentParams::HEIGHT_ZMAX_RATIO)
    {
        TreeHeightParams params;
        
        if (!cloud || cloud->empty()) {
            // 默认值（假设树高2米）
            params.H_tree = 2.0f;
            params.z_ground = 0.0f;
            params.branch_z_min = 2.0f * BranchSegmentParams::HEIGHT_ZMIN_RATIO;
            params.branch_z_max = 2.0f * BranchSegmentParams::HEIGHT_ZMAX_RATIO;
            return params;
        }
        
        // 使用PCL获取点云Z轴范围
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        
        params.z_ground = min_pt.z;
        params.H_tree = max_pt.z - min_pt.z;
        
        // 防止树高过小导致的异常
        if (params.H_tree < 0.5f) {
            params.H_tree = 2.0f;  // 最小假设树高
        }
        
        // 计算一级枝区域的相对高度（直接替换原来的 0.5, 1.5）
        params.branch_z_min = params.H_tree * branch_ratio_min;
        params.branch_z_max = params.H_tree * branch_ratio_max;
        
        return params;
    }


};

#endif // CONFIG_H
