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

// ===================== 树枝匹配参数 =====================
struct BranchMatchParams {
    double tol_r = 0.005;       // 半径容差
    double tol_theta = 0.262;   // 角度容差（约15度）
    double tol_z = 0.05;        // 高度容差

    // RANSAC几何一致性筛选
    double ransac_inlier_thresh = 0.15;  // 内点阈值（弧度）
    int ransac_max_iter = 100;

    // 代价矩阵权重
    double weight_radius = 1.0;
    double weight_angle = 1.0;
    double weight_height = 1.0;
};

// ===================== 圆柱拟合参数 =====================
struct CylinderFitParams {
    float normal_weight = 0.01f;
    int max_iter = 10000;
    double dist_thresh = 0.01;
    double max_radius = 0.2;
};

// ===================== ICP参数 =====================
struct ICPParams {
    double convergence_thresh = 1e-7;
    int max_iter = 1000;
    int max_correspondences = 80000;
    double overlap_ratio = 0.9;
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

    // 基于树枝半径分布自适应匹配容差
    template<typename ClusterInfoType>
    static double computeRadiusTolerance(const std::vector<ClusterInfoType>& clusters,
        double default_tol = 0.005) {
        if (clusters.empty()) return default_tol;

        std::vector<double> radii;
        radii.reserve(clusters.size());
        for (const auto& c : clusters) {
            radii.push_back(c.center_axis_radius.radius);
        }

        if (radii.size() < 3) return default_tol;

        // 使用 MAD (Median Absolute Deviation) 估计
        std::sort(radii.begin(), radii.end());
        double median = radii[radii.size() / 2];

        std::vector<double> abs_devs;
        abs_devs.reserve(radii.size());
        for (double r : radii) {
            abs_devs.push_back(std::abs(r - median));
        }
        std::sort(abs_devs.begin(), abs_devs.end());

        double mad = abs_devs[abs_devs.size() / 2] * 1.4826; // 转换为标准差估计
        return std::max(0.003, std::min(mad * 2.0, 0.01));
    }

    // 基于角度分布自适应角度容差
    template<typename ClusterInfoType>
    static double computeAngleTolerance(const std::vector<ClusterInfoType>& clusters,
        const Eigen::Vector3d& trunk_axis,
        double default_tol = 0.262) {
        if (clusters.size() < 3) return default_tol;

        std::vector<double> angles;
        angles.reserve(clusters.size());
        for (const auto& c : clusters) {
            double dot = std::max(-1.0, std::min(c.center_axis_radius.axis.dot(trunk_axis), 1.0));
            angles.push_back(std::acos(dot));
        }

        std::sort(angles.begin(), angles.end());
        double median = angles[angles.size() / 2];

        std::vector<double> abs_devs;
        abs_devs.reserve(angles.size());
        for (double a : angles) {
            abs_devs.push_back(std::abs(a - median));
        }
        std::sort(abs_devs.begin(), abs_devs.end());

        double mad = abs_devs[abs_devs.size() / 2] * 1.4826;
        return std::max(0.1, std::min(mad * 2.5, 0.5)); // 约5.7度到28.6度
    }

    // 基于高度分布自适应高度容差
    template<typename ClusterInfoType>
    static double computeHeightTolerance(const std::vector<ClusterInfoType>& clusters,
        double default_tol = 0.05) {
        if (clusters.size() < 3) return default_tol;

        std::vector<double> heights;
        heights.reserve(clusters.size());
        for (const auto& c : clusters) {
            heights.push_back(static_cast<double>(c.z_relative));
        }

        std::sort(heights.begin(), heights.end());
        double median = heights[heights.size() / 2];

        std::vector<double> abs_devs;
        abs_devs.reserve(heights.size());
        for (double h : heights) {
            abs_devs.push_back(std::abs(h - median));
        }
        std::sort(abs_devs.begin(), abs_devs.end());

        double mad = abs_devs[abs_devs.size() / 2] * 1.4826;
        return std::max(0.02, std::min(mad * 2.0, 0.1));
    }

    // 综合计算自适应匹配参数
    template<typename ClusterInfoType>
    static BranchMatchParams computeAdaptiveMatchParams(
        const std::vector<ClusterInfoType>& source_clusters,
        const std::vector<ClusterInfoType>& target_clusters,
        const Eigen::Vector3d& trunk_axis_s,
        const Eigen::Vector3d& trunk_axis_t)
    {
        BranchMatchParams params;

        // 从两组聚类中计算自适应参数，取较宽松的值
        double tol_r_s = computeRadiusTolerance(source_clusters);
        double tol_r_t = computeRadiusTolerance(target_clusters);
        params.tol_r = std::max(tol_r_s, tol_r_t);

        double tol_theta_s = computeAngleTolerance(source_clusters, trunk_axis_s);
        double tol_theta_t = computeAngleTolerance(target_clusters, trunk_axis_t);
        params.tol_theta = std::max(tol_theta_s, tol_theta_t);

        double tol_z_s = computeHeightTolerance(source_clusters);
        double tol_z_t = computeHeightTolerance(target_clusters);
        params.tol_z = std::max(tol_z_s, tol_z_t);

        return params;
    }
};

#endif // CONFIG_H
