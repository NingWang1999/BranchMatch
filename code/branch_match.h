#pragma once
#ifndef BRANCH_MATCH_H
#define BRANCH_MATCH_H

#include"pre_branch.h"
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <tuple>
#include <limits>
#include <algorithm>
#include <unordered_map>

struct branch_pair
{
    std::pair<ClusterInfo, ClusterInfo> matched_pair; // 存储匹配的所有配对点云对,source target
    double lnscore_pair;//存储配对得分
    double score_pair;
};

struct Comparator
{
    Comparator(int j_val) : j(j_val) {}

    bool operator()(const branch_pair& lhs, const branch_pair& rhs) const
    {
        if (j > 1)
        {
            return lhs.matched_pair.first.inlier_amount > rhs.matched_pair.first.inlier_amount;
        }
        else
        {
            return lhs.matched_pair.first.inlier_amount + lhs.matched_pair.second.inlier_amount  > rhs.matched_pair.first.inlier_amount + rhs.matched_pair.second.inlier_amount ;
        }
    }

    int j;
};

class MatchingResult
{
public:
    // 添加匹配的点云对
    void AddMatchedClusters(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, double lnscore, double score);

    //// 获取所有匹配的点云对
    //const std::vector<std::pair<ClusterInfo, ClusterInfo>>& GetMatchedClusters() const;

    // 获取所有匹配的点云对及其得分
    const std::vector<branch_pair>& GetMatchedClustersWithScores() const;

    //根据内点比对所有配对进行排序
    void SortMatchedClusters(int j);

    //从簇中提取点云对
    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractClusterCloud
    (const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
        const pcl::PointIndices& cluster_indices);

    //保存点云对为pcd
    void SaveMatchedClustersAsPCDs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud);

private:
    std::vector<branch_pair> pair_;
    //std::vector<std::pair<ClusterInfo, ClusterInfo>> matched_clusters_; // 存储匹配的所有配对点云对
};

//用于寻找匹配树枝（根据和树干夹角、半径、中心点相对地面的Z轴相对高）
class BranchMatcher
{
public:
    // 设置源点云和目标点云的sorted_clusters
    void SetSortedClusters(const std::vector<ClusterInfo>& source_clusters, const std::vector<ClusterInfo>& target_clusters);

    // 执行匹配操作，筛选出真正的同一个树枝的两个点云对
    MatchingResult MatchBranches(const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const double tol_z,int m);

private:
    // 判断两个点云簇是否为同一个树枝（根据半径、夹角、相对地面高度等）
    bool IsSameBranch(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster,
        const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const float tol_z, double& lnscore, double& score);
    std::vector<ClusterInfo> source_clusters_; // 存放源 树枝 点云的sorted_clusters
    std::vector<ClusterInfo> target_clusters_; // 存放目标 树枝 点云的sorted_clusters
};

#endif