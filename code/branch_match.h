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
    std::pair<ClusterInfo, ClusterInfo> matched_pair; // source target
    double lnscore_pair;//save score
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
    void AddMatchedClusters(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, double lnscore, double score);

    const std::vector<branch_pair>& GetMatchedClustersWithScores() const;

    void SortMatchedClusters(int j);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractClusterCloud
    (const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
        const pcl::PointIndices& cluster_indices);

    void SaveMatchedClustersAsPCDs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud);

private:
    std::vector<branch_pair> pair_;
};

class BranchMatcher
{
public:
    void SetSortedClusters(const std::vector<ClusterInfo>& source_clusters, const std::vector<ClusterInfo>& target_clusters);

    MatchingResult MatchBranches(const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const double tol_z,int m);

private:
    bool IsSameBranch(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster,
        const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const float tol_z, double& lnscore, double& score);
    std::vector<ClusterInfo> source_clusters_;
    std::vector<ClusterInfo> target_clusters_; 
};

#endif
