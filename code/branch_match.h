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
    std::pair<ClusterInfo, ClusterInfo> matched_pair; // �洢ƥ���������Ե��ƶ�,source target
    double lnscore_pair;//�洢��Ե÷�
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
    // ���ƥ��ĵ��ƶ�
    void AddMatchedClusters(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, double lnscore, double score);

    //// ��ȡ����ƥ��ĵ��ƶ�
    //const std::vector<std::pair<ClusterInfo, ClusterInfo>>& GetMatchedClusters() const;

    // ��ȡ����ƥ��ĵ��ƶԼ���÷�
    const std::vector<branch_pair>& GetMatchedClustersWithScores() const;

    //�����ڵ�ȶ�������Խ�������
    void SortMatchedClusters(int j);

    //�Ӵ�����ȡ���ƶ�
    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractClusterCloud
    (const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
        const pcl::PointIndices& cluster_indices);

    //������ƶ�Ϊpcd
    void SaveMatchedClustersAsPCDs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud);

private:
    std::vector<branch_pair> pair_;
    //std::vector<std::pair<ClusterInfo, ClusterInfo>> matched_clusters_; // �洢ƥ���������Ե��ƶ�
};

//����Ѱ��ƥ����֦�����ݺ����ɼнǡ��뾶�����ĵ���Ե����Z����Ըߣ�
class BranchMatcher
{
public:
    // ����Դ���ƺ�Ŀ����Ƶ�sorted_clusters
    void SetSortedClusters(const std::vector<ClusterInfo>& source_clusters, const std::vector<ClusterInfo>& target_clusters);

    // ִ��ƥ�������ɸѡ��������ͬһ����֦���������ƶ�
    MatchingResult MatchBranches(const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const double tol_z,int m);

private:
    // �ж��������ƴ��Ƿ�Ϊͬһ����֦�����ݰ뾶���нǡ���Ե���߶ȵȣ�
    bool IsSameBranch(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster,
        const CylinderParams& trunk_cylinder_params_s,
        const CylinderParams& trunk_cylinder_params_t,
        const double tol_r, const double tol_theta, const float tol_z, double& lnscore, double& score);
    std::vector<ClusterInfo> source_clusters_; // ���Դ ��֦ ���Ƶ�sorted_clusters
    std::vector<ClusterInfo> target_clusters_; // ���Ŀ�� ��֦ ���Ƶ�sorted_clusters
};

#endif