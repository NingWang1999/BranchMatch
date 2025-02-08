#include "branch_match.h"

// 添加匹配的点云对
void MatchingResult::AddMatchedClusters(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, double lnscore, double score)
{
    branch_pair new_pair = { {source_cluster, target_cluster}, lnscore, score };
    pair_.push_back(new_pair);
}

// 获取所有匹配的点云对及其得分
const std::vector<branch_pair>& MatchingResult::GetMatchedClustersWithScores() const 
{
    return pair_;
}
//根据内点比对所有配对进行排序
void MatchingResult::SortMatchedClusters(int j)
{
    if (pair_.size() > 1)
    {
        std::sort(pair_.begin(), pair_.end(), Comparator(j));
    }
}

//从点云中根据索引提取
pcl::PointCloud<pcl::PointXYZ>::Ptr MatchingResult::ExtractClusterCloud
    (const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
        const pcl::PointIndices& cluster_indices)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_cloud->reserve(cluster_indices.indices.size());

    for (const auto index : cluster_indices.indices)
    {
        cluster_cloud->points.push_back(original_cloud->points[index]);
    }
    cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->points.size());
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = false;

    return cluster_cloud;
}
//保存点云对为pcd
void MatchingResult::SaveMatchedClustersAsPCDs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
    if (pair_.size() == 0) 
    {
        std::cout << "没有找到配对树枝" << std::endl;
        return;
    }
    for (size_t i = 0; i < pair_.size(); ++i)
    {
        const auto& source_cluster = pair_[i].matched_pair.first;
        const auto& target_cluster = pair_[i].matched_pair.second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cluster_cloud = ExtractClusterCloud(source_cloud, source_cluster.indices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cluster_cloud = ExtractClusterCloud(target_cloud, target_cluster.indices);

        std::cerr << "保存配对，第 " << i + 1 << "  对；得分为 " << pair_[i].lnscore_pair << std::endl;


        std::string source_file_name = "match_" + std::to_string(i + 1) + "_source.pcd";
        std::string target_file_name = "match_" + std::to_string(i + 1) + "_target.pcd";

        pcl::io::savePCDFileBinary(source_file_name, *source_cluster_cloud);
        pcl::io::savePCDFileBinary(target_file_name, *target_cluster_cloud);
    }
}


// 设置源点云和目标点云的sorted_clusters
void BranchMatcher::SetSortedClusters(const std::vector<ClusterInfo>& source_clusters, const std::vector<ClusterInfo>& target_clusters)
{
    source_clusters_ = source_clusters;
    target_clusters_ = target_clusters;
}

//执行匹配操作，筛选出真正的同一个树枝的两个点云对
//筛选出很多对
//首先需要确保找到的都是正确的配对
//其次才是筛选出最优配对

MatchingResult BranchMatcher::MatchBranches(const CylinderParams& trunk_cylinder_params_s,
    const CylinderParams& trunk_cylinder_params_t,
    const double tol_r, const double tol_theta, const double tol_z,int m)
{
    MatchingResult result;
    std::vector<std::tuple<int, int, double, double>> matches; // 用于存储所有可能的匹配 (source_index, target_index, score)


    //int skip_j = -1;//用于跳过内循环中没必要再找一遍的
    for (int i = 0; i < source_clusters_.size(); i++)
    {
        int best_j = -1;//记录一次外循环找到的最好的j
        double last_lnscore = std::numeric_limits<double>::infinity();//记录内循环上一次的得分
        double last_score;
        for (int j = 0; j < target_clusters_.size(); j++)
        {
            //因为是按照Z排序过的聚类，可以直接跳过上次配上之前的聚类
            //但是有可能上次那对是错的，所以用<，而不是<=
            //这一步只是为了省时间。
            // 但是造成了不对称，导致source和target互换就会出错
            //if (j < skip_j) 
            //{
            //    continue;  //开始下一次内循环
            //}
            double current_lnscore;//临时存放当前分类得分
            double current_score;//存放后续排序得分
            //判断target聚类j
            if (IsSameBranch(source_clusters_[i], target_clusters_[j],
                trunk_cylinder_params_s,
                trunk_cylinder_params_t,
                tol_r, tol_theta, tol_z, current_lnscore,current_score))
            {
                if (current_lnscore < last_lnscore)//更新
                {
                    last_lnscore = current_lnscore;
                    last_score = current_score;
                    best_j = j;
                }
                //skip_j = best_j;
            }
        }
        // 当前一次外循环结束，添加最佳 j 值对应的匹配结果
        if (best_j != -1)
        {
            matches.emplace_back(i, best_j, last_lnscore, last_score);
            //result.AddMatchedClusters(source_clusters_[i], target_clusters_[best_j], last_score);
        }
    }

    // 处理所有可能的匹配，确保每个 target_clusters_ 只匹配一次
    std::unordered_map<int, std::tuple<int, double, double>> best_matches; // target_index -> (source_index, score)
    
    for (const auto& match : matches)
    {
        int source_index = std::get<0>(match);
        int target_index = std::get<1>(match);
        double lnscore = std::get<2>(match);
        double score = std::get<3>(match);

        // 如果 target_index 尚未匹配，或者找到更好的匹配（分类得分更低），更新匹配
        if (best_matches.find(target_index) == best_matches.end() || std::get<1>(best_matches[target_index]) > lnscore)
        {
            best_matches[target_index] = std::make_tuple(source_index, lnscore, score);
        }
    }
    // 添加最终匹配结果，一对一的！
    for (const auto& best_match : best_matches)
    {
        int target_index = best_match.first;
        int source_index = std::get<0>(best_match.second);
        double lnscore = std::get<1>(best_match.second);
        double score = std::get<2>(best_match.second);

        result.AddMatchedClusters(source_clusters_[source_index], target_clusters_[target_index], lnscore, score);
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //直接只存储排序后的第一对
    result.SortMatchedClusters(m);
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    return result;
}

// 判断两个点云簇是否为同一个树枝（根据半径、夹角、相对地面高度等）
bool BranchMatcher::IsSameBranch(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, 
    const CylinderParams& trunk_cylinder_params_s,
    const CylinderParams& trunk_cylinder_params_t,
    const double tol_r,const double tol_theta,const float tol_z, double& lnscore, double& score)
{
    const auto& source_params = source_cluster.center_axis_radius;
    const auto& target_params = target_cluster.center_axis_radius;

    // 根据半径、夹角、相对地面高度等计算相似度，并判断是否在tolerance范围内
    double radius_similarity = std::abs(source_params.radius - target_params.radius);
    //acos返回的是弧度[0,π]

    double angle_source = (std::acos(source_params.axis.dot(trunk_cylinder_params_s.axis)));
    double angle_target = (std::acos(target_params.axis.dot(trunk_cylinder_params_t.axis)));
    double angle_similarity = std::abs(angle_source - angle_target);
    ////与地面平面的距离
    float height_similarity = std::abs(source_cluster.z_relative - target_cluster.z_relative);


    //归一化误差
    double deltaR_norm = radius_similarity / tol_r;
    double deltaTheta_norm = angle_similarity / tol_theta;
    double deltaH_norm = static_cast<double>(height_similarity) / tol_z;
    //动态权重
    double epsilon = 1e-6;
    double weightR = -std::log(deltaR_norm+epsilon);
    double weightTheta = -std::log(deltaTheta_norm + epsilon);
    double weightH = -std::log(deltaH_norm + epsilon);
    double weightSum = weightR + weightTheta + weightH;
    weightR /= weightSum;
    weightTheta /= weightSum;
    weightH /= weightSum;

    lnscore = (deltaR_norm * weightR) + (deltaTheta_norm * weightTheta) + (deltaH_norm * weightH);
    score = deltaR_norm + deltaTheta_norm + deltaH_norm;
    //score = radius_similarity;

    return radius_similarity <= tol_r &&
        angle_similarity <= tol_theta &&
        height_similarity <= tol_z;//可以把z写成和theta有关的函数形式，更加准确！！！！！！！！！！！！
}
