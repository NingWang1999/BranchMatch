#include "branch_match.h"

void MatchingResult::AddMatchedClusters(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, double lnscore, double score)
{
    branch_pair new_pair = { {source_cluster, target_cluster}, lnscore, score };
    pair_.push_back(new_pair);
}

const std::vector<branch_pair>& MatchingResult::GetMatchedClustersWithScores() const 
{
    return pair_;
}

void MatchingResult::SortMatchedClusters(int j)
{
    if (pair_.size() > 1)
    {
        std::sort(pair_.begin(), pair_.end(), Comparator(j));
    }
}

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

void MatchingResult::SaveMatchedClustersAsPCDs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
    if (pair_.size() == 0) 
    {
        std::cout << "there are no matching branches" << std::endl;
        return;
    }
    for (size_t i = 0; i < pair_.size(); ++i)
    {
        const auto& source_cluster = pair_[i].matched_pair.first;
        const auto& target_cluster = pair_[i].matched_pair.second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cluster_cloud = ExtractClusterCloud(source_cloud, source_cluster.indices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cluster_cloud = ExtractClusterCloud(target_cloud, target_cluster.indices);

        std::cerr << "save match，the " << i + 1 << "  pair；score are " << pair_[i].lnscore_pair << std::endl;


        std::string source_file_name = "match_" + std::to_string(i + 1) + "_source.pcd";
        std::string target_file_name = "match_" + std::to_string(i + 1) + "_target.pcd";

        pcl::io::savePCDFileBinary(source_file_name, *source_cluster_cloud);
        pcl::io::savePCDFileBinary(target_file_name, *target_cluster_cloud);
    }
}

void BranchMatcher::SetSortedClusters(const std::vector<ClusterInfo>& source_clusters, const std::vector<ClusterInfo>& target_clusters)
{
    source_clusters_ = source_clusters;
    target_clusters_ = target_clusters;
}

MatchingResult BranchMatcher::MatchBranches(const CylinderParams& trunk_cylinder_params_s,
    const CylinderParams& trunk_cylinder_params_t,
    const double tol_r, const double tol_theta, const double tol_z,int m)
{
    MatchingResult result;
    std::vector<std::tuple<int, int, double, double>> matches; // (source_index, target_index, score)
    for (int i = 0; i < source_clusters_.size(); i++)
    {
        int best_j = -1;
        double last_lnscore = std::numeric_limits<double>::infinity();
        double last_score;
        for (int j = 0; j < target_clusters_.size(); j++)
        {
            double current_lnscore;
            double current_score;
            if (IsSameBranch(source_clusters_[i], target_clusters_[j],
                trunk_cylinder_params_s,
                trunk_cylinder_params_t,
                tol_r, tol_theta, tol_z, current_lnscore,current_score))
            {
                if (current_lnscore < last_lnscore)
                {
                    last_lnscore = current_lnscore;
                    last_score = current_score;
                    best_j = j;
                }
            }
        }
        if (best_j != -1)
        {
            matches.emplace_back(i, best_j, last_lnscore, last_score);
            //result.AddMatchedClusters(source_clusters_[i], target_clusters_[best_j], last_score);
        }
    }
    std::unordered_map<int, std::tuple<int, double, double>> best_matches; // target_index -> (source_index, score)
    
    for (const auto& match : matches)
    {
        int source_index = std::get<0>(match);
        int target_index = std::get<1>(match);
        double lnscore = std::get<2>(match);
        double score = std::get<3>(match);

        if (best_matches.find(target_index) == best_matches.end() || std::get<1>(best_matches[target_index]) > lnscore)
        {
            best_matches[target_index] = std::make_tuple(source_index, lnscore, score);
        }
    }
    for (const auto& best_match : best_matches)
    {
        int target_index = best_match.first;
        int source_index = std::get<0>(best_match.second);
        double lnscore = std::get<1>(best_match.second);
        double score = std::get<2>(best_match.second);

        result.AddMatchedClusters(source_clusters_[source_index], target_clusters_[target_index], lnscore, score);
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    result.SortMatchedClusters(m);
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    return result;
}

bool BranchMatcher::IsSameBranch(const ClusterInfo& source_cluster, const ClusterInfo& target_cluster, 
    const CylinderParams& trunk_cylinder_params_s,
    const CylinderParams& trunk_cylinder_params_t,
    const double tol_r,const double tol_theta,const float tol_z, double& lnscore, double& score)
{
    const auto& source_params = source_cluster.center_axis_radius;
    const auto& target_params = target_cluster.center_axis_radius;

    double radius_similarity = std::abs(source_params.radius - target_params.radius);
    //acos[0,π]

    double angle_source = (std::acos(source_params.axis.dot(trunk_cylinder_params_s.axis)));
    double angle_target = (std::acos(target_params.axis.dot(trunk_cylinder_params_t.axis)));
    double angle_similarity = std::abs(angle_source - angle_target);
    float height_similarity = std::abs(source_cluster.z_relative - target_cluster.z_relative);

    double deltaR_norm = radius_similarity / tol_r;
    double deltaTheta_norm = angle_similarity / tol_theta;
    double deltaH_norm = static_cast<double>(height_similarity) / tol_z;
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
        height_similarity <= tol_z;
}
