#include <iostream>
#include <pcl/console/parse.h>//console中的字段的头文件
#include <pcl/console/print.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <algorithm> // for std::sort
#include <chrono>
#include <cctype>

#include"config.h"
#include"point_cloud_preprocessor.h"
#include"cylinder_fitter.h"
#include"cylindrical_registration.h"
#include"pre_branch.h"
#include"branch_match.h"
#include"branch_registration.h"
#include"icp_cc.h"

double tstart, tstop, ttime;//时间
std::string dir_;
std::vector<std::pair<std::string, boost::filesystem::path>> pcd_files_; // 用 pair 存储文件名和路径
// 自然排序比较函数
bool naturalCompare(const std::string& a, const std::string& b) {
    auto ai = a.begin();
    auto bi = b.begin();
    while (ai != a.end() && bi != b.end()) {
        if (std::isdigit(*ai) && std::isdigit(*bi)) {
            // 处理数字
            std::string num1, num2;
            while (ai != a.end() && std::isdigit(*ai)) num1 += *ai++;
            while (bi != b.end() && std::isdigit(*bi)) num2 += *bi++;
            if (std::stoll(num1) != std::stoll(num2))
                return std::stoll(num1) < std::stoll(num2);
        }
        else {
            if (*ai != *bi)
                return *ai < *bi;
            ++ai;
            ++bi;
        }
    }
    return a.size() < b.size();
}

//源点云变换到目标点云
int
main(int argc, char** argv)
{
    tstart = (double)clock() / CLOCKS_PER_SEC;//计时

    // 开始总时间记录
    std::chrono::time_point<std::chrono::high_resolution_clock> start_total = std::chrono::high_resolution_clock::now();


    pcl::console::print_info("Begin to have pcd file list\n");
    if (argc < 2)//函数名开始即计数为1 
    {
        pcl::console::print_error("Syntax is: %s ./your_program -dir /path/to/pointclouds \n", argv[0]);
        pcl::console::print_info("  where options are:\n");
        pcl::console::print_info("                     -dir X =directory of pcd sequences");
        return -1;
    }
    pcl::console::parse_argument(argc, argv, "-dir", dir_);//识别-dir，将后面的内容放到dir_中
    pcd_files_.clear(); // 清空容器

    //点云序列读取模块
    boost::filesystem::directory_iterator end_itr;
    if (boost::filesystem::is_directory(dir_))
    {
        for (boost::filesystem::directory_iterator itr(dir_); itr != end_itr; ++itr)
        {
            std::string ext = itr->path().extension().string();
            if (ext.compare(".pcd") == 0)
            {
                pcd_files_.emplace_back(itr->path().filename().string(), itr->path());

            }
            else
            {
                PCL_DEBUG("[PCDVideoPlayer::selectFolderButtonPressed] : found a different file\n");
            }
        }
    }//.pcd文件的文件名和文件路径分别存放
    else
    {
        PCL_ERROR("Path is not a directory\n");
        exit(-1);
    }

    // 对文件名和路径进行排序
    std::sort(pcd_files_.begin(), pcd_files_.end(), [](const auto& a, const auto& b) {
        return naturalCompare(a.first, b.first);
        });

    pcl::console::print_info("Have pcd file list successfully\n");
    int size_squences = pcd_files_.size();
    std::cout << "Total file of squences is " << size_squences << std::endl;

    // 输出文件名
    for (const auto& file : pcd_files_)
    {
        std::cout << file.first << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files_[0].second.string(), *target);



    //这个是成功的部分
   // 最终的读取点云数据程序
    for (int j = 1; j < size_squences; j++)
    {
        std::cout << "\n！！！这是第 " << j << " 次" << std::endl;
        // 计时任务1
        std::chrono::time_point<std::chrono::high_resolution_clock> start_coarse = std::chrono::high_resolution_clock::now();

        //float voxel_t = 0.008;//体素大小应该随着点的数量变大
        //int cluster_t_min = 150;
        //int cluster_t_max = 1500;
        //int cluster_t_min_j = j * cluster_t_min;
        //int cluster_t_max_j = j * cluster_t_max;

        //int cluster_t_min_short = 100;
        //int cluster_t_max_short = 1000;//600
        //int cluster_t_min_j_short = j * cluster_t_min_short;
        //int cluster_t_max_j_short = j * cluster_t_max_short;
        ////double overlap_ = 0.4;
        //读取点云数据
        //std::cout << "Reading file: " << i + 1 << " and " << i + 2 << " " << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files_[j].second.string(), *source);
        std::cout << "Loaded source_cloud: " << source->width * source->height << " data points" << std::endl;
        std::cout << "Loaded target_cloud: " << target->width * target->height << " data points" << std::endl;


        // ===================== 密度自适应参数计算（source和target分别计算）=====================
        // Source点云：计算密度参数
        PointCloudPreprocessor pre_density_s(source);
        pre_density_s.ransac_ground(10000, 0.15, 1);
        float d_mean_s = AdaptiveParams::computeMeanPointDistance(pre_density_s.getGroundClouds(), 1000);
        DensityScaledParams density_params_s = AdaptiveParams::computeDensityScaledParams(d_mean_s);
        std::cout << "\n[Source密度参数]";
        density_params_s.print();

        // Target点云：计算密度参数（累积点云密度会增加，d_mean会减小，算法自动调整）
        PointCloudPreprocessor pre_density_t(target);
        pre_density_t.ransac_ground(10000, 0.15, 1);
        float d_mean_t = AdaptiveParams::computeMeanPointDistance(pre_density_t.getGroundClouds(), 1000);
        DensityScaledParams density_params_t = AdaptiveParams::computeDensityScaledParams(d_mean_t);
        std::cout << "[Target密度参数]";
        density_params_t.print();

        // ===================== 树高自适应参数计算 =====================
        // 基于树高计算一级枝的相对高度区域 (15%-50%)
        TreeHeightParams height_params_s = AdaptiveParams::computeTreeHeightParams(pre_density_s.getGroundClouds());
        std::cout << "[Source]"; height_params_s.print();

        TreeHeightParams height_params_t = AdaptiveParams::computeTreeHeightParams(pre_density_t.getGroundClouds());
        std::cout << "[Target]"; height_params_t.print();

        //----------------------------------------------------------------------------------------
               //----------------------------------------------------------------------------------------
               //预处理，后续要把这些整合到一个预处理函数，把树枝处理也放进这个类中。
        //PointCloudPreprocessor pre_s(source);
        //pre_s.ransac_ground(10000, 0.15, 1); //0.12
        //pre_s.statiscal_removal(5, 0.1);
        //pre_s.voxel_removal(voxel_t);
        //pre_s.height_filter(0.5, 1.5);//!!!!!!!!!!!!
        //pre_s.cluster(0.06, 1000, 1000000);
        ////pre_s.mls_suface(2, 0.03, 1);
        //pre_s.normal_estimate(0.03, 1);

        //PointCloudPreprocessor pre_t(target);
        //pre_t.ransac_ground(10000, 0.15, 1);
        //pre_t.statiscal_removal(5, 0.1);
        //pre_t.voxel_removal(voxel_t);
        ////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!???????????????????????????????????
        ////这里用0.5-1.0之间的树干拟合圆柱，但是后面选取的是0.7-1.5之间的树枝，还需要再考虑
        //pre_t.height_filter(0.5, 1.5);//!!!!!!!!!!!!
        //pre_t.cluster(0.06, 1000, 1000000);
        ////pre_t.mls_suface(2, 0.03, 1);
        //pre_t.normal_estimate(0.03, 1);

        PointCloudPreprocessor pre_s(source);
        pre_s.ransac_ground(10000, 0.15, 1);
        pre_s.statiscal_removal(density_params_s.sor_mean_k, 0.1);
        pre_s.voxel_removal(density_params_s.voxel_size);
        pre_s.height_filter(height_params_s.branch_z_min, height_params_s.branch_z_max);
        pre_s.cluster(density_params_s.cluster_tolerance, 1000, 1000000);
        pre_s.normal_estimate(density_params_s.normal_radius, 1);

        PointCloudPreprocessor pre_t(target);
        pre_t.ransac_ground(10000, 0.15, 1);
        pre_t.statiscal_removal(density_params_t.sor_mean_k, 0.1);
        pre_t.voxel_removal(density_params_t.voxel_size);
        pre_t.height_filter(height_params_t.branch_z_min, height_params_t.branch_z_max);
        pre_t.cluster(density_params_t.cluster_tolerance, 1000, 1000000);
        pre_t.normal_estimate(density_params_t.normal_radius, 1);

        //中部树干圆柱拟合
        CylinderFitter cy_s(pre_s.getNCloudsOut());
        cy_s.setCloudHei(pre_s.getHeightClouds());
        cy_s.fitCylinder(0.01, 10000, 0.01, 0.1, 1);
        CylinderParams trunk_params_s = cy_s.getCylinderParams();
        std::cout << trunk_params_s.axis << std::endl;
        std::cout << trunk_params_s.radius << "\n" << std::endl;

        CylinderFitter cy_t(pre_t.getNCloudsOut());
        cy_t.setCloudHei(pre_t.getHeightClouds());
        cy_t.fitCylinder(0.01, 10000, 0.01, 0.1, 1);
        CylinderParams trunk_params_t = cy_t.getCylinderParams();
        std::cout << trunk_params_t.axis << std::endl;
        std::cout << trunk_params_t.radius << "\n" << std::endl;

        // 获取拟合出的初始半径
        double r_trunk_s = trunk_params_s.radius;
        double r_trunk_t = trunk_params_t.radius;
        double unified_trunk_r = (r_trunk_s + r_trunk_t) / 2.0;
        // 将统一半径写入本地参数快照（避免 getCylinderParams() 重新计算覆盖）
        trunk_params_s.radius = static_cast<float>(unified_trunk_r);
        trunk_params_t.radius = static_cast<float>(unified_trunk_r);

        std::cout << cy_s.getCylinderParams().radius << "\n" << std::endl;
        std::cout << cy_t.getCylinderParams().radius << "\n" << std::endl;

        // ===================== 树枝预处理（各自使用各自的密度参数）=====================
        // 环形分割自适应参数：基于主干半径的比例膨胀
        float r_min_s, r_max_s, r_min_t, r_max_t;
        BranchSegmentParams::computeRingParams(trunk_params_s.radius, r_min_s, r_max_s);
        BranchSegmentParams::computeRingParams(trunk_params_t.radius, r_min_t, r_max_t);
        std::cout << "[环形分割自适应] source: r_min=" << r_min_s << ", r_max=" << r_max_s
            << "; target: r_min=" << r_min_t << ", r_max=" << r_max_t << std::endl;

        BranchSegment branch_s(pre_s.getStatiscalClouds());
        branch_s.ring_seg(trunk_params_s, pre_s.getGroundPlane(), r_min_s, r_max_s,
            height_params_s.branch_z_min, height_params_s.branch_z_max);
        branch_s.cluster_branch(trunk_params_s,
            density_params_s.branch_cluster_tol,
            density_params_s.cluster_min,
            density_params_s.cluster_max);

        BranchSegment branch_t(pre_t.getStatiscalClouds());
        branch_t.ring_seg(trunk_params_t, pre_t.getGroundPlane(), r_min_t, r_max_t,
            height_params_t.branch_z_min, height_params_t.branch_z_max);
        branch_t.cluster_branch(trunk_params_t,
            density_params_t.branch_cluster_tol,
            density_params_t.cluster_min,
            density_params_t.cluster_max);

        ////树枝预处理
        //BranchSegment branch_s(pre_s.getStatiscalClouds());//输入的点云只经过了统计滤波
        //branch_s.ring_seg(cy_s.getCylinderParams(), pre_s.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);//0.2-0.25这个距离需要再考究，因为切的近，才能切到粗壮的！！！！！！！
        //branch_s.cluster_branch(cy_s.getCylinderParams(), 0.01, cluster_t_min, cluster_t_max);//95\124

        //BranchSegment branch_t(pre_t.getStatiscalClouds());
        //////////////////////////////////////////////////////////////////////////////////0.7->0.5(最后一次改的)/////////////////////////////////////////////
        //branch_t.ring_seg(cy_t.getCylinderParams(), pre_t.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);
        //branch_t.cluster_branch(cy_t.getCylinderParams(), 0.01, cluster_t_min_j, cluster_t_max_j);

        // 缓存排序后的潜在聚类，避免重复拷贝
        const auto sorted_clusters_s = branch_s.getSortedClusters();
        const auto sorted_clusters_t = branch_t.getSortedClusters();

        //树枝配对 - 使用优化匹配器
        std::cout << "潜在配对树枝source：" << sorted_clusters_s.size()
            << " ；潜在配对树枝target：" << sorted_clusters_t.size() << std::endl;

        // 保存所有潜在配对的树枝点云（从 sorted_clusters 索引回填）
        auto buildPotentialCloud = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& all_branch,
            const std::vector<ClusterInfo>& sorted_clusters)
            -> pcl::PointCloud<pcl::PointXYZ>::Ptr
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& cluster : sorted_clusters)
                {
                    for (int idx : cluster.indices.indices)
                    {
                        if (idx >= 0 && idx < static_cast<int>(all_branch->size()))
                        {
                            cloud_out->points.push_back(all_branch->points[idx]);
                        }
                    }
                }
                cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
                cloud_out->height = 1;
                cloud_out->is_dense = false;
                return cloud_out;
            };

        auto potential_source_cloud = buildPotentialCloud(branch_s.getAllBranch(), sorted_clusters_s);
        auto potential_target_cloud = buildPotentialCloud(branch_t.getAllBranch(), sorted_clusters_t);

        std::stringstream ss_potential_s, ss_potential_t;
        ss_potential_s << "potential_branches_source_" << j << ".pcd";
        ss_potential_t << "potential_branches_target_" << j << ".pcd";
        pcl::io::savePCDFileBinary(ss_potential_s.str(), *potential_source_cloud);
        pcl::io::savePCDFileBinary(ss_potential_t.str(), *potential_target_cloud);
        std::cout << "[保存] 潜在树枝点云: " << ss_potential_s.str() << ", " << ss_potential_t.str() << std::endl;

        //树枝配对
        BranchMatcher matcher;
        matcher.SetSortedClusters(branch_s.getSortedClusters(), branch_t.getSortedClusters());
        std::cout << "潜在配对树枝source：" << branch_s.getSortedClusters().size() << " ；潜在配对树枝target：" << branch_t.getSortedClusters().size() << std::endl;

        //保存配对树枝pcd
        MatchingResult result_s_t =
            matcher.MatchBranches(
                cy_s.getCylinderParams(),
                cy_t.getCylinderParams(),
                0.005, 0.262, 0.05, j);
        result_s_t.SaveMatchedClustersAsPCDs(branch_s.getAllBranch(), branch_t.getAllBranch());

        /*使用树干配准后的坐标进行树枝配准，但找配对时却是用树枝配准前的，这其中的坐标关系？？？？？？？？？？？？？？？？？？？？？？
        只说绕Z轴旋转，可以将向量直接投影到XOY平面，求夹角。
        到底绕哪个轴转，且如果需要成为绕Z轴转，前面树干配准要怎么改？
        直接使用T，在变换点云的同时，变换地面和轴线参数。*/

        ////树枝配准
        //BranchRegistration branch(source, target);
        ////只用最优的一对配对
        //branch.alignCylinders_with_project(result_s_t.GetMatchedClustersWithScores()[0].matched_pair);
        //branch.BranchCoarseRegistration();

        ////----------------------------------------------------------------------------------------
        ////----------------------------------------------------------------------------------------
        ////使用T变换,获取变换后点云中后续要用到的参数！！！！！！！！！！！！
        //PointCloudPreprocessor pre_ss(branch.getSourceAlignedCloud());
        //pre_ss.ransac_ground(10000, 0.15, 1);
        //pre_ss.statiscal_removal(density_params_s.sor_mean_k, 0.1);
        //pre_ss.voxel_removal(density_params_s.voxel_size);
        //pre_ss.height_filter(0.2, 0.3);
        //pre_ss.mls_suface(2, density_params_s.mls_radius, 1);

        //PointCloudPreprocessor pre_tt(branch.getTargetAlignedCloud());
        //pre_tt.ransac_ground(10000, 0.15, 1);
        //pre_tt.statiscal_removal(density_params_t.sor_mean_k, 0.1);
        //pre_tt.voxel_removal(density_params_t.voxel_size);
        //pre_tt.height_filter(0.2, 0.3);
        //pre_tt.mls_suface(2, density_params_t.mls_radius, 1);

        //////使用T变换,获取变换后点云中后续要用到的参数！！！！！！！！！！！！
        ////PointCloudPreprocessor pre_ss(branch.getSourceAlignedCloud());
        ////pre_ss.ransac_ground(10000, 0.15, 1); //0.12
        ////pre_ss.statiscal_removal(10, 0.1);
        ////pre_ss.voxel_removal(voxel_t);
        //////!!!!!!!
        ////pre_ss.height_filter(0.2, 0.3);
        //////pre_ss.cluster(0.03, 100, 10000); //聚类输出点个数最大聚类,4-4的第一站，杂草点更多，导致出问题
        ////pre_ss.mls_suface(2, 0.03, 1);
        ////// return 0;

        ////PointCloudPreprocessor pre_tt(branch.getTargetAlignedCloud());
        ////pre_tt.ransac_ground(10000, 0.15, 1); //、、、、、、、、、、、、、、、、、、、、、、、0.12
        ////pre_tt.statiscal_removal(10, 0.1);
        ////pre_tt.voxel_removal(voxel_t);
        //////!!!!!!!
        ////pre_tt.height_filter(0.2, 0.3);//需要有地面参数，即ransac_ground函数
        //////pre_tt.cluster(0.03, 100, 10000); //聚类输出点个数最大聚类
        ////pre_tt.mls_suface(2, 0.03, 1);

        ////树干圆柱拟合
        //CylinderFitter cy_ss(pre_ss.getCloudsOut());
        //cy_ss.setCloudHei(pre_ss.getHeightClouds());
        //cy_ss.fitCylinder(0.1, 10000, 0.01, 0.1, 1);

        //CylinderFitter cy_tt(pre_tt.getCloudsOut());
        //cy_tt.setCloudHei(pre_tt.getHeightClouds());
        //cy_tt.fitCylinder(0.1, 10000, 0.01, 0.1, 1);

        ////树干配准
        //CylindricalRegistration trunk(branch.getSourceAlignedCloud(), branch.getTargetAlignedCloud());
        //trunk.alignCylinders(cy_ss.getCylinderParams(), cy_tt.getCylinderParams());
        //trunk.TrunkCoarseRegistration();

        ////粗配准的变换矩阵
        //Eigen::Affine3d Tr = trunk.getMatix() * branch.getMatix();
        //std::cout << "\n粗配准的变换矩阵：\n" << Tr.matrix() << std::endl;

        // 假设 result_s_t 中存放了通过 score 排序的候选对
        const auto& candidates = result_s_t.GetMatchedClustersWithScores();

        if (candidates.empty()) {
            std::cerr << "[错误] 第 " << j << " 帧完全无法匹配，跳过此帧" << std::endl;
            continue;
        }

        // 1. 设定要迭代验证的配对数量 (最多验证前 5 对)
        int K_max = std::min(5, static_cast<int>(candidates.size()));

        // 记录全局最优结果的变量
        Eigen::Affine3d best_Tr = Eigen::Affine3d::Identity();
        Eigen::Affine3d best_branch_matrix = Eigen::Affine3d::Identity();
        Eigen::Affine3d best_trunk_matrix = Eigen::Affine3d::Identity();
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_target_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        int max_inliers = -1;
        double min_rmse = 1000.0;
        int best_k = 0;

        std::cout << "\n[启动矩阵级验证] 开始逐一验证前 " << K_max << " 个配对..." << std::endl;

        for (int k = 0; k < K_max; ++k)
        {
            std::cout << ">>> 正在计算并验证第 " << k + 1 << " 个配对的完整矩阵..." << std::endl;

            // ==========================================
            // Step 1: 针对当前配对，求取完整的旋转平移矩阵
            // ==========================================
            BranchRegistration branch_reg(source, target);
            branch_reg.alignCylinders_with_project(candidates[k].matched_pair);
            branch_reg.BranchCoarseRegistration();

            PointCloudPreprocessor pre_ss(branch_reg.getSourceAlignedCloud());
            pre_ss.ransac_ground(10000, 0.15, 1);
            pre_ss.statiscal_removal(density_params_s.sor_mean_k, 0.1);
            pre_ss.voxel_removal(density_params_s.voxel_size);
            pre_ss.height_filter(0.2, 0.3);
            pre_ss.mls_suface(2, density_params_s.mls_radius, 1);

            PointCloudPreprocessor pre_tt(branch_reg.getTargetAlignedCloud());
            pre_tt.ransac_ground(10000, 0.15, 1);
            pre_tt.statiscal_removal(density_params_t.sor_mean_k, 0.1);
            pre_tt.voxel_removal(density_params_t.voxel_size);
            pre_tt.height_filter(0.2, 0.3);
            pre_tt.mls_suface(2, density_params_t.mls_radius, 1);

            CylinderFitter cy_ss(pre_ss.getCloudsOut());
            cy_ss.setCloudHei(pre_ss.getHeightClouds());
            cy_ss.fitCylinder(0.1, 10000, 0.01, 0.1, 1);

            CylinderFitter cy_tt(pre_tt.getCloudsOut());
            cy_tt.setCloudHei(pre_tt.getHeightClouds());
            cy_tt.fitCylinder(0.1, 10000, 0.01, 0.1, 1);

            CylindricalRegistration trunk_reg(branch_reg.getSourceAlignedCloud(), branch_reg.getTargetAlignedCloud());
            trunk_reg.alignCylinders(cy_ss.getCylinderParams(), cy_tt.getCylinderParams());
            trunk_reg.TrunkCoarseRegistration();

            // 当前配对解算出的完整粗配准矩阵
            Eigen::Affine3d current_Tr = trunk_reg.getMatix() * branch_reg.getMatix();

            // ==========================================
            // Step 2: 验证矩阵是否正确 (全局检验)
            // ==========================================
            int current_inliers = 0;
            double current_error_sum = 0.0;

            // 遍历 Source 中的每一根树枝
            for (const auto& s_branch : branch_s.getSortedClusters()) {

                // 将 Source 树枝的中心点和轴向，用算出的 current_Tr 转到 Target 坐标系下
                Eigen::Vector3d transformed_center = current_Tr * s_branch.center_axis_radius.center;
                Eigen::Vector3d transformed_axis = current_Tr.rotation() * s_branch.center_axis_radius.axis;

                double best_dist = 1000.0;
                bool found_match = false;

                // 在 Target 的树枝库中，寻找有没有重合的
                for (const auto& t_branch : branch_t.getSortedClusters()) {
                    double dist = (transformed_center - t_branch.center_axis_radius.center).norm();
                    double angle = std::acos(std::abs(transformed_axis.dot(t_branch.center_axis_radius.axis))) * 180.0 / M_PI;

                    // 【限定条件】：距离误差 < 10cm 且 角度误差 < 15度 认为是重合的同名树枝
                    if (dist < 0.10 && angle < 15.0) {
                        if (dist < best_dist) {
                            best_dist = dist;
                            found_match = true;
                        }
                    }
                }

                // 如果成功找到了一根对应的树枝，内点数 +1
                if (found_match) {
                    current_inliers++;
                    current_error_sum += best_dist;
                }
            }

            double current_rmse = (current_inliers > 0) ? (current_error_sum / current_inliers) : 1000.0;
            std::cout << "  -> 检验结果: 成功对齐了 " << current_inliers << " 根树枝, 平均距离误差: " << current_rmse << "m" << std::endl;

            // ==========================================
            // Step 3: 更新全局最优（同时保存中间结果）
            // ==========================================
            // 谁对齐的树枝多，谁就是最优解；如果对齐数量一样，选平均误差最小的
            if (current_inliers > max_inliers ||
                (current_inliers == max_inliers && current_rmse < min_rmse))
            {
                max_inliers = current_inliers;
                min_rmse = current_rmse;
                best_Tr = current_Tr;
                best_k = k;
                // 保存中间结果，避免循环后重新计算
                best_branch_matrix = branch_reg.getMatix();
                best_trunk_matrix = trunk_reg.getMatix();
                *best_source_aligned = *(trunk_reg.getSourceAlignedCloud());
                *best_target_aligned = *(trunk_reg.getTargetAlignedCloud());
            }

            // ==========================================
            // Step 4: 提前终止条件 (节省计算时间)
            // ==========================================
            // 假设一棵树一共有 5 根有效分支，如果当前矩阵能完美对齐 3 根以上，并且误差极小，就认为找对了！
            if (current_inliers >= 3 && current_rmse < 0.05) {
                std::cout << "[完美命中] 已找到高度置信的配准矩阵，提前终止验证循环！" << std::endl;
                break;
            }
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "最终选定候选对 [" << best_k + 1 << "] 作为最优解！" << std::endl;
        std::cout << "该矩阵共匹配了 " << max_inliers << " 根树枝，平均误差 " << min_rmse << "m" << std::endl;
        std::cout << "粗配准的变换矩阵：\n" << best_Tr.matrix() << std::endl;
        std::cout << "========================================\n" << std::endl;

        // 接下来的 ICP 代码，将不再需要重新算 Tr，你可以直接把算好的 best_Tr 喂给它


        std::chrono::time_point<std::chrono::high_resolution_clock> end_coarse = std::chrono::high_resolution_clock::now();
        std::chrono::seconds duration_task1 = std::chrono::duration_cast<std::chrono::seconds>(end_coarse - start_coarse);
        std::cout << "Coarse-Registration took " << duration_task1.count() << " s." << std::endl;

        //----------------------------------------------------------------------------------------
        //----------------------------------------------------------------------------------------
        //  // 计时任务2
        std::chrono::time_point<std::chrono::high_resolution_clock> start_icp = std::chrono::high_resolution_clock::now();

        //icp进行精配准（使用保存的最优粗配准结果）
        PointCloudPreprocessor pre_sss(best_source_aligned);
        pre_sss.ransac_ground(10000, 0.15, 1);
        pre_sss.statiscal_removal(density_params_s.sor_mean_k, 0.1);
        pre_sss.voxel_removal(density_params_s.voxel_size);
        pre_sss.height_filter(0.2, 0.3);
        pre_sss.mls_suface(2, density_params_s.mls_radius, 1);

        PointCloudPreprocessor pre_ttt(best_target_aligned);
        pre_ttt.ransac_ground(10000, 0.15, 1);
        pre_ttt.statiscal_removal(density_params_t.sor_mean_k, 0.1);
        pre_ttt.voxel_removal(density_params_t.voxel_size);
        pre_ttt.height_filter(0.2, 0.3);
        pre_ttt.mls_suface(2, density_params_t.mls_radius, 1);

        //树干圆柱拟合
        CylinderFitter cy_sss(pre_sss.getCloudsOut());
        cy_sss.setCloudHei(pre_sss.getHeightClouds());
        cy_sss.fitCylinder(0.1, 10000, 0.01, 0.1, 1);

        CylinderFitter cy_ttt(pre_ttt.getCloudsOut());
        cy_ttt.setCloudHei(pre_ttt.getHeightClouds());
        cy_ttt.fitCylinder(0.1, 10000, 0.01, 0.1, 1);
        //树干去除,overlap直接写1
        //必须把树干切了！！！
        BranchSegment branch_ss(pre_sss.getVoxelClouds());
        branch_ss.ring_seg(cy_sss.getCylinderParams(), pre_sss.getGroundPlane(), cy_sss.getCylinderParams().radius + 0.1, cy_sss.getCylinderParams().radius + 2, 0, 4);

        BranchSegment branch_tt(pre_ttt.getVoxelClouds());
        branch_tt.ring_seg(cy_ttt.getCylinderParams(), pre_ttt.getGroundPlane(), cy_ttt.getCylinderParams().radius + 0.1, cy_ttt.getCylinderParams().radius + 2, 0, 4);


        //CC的ICP
        std::cout << "Ready to use icp-cc" << std::endl;
        CC_ICP icp;
        icp.setInitialClouds(best_source_aligned, best_target_aligned);
        icp.setPreprocessClouds(branch_ss.getAllBranch(), branch_tt.getAllBranch());
        icp.IcpUseCC(1e-7, 1000, 80000, 0.9);
        icp.Trans();

        //icp精配准的变换矩阵
        std::cout << "\n精配准的变换矩阵：\n" << icp.getMatix().matrix() << std::endl;

        ////先保存配准后的source点云
        //std::stringstream trans;
        //trans << "trans_source" << j << ".pcd";
        //pcl::io::savePCDFileBinary(trans.str(), *icp.getSourceResults());

        //将新的变换后的点云加入到target中
        *target += *(icp.getSourceResults());

        //整体的变换矩阵（使用保存的矩阵）
        Eigen::Affine3d T = icp.getMatix() * best_trunk_matrix * best_branch_matrix;
        std::cout << "\n输出整体的变换矩阵：\n" << T.matrix() << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> end_icp = std::chrono::high_resolution_clock::now();
        std::chrono::seconds duration_task2 = std::chrono::duration_cast<std::chrono::seconds>(end_icp - start_icp);
        std::cout << "Task 2 took " << duration_task2.count() << " s." << std::endl;

    }

    //所有的source变换后都加入到target了，所以最终的target就是结果
    pcl::io::savePCDFileBinary("jieguo.pcd", *target);

    // 计算总时间
    std::chrono::time_point<std::chrono::high_resolution_clock> end_total = std::chrono::high_resolution_clock::now();
    std::chrono::seconds duration_total = std::chrono::duration_cast<std::chrono::seconds>(end_total - start_total);
    std::cout << "Total time: " << duration_total.count() << " s." << std::endl;

    tstop = (double)clock() / CLOCKS_PER_SEC;
    ttime = tstop - tstart;
    std::cout << "run time is " << ttime << " seconds." << std::endl;//点云处理时间

    std::cout << "Finish!!!" << std::endl;

    return (0);
}
