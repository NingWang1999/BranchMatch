#include <iostream>
#include <pcl/console/parse.h>
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


#include"point_cloud_preprocessor.h"
#include"cylinder_fitter.h"
#include"cylindrical_registration.h"
#include"pre_branch.h"
#include"branch_match.h"
#include"branch_registration.h"
#include"icp_cc.h"

double tstart, tstop, ttime;
std::string dir_;
std::vector<std::pair<std::string, boost::filesystem::path>> pcd_files_; 
bool naturalCompare(const std::string& a, const std::string& b) {
    auto ai = a.begin();
    auto bi = b.begin();
    while (ai != a.end() && bi != b.end()) {
        if (std::isdigit(*ai) && std::isdigit(*bi)) {
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

int
main(int argc, char** argv)
{
    tstart = (double)clock() / CLOCKS_PER_SEC;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_total = std::chrono::high_resolution_clock::now();
    pcl::console::print_info("Begin to have pcd file list\n");
    if (argc < 2)
    {
        pcl::console::print_error("Syntax is: %s ./your_program -dir /path/to/pointclouds \n", argv[0]);
        pcl::console::print_info("  where options are:\n");
        pcl::console::print_info("                     -dir X =directory of pcd sequences");
        return -1;
    }
    pcl::console::parse_argument(argc, argv, "-dir", dir_);
    pcd_files_.clear();

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
    }
    else
    {
        PCL_ERROR("Path is not a directory\n");
        exit(-1);
    }

    std::sort(pcd_files_.begin(), pcd_files_.end(), [](const auto& a, const auto& b) {
        return naturalCompare(a.first, b.first);
        });

    pcl::console::print_info("Have pcd file list successfully\n");
    int size_squences = pcd_files_.size();
    std::cout << "Total file of squences is " << size_squences << std::endl;

    for (const auto& file : pcd_files_)
    {
        std::cout << file.first << std::endl;
    }
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files_[0].second.string(), *target);

    for (int j = 1; j < size_squences; j++)
    {
        std::cout << "\n！！！this is the " << j << " times" << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> start_coarse = std::chrono::high_resolution_clock::now();

        float voxel_t = 0.008;
        int cluster_t_min = 150;
        int cluster_t_max = 1500;
        int cluster_t_min_j = j * cluster_t_min;
        int cluster_t_max_j = j * cluster_t_max;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files_[j].second.string(), *source);
        std::cout << "Loaded source_cloud: " << source->width * source->height << " data points" << std::endl;
        std::cout << "Loaded target_cloud: " << target->width * target->height << " data points" << std::endl;
        
        PointCloudPreprocessor pre_s(source);
        pre_s.ransac_ground(10000, 0.15, 1);
        pre_s.statiscal_removal(5, 0.1);
        pre_s.voxel_removal(voxel_t);
        pre_s.height_filter(0.5, 1.5);
        pre_s.cluster(0.06, 1000, 1000000);
        pre_s.normal_estimate(0.03, 1);

        PointCloudPreprocessor pre_t(target);
        pre_t.ransac_ground(10000, 0.15, 1);
        pre_t.statiscal_removal(5, 0.1);
        pre_t.voxel_removal(voxel_t);
        pre_t.height_filter(0.5, 1.5);
        pre_t.cluster(0.06, 1000, 1000000);
        pre_t.normal_estimate(0.03, 1);
        
        CylinderFitter cy_s(pre_s.getNCloudsOut());
        cy_s.setCloudHei(pre_s.getHeightClouds());
        cy_s.fitCylinder(0.01, 10000, 0.01, 0.2, 1);
        std::cout << "\n" << cy_s.getCylinderParams().axis << std::endl;

        CylinderFitter cy_t(pre_t.getNCloudsOut());
        cy_t.setCloudHei(pre_t.getHeightClouds());
        cy_t.fitCylinder(0.01, 10000, 0.01, 0.2, 1);
        std::cout << "\n" << cy_t.getCylinderParams().axis << std::endl;
        
        BranchSegment branch_s(pre_s.getStatiscalClouds());
        branch_s.ring_seg(cy_s.getCylinderParams(), pre_s.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);
        branch_s.cluster_branch(cy_s.getCylinderParams(), 0.01, cluster_t_min, cluster_t_max);

        BranchSegment branch_t(pre_t.getStatiscalClouds());
        branch_t.ring_seg(cy_t.getCylinderParams(), pre_t.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);
        branch_t.cluster_branch(cy_t.getCylinderParams(), 0.01, cluster_t_min_j, cluster_t_max_j);

        BranchMatcher matcher;
        matcher.SetSortedClusters(branch_s.getSortedClusters(), branch_t.getSortedClusters());
        std::cout << "potential pair from source：" << branch_s.getSortedClusters().size() << " ；potential pair from target：" << branch_t.getSortedClusters().size() << std::endl;

        MatchingResult result_s_t =
            matcher.MatchBranches(
                cy_s.getCylinderParams(),
                cy_t.getCylinderParams(),
                0.005, 0.262, 0.05, j);
        result_s_t.SaveMatchedClustersAsPCDs(branch_s.getAllBranch(), branch_t.getAllBranch());

        BranchRegistration branch(source, target);
        branch.alignCylinders_with_project(result_s_t.GetMatchedClustersWithScores()[0].matched_pair);
        branch.BranchCoarseRegistration();
        
        PointCloudPreprocessor pre_ss(branch.getSourceAlignedCloud());
        pre_ss.ransac_ground(10000, 0.15, 1);
        pre_ss.statiscal_removal(10, 0.1);
        pre_ss.voxel_removal(voxel_t);
        pre_ss.height_filter(0.2, 0.3);
        pre_ss.mls_suface(2, 0.03, 1);

        PointCloudPreprocessor pre_tt(branch.getTargetAlignedCloud());
        pre_tt.ransac_ground(10000, 0.15, 1); 
        pre_tt.statiscal_removal(10, 0.1);
        pre_tt.voxel_removal(voxel_t);
        pre_tt.height_filter(0.2, 0.3);
        pre_tt.mls_suface(2, 0.03, 1);

        CylinderFitter cy_ss(pre_ss.getCloudsOut());
        cy_ss.setCloudHei(pre_ss.getHeightClouds());
        cy_ss.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        CylinderFitter cy_tt(pre_tt.getCloudsOut());
        cy_tt.setCloudHei(pre_tt.getHeightClouds());
        cy_tt.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        CylindricalRegistration trunk(branch.getSourceAlignedCloud(), branch.getTargetAlignedCloud());
        trunk.alignCylinders(cy_ss.getCylinderParams(), cy_tt.getCylinderParams());
        trunk.TrunkCoarseRegistration();

        Eigen::Affine3d Tr = trunk.getMatix() * branch.getMatix();
        std::cout << "\ncoarse registration matrix：\n" << Tr.matrix() << std::endl;


        std::chrono::time_point<std::chrono::high_resolution_clock> end_coarse = std::chrono::high_resolution_clock::now();
        std::chrono::seconds duration_task1 = std::chrono::duration_cast<std::chrono::seconds>(end_coarse - start_coarse);
        std::cout << "Coarse-Registration took " << duration_task1.count() << " s." << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> start_icp = std::chrono::high_resolution_clock::now();

        PointCloudPreprocessor pre_sss(trunk.getSourceAlignedCloud());
        pre_sss.ransac_ground(10000, 0.15, 1);
        pre_sss.statiscal_removal(10, 0.1);
        pre_sss.voxel_removal(voxel_t);
        pre_sss.height_filter(0.2, 0.3);
        pre_sss.mls_suface(2, 0.03, 1);

        PointCloudPreprocessor pre_ttt(trunk.getTargetAlignedCloud());
        pre_ttt.ransac_ground(10000, 0.15, 1);
        pre_ttt.statiscal_removal(10, 0.1);
        pre_ttt.voxel_removal(voxel_t);
        pre_ttt.height_filter(0.2, 0.3);
        pre_ttt.mls_suface(2, 0.03, 1);
        CylinderFitter cy_sss(pre_sss.getCloudsOut());
        cy_sss.setCloudHei(pre_sss.getHeightClouds());
        cy_sss.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        CylinderFitter cy_ttt(pre_ttt.getCloudsOut());
        cy_ttt.setCloudHei(pre_ttt.getHeightClouds());
        cy_ttt.fitCylinder(0.1, 10000, 0.01, 0.2, 1);
        BranchSegment branch_ss(pre_sss.getVoxelClouds());
        branch_ss.ring_seg(cy_sss.getCylinderParams(), pre_sss.getGroundPlane(), cy_sss.getCylinderParams().radius + 0.1, cy_sss.getCylinderParams().radius + 2, 0, 4);

        BranchSegment branch_tt(pre_ttt.getVoxelClouds());
        branch_tt.ring_seg(cy_ttt.getCylinderParams(), pre_ttt.getGroundPlane(), cy_ttt.getCylinderParams().radius + 0.1, cy_ttt.getCylinderParams().radius + 2, 0, 4);

        std::cout << "Ready to use icp-cc" << std::endl;
        CC_ICP icp;
        icp.setInitialClouds(trunk.getSourceAlignedCloud(), trunk.getTargetAlignedCloud());
        icp.setPreprocessClouds(branch_ss.getAllBranch(), branch_tt.getAllBranch());
        icp.IcpUseCC(1e-7, 1000, 80000, 0.9);
        icp.Trans();
        std::cout << "\nfine registration matrix：\n" << icp.getMatix().matrix() << std::endl;

        *target += *(icp.getSourceResults());

        Eigen::Affine3d T = icp.getMatix() * trunk.getMatix() * branch.getMatix();
        std::cout << "\nthe final registration matrix：\n" << T.matrix() << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> end_icp = std::chrono::high_resolution_clock::now();
        std::chrono::seconds duration_task2 = std::chrono::duration_cast<std::chrono::seconds>(end_icp - start_icp);
        std::cout << "Task 2 took " << duration_task2.count() << " s." << std::endl;

    }
    pcl::io::savePCDFileBinary("jieguo.pcd", *target);

    std::chrono::time_point<std::chrono::high_resolution_clock> end_total = std::chrono::high_resolution_clock::now();
    std::chrono::seconds duration_total = std::chrono::duration_cast<std::chrono::seconds>(end_total - start_total);
    std::cout << "Total time: " << duration_total.count() << " s." << std::endl;

    tstop = (double)clock() / CLOCKS_PER_SEC;
    ttime = tstop - tstart;
    std::cout << "run time is " << ttime << " seconds." << std::endl;

    std::cout << "Finish!!!" << std::endl;

    return (0);
}
