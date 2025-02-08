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

        float voxel_t = 0.008;//体素大小应该随着点的数量变大
        int cluster_t_min = 150;
        int cluster_t_max = 1500;
        int cluster_t_min_j = j * cluster_t_min;
        int cluster_t_max_j = j * cluster_t_max;

        int cluster_t_min_short = 100;
        int cluster_t_max_short = 1000;//600
        int cluster_t_min_j_short = j * cluster_t_min_short;
        int cluster_t_max_j_short = j * cluster_t_max_short;
        //double overlap_ = 0.4;
        //读取点云数据
        //std::cout << "Reading file: " << i + 1 << " and " << i + 2 << " " << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files_[j].second.string(), *source);
        std::cout << "Loaded source_cloud: " << source->width * source->height << " data points" << std::endl;
        std::cout << "Loaded target_cloud: " << target->width * target->height << " data points" << std::endl;

        //----------------------------------------------------------------------------------------
               //----------------------------------------------------------------------------------------
               //预处理，后续要把这些整合到一个预处理函数，把树枝处理也放进这个类中。
        PointCloudPreprocessor pre_s(source);
        pre_s.ransac_ground(10000, 0.15, 1); //0.12
        pre_s.statiscal_removal(5, 0.1);
        pre_s.voxel_removal(voxel_t);
        pre_s.height_filter(0.5, 1.5);//!!!!!!!!!!!!
        pre_s.cluster(0.06, 1000, 1000000);
        //pre_s.mls_suface(2, 0.03, 1);
        pre_s.normal_estimate(0.03, 1);

        PointCloudPreprocessor pre_t(target);
        pre_t.ransac_ground(10000, 0.15, 1);
        pre_t.statiscal_removal(5, 0.1);
        pre_t.voxel_removal(voxel_t);
        //////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!???????????????????????????????????
        //这里用0.5-1.0之间的树干拟合圆柱，但是后面选取的是0.7-1.5之间的树枝，还需要再考虑
        pre_t.height_filter(0.5, 1.5);//!!!!!!!!!!!!
        pre_t.cluster(0.06, 1000, 1000000);
        //pre_t.mls_suface(2, 0.03, 1);
        pre_t.normal_estimate(0.03, 1);

        //中部树干圆柱拟合
        CylinderFitter cy_s(pre_s.getNCloudsOut());
        cy_s.setCloudHei(pre_s.getHeightClouds());
        cy_s.fitCylinder(0.01, 10000, 0.01, 0.2, 1);
        std::cout << "\n" << cy_s.getCylinderParams().axis << std::endl;

        CylinderFitter cy_t(pre_t.getNCloudsOut());
        cy_t.setCloudHei(pre_t.getHeightClouds());
        cy_t.fitCylinder(0.01, 10000, 0.01, 0.2, 1);
        std::cout << "\n" << cy_t.getCylinderParams().axis << std::endl;
        //std::cout << cy_s.getCylinderParams().center << std::endl;

        //树枝预处理
        BranchSegment branch_s(pre_s.getStatiscalClouds());//输入的点云只经过了统计滤波
        branch_s.ring_seg(cy_s.getCylinderParams(), pre_s.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);//0.2-0.25这个距离需要再考究，因为切的近，才能切到粗壮的！！！！！！！
        branch_s.cluster_branch(cy_s.getCylinderParams(), 0.01, cluster_t_min, cluster_t_max);//95\124

        BranchSegment branch_t(pre_t.getStatiscalClouds());
        ////////////////////////////////////////////////////////////////////////////////0.7->0.5(最后一次改的)/////////////////////////////////////////////
        branch_t.ring_seg(cy_t.getCylinderParams(), pre_t.getGroundPlane(), 0.07, 0.14, 0.5, 1.5);
        branch_t.cluster_branch(cy_t.getCylinderParams(), 0.01, cluster_t_min_j, cluster_t_max_j);

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

        //树枝配准
        BranchRegistration branch(source, target);
        //只用最优的一对配对
        branch.alignCylinders_with_project(result_s_t.GetMatchedClustersWithScores()[0].matched_pair);
        branch.BranchCoarseRegistration();

        //----------------------------------------------------------------------------------------
        //----------------------------------------------------------------------------------------
        //使用T变换,获取变换后点云中后续要用到的参数！！！！！！！！！！！！
        PointCloudPreprocessor pre_ss(branch.getSourceAlignedCloud());
        pre_ss.ransac_ground(10000, 0.15, 1); //0.12
        pre_ss.statiscal_removal(10, 0.1);
        pre_ss.voxel_removal(voxel_t);
        //!!!!!!!
        pre_ss.height_filter(0.2, 0.3);
        //pre_ss.cluster(0.03, 100, 10000); //聚类输出点个数最大聚类,4-4的第一站，杂草点更多，导致出问题
        pre_ss.mls_suface(2, 0.03, 1);
        // return 0;

        PointCloudPreprocessor pre_tt(branch.getTargetAlignedCloud());
        pre_tt.ransac_ground(10000, 0.15, 1); //、、、、、、、、、、、、、、、、、、、、、、、0.12
        pre_tt.statiscal_removal(10, 0.1);
        pre_tt.voxel_removal(voxel_t);
        //!!!!!!!
        pre_tt.height_filter(0.2, 0.3);//需要有地面参数，即ransac_ground函数
        //pre_tt.cluster(0.03, 100, 10000); //聚类输出点个数最大聚类
        pre_tt.mls_suface(2, 0.03, 1);

        //树干圆柱拟合
        CylinderFitter cy_ss(pre_ss.getCloudsOut());
        cy_ss.setCloudHei(pre_ss.getHeightClouds());
        cy_ss.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        CylinderFitter cy_tt(pre_tt.getCloudsOut());
        cy_tt.setCloudHei(pre_tt.getHeightClouds());
        cy_tt.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        //树干配准
        CylindricalRegistration trunk(branch.getSourceAlignedCloud(), branch.getTargetAlignedCloud());
        trunk.alignCylinders(cy_ss.getCylinderParams(), cy_tt.getCylinderParams());
        trunk.TrunkCoarseRegistration();

        //粗配准的变换矩阵
        Eigen::Affine3d Tr = trunk.getMatix() * branch.getMatix();
        std::cout << "\n粗配准的变换矩阵：\n" << Tr.matrix() << std::endl;


        std::chrono::time_point<std::chrono::high_resolution_clock> end_coarse = std::chrono::high_resolution_clock::now();
        std::chrono::seconds duration_task1 = std::chrono::duration_cast<std::chrono::seconds>(end_coarse - start_coarse);
        std::cout << "Coarse-Registration took " << duration_task1.count() << " s." << std::endl;


        //----------------------------------------------------------------------------------------
        //----------------------------------------------------------------------------------------
        //  // 计时任务2
        std::chrono::time_point<std::chrono::high_resolution_clock> start_icp = std::chrono::high_resolution_clock::now();

        //icp进行精配准
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
        //树干圆柱拟合
        CylinderFitter cy_sss(pre_sss.getCloudsOut());
        cy_sss.setCloudHei(pre_sss.getHeightClouds());
        cy_sss.fitCylinder(0.1, 10000, 0.01, 0.2, 1);

        CylinderFitter cy_ttt(pre_ttt.getCloudsOut());
        cy_ttt.setCloudHei(pre_ttt.getHeightClouds());
        cy_ttt.fitCylinder(0.1, 10000, 0.01, 0.2, 1);
        //树干去除,overlap直接写1
        //必须把树干切了！！！
        BranchSegment branch_ss(pre_sss.getVoxelClouds());
        branch_ss.ring_seg(cy_sss.getCylinderParams(), pre_sss.getGroundPlane(), cy_sss.getCylinderParams().radius + 0.1, cy_sss.getCylinderParams().radius + 2, 0, 4);

        BranchSegment branch_tt(pre_ttt.getVoxelClouds());
        branch_tt.ring_seg(cy_ttt.getCylinderParams(), pre_ttt.getGroundPlane(), cy_ttt.getCylinderParams().radius + 0.1, cy_ttt.getCylinderParams().radius + 2, 0, 4);


        //CC的ICP
        std::cout << "Ready to use icp-cc" << std::endl;
        CC_ICP icp;
        icp.setInitialClouds(trunk.getSourceAlignedCloud(), trunk.getTargetAlignedCloud());
        //icp.setInitialClouds(branch_ss.getAllBranch(), branch_tt.getAllBranch());
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

        ////保存上一步融入增强的target点云，用于调试
        //std::stringstream tarstrong;
        //tarstrong << "tarstrong" << j << ".pcd";
        //pcl::io::savePCDFileBinary(tarstrong.str(), *target);

        //整体的变换矩阵
        Eigen::Affine3d T = icp.getMatix() * trunk.getMatix() * branch.getMatix();
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
