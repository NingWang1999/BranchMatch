#pragma once
#ifndef BRANCH_REGISTRATION_H
#define BRANCH_REGISTRATION_H

#include "cylinder_fitter.h"
#include "branch_match.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class BranchRegistration
{
public:
    BranchRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud):
        source_clouds(source_cloud),
        target_clouds(target_cloud),
        source_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        target_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        T(Eigen::Affine3d::Identity())
        
    {}
    //根据树枝圆柱轴线求得旋转平移变换矩阵
    //输入了branch_match得到的配对树枝的各种圆柱拟合信息
    //利用投影，在XOY面进行旋转（要求事先进行了树干配准）
    //传入树干中心点用于调整树枝轴线向量方向为远离树干
    void alignCylinders_with_project(const std::pair<ClusterInfo, ClusterInfo> matched_clusters);
    //使用T，将source点云变换到target
    void BranchCoarseRegistration();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceAlignedCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTargetAlignedCloud();
    //返回T
    Eigen::Affine3d getMatix();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds, target_clouds;//树干配准后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_clouds, target_aligned_clouds;//配准后的点云
    Eigen::Affine3d T;//配准用的变换矩阵
};

#endif