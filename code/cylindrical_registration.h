#pragma once
#ifndef CYLINDRICAL_REGISTRATION_H
#define CYLINDRICAL_REGISTRATION_H

//#include "point_cloud_preprocessor.h"
#include "cylinder_fitter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/common/transforms.h>

//圆柱的直接求旋转平移矩阵进行刚体变换
class CylindricalRegistration {
public:
    CylindricalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) :
        source_clouds(source_cloud),
        target_clouds(target_cloud),
        source_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        target_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        T(Eigen::Affine3d::Identity())
    {}
    // 根据圆柱轴线求得旋转平移变换矩阵
    void alignCylinders(const CylinderParams& src_cylinder, const CylinderParams& tgt_cylinder);
    //将source点云变换到target
    void TrunkCoarseRegistration();
    //返回T
    Eigen::Affine3d getMatix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceAlignedCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTargetAlignedCloud();

private:
    //PointCloudPreprocessor preprocessor();
    //CylinderFitter source_fitter, target_fitter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds, target_clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_clouds, target_aligned_clouds;
    Eigen::Affine3d T;
};

#endif // CYLINDRICAL_REGISTRATION_H