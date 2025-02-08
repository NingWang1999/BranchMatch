#pragma once
#ifndef CYLINDRICAL_REGISTRATION_H
#define CYLINDRICAL_REGISTRATION_H

#include "cylinder_fitter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/common/transforms.h>

class CylindricalRegistration {
public:
    CylindricalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) :
        source_clouds(source_cloud),
        target_clouds(target_cloud),
        source_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        target_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        T(Eigen::Affine3d::Identity())
    {}
    void alignCylinders(const CylinderParams& src_cylinder, const CylinderParams& tgt_cylinder);
    void TrunkCoarseRegistration();
    Eigen::Affine3d getMatix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceAlignedCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTargetAlignedCloud();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds, target_clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_clouds, target_aligned_clouds;
    Eigen::Affine3d T;
};

#endif // CYLINDRICAL_REGISTRATION_H
