#pragma once
#ifndef POINT_CLOUD_PREPROCESSOR_H
#define POINT_CLOUD_PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <math.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include "cylinder_fitter.h"

class PointCloudPreprocessor {
public:
    PointCloudPreprocessor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :
        clouds_in(cloud),
        clouds_out(new pcl::PointCloud<pcl::PointNormal>),
        ground_plane_coefficients(new pcl::ModelCoefficients),
        statiscal_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        height_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        n_clouds_out(new pcl::PointCloud<pcl::PointNormal>),
        no_trunk_(new pcl::PointCloud<pcl::PointXYZ>)
    { }

    void ransac_ground(int it_n, float dis, bool ground_ornot);
    void statiscal_removal(int a, float b);
    void voxel_removal(float leaf_size);
    void cluster(float tol, int min_s, int max_s);
    void height_filter(double a, double b);
    void mls_suface(int n, float r, bool store);
    void normal_estimate(float r, bool store);
    pcl::ModelCoefficients::Ptr getGroundPlane(); 
    void remove_trunk(const CylinderParams& trunk_axis_, pcl::PointCloud<pcl::PointXYZ>::Ptr have_trunk_, bool store);

    pcl::PointCloud<pcl::PointNormal>::Ptr getCloudsOut();
    pcl::PointCloud<pcl::PointNormal>::Ptr getNCloudsOut();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getStatiscalClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getGroundClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getHeightClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getNotrunk();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_in;
    pcl::PointCloud<pcl::PointNormal>::Ptr clouds_out;
    pcl::PointCloud<pcl::PointNormal>::Ptr n_clouds_out;
    pcl::ModelCoefficients::Ptr ground_plane_coefficients;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr statiscal_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered;	
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_trunk_;

};

#endif // POINT_CLOUD_PREPROCESSOR_H
