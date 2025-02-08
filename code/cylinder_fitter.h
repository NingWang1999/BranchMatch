#pragma once
#ifndef CYLINDER_FITTER_H
#define CYLINDER_FITTER_H

#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <iostream>
#include <algorithm>
#include <pcl/common/distances.h>

static bool compare_x(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.x < b.x);
}
static bool compare_z(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.z < b.z);
}

struct CylinderParams
{
	Eigen::Vector3d point;
	Eigen::Vector3d axis;
	Eigen::Vector4f centroid;
	Eigen::Vector3d center;
	Eigen::Vector3d v_center;
	float radius;
};

class CylinderFitter
{
public:
	CylinderFitter(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) :
		clouds_in(cloud),
		clouds_hei(new pcl::PointCloud<pcl::PointXYZ>),
		clouds_out(new pcl::PointCloud<pcl::PointXYZ>),
		Coefficients(new pcl::ModelCoefficients)
	{
		clouds_hei = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

		pcl::PointXYZ defaultPoint1(0.0f, 0.0f, 0.0f);
		pcl::PointXYZ defaultPoint2(1.0f, 0.0f, 0.0f);
		clouds_hei->push_back(defaultPoint1);
		clouds_hei->push_back(defaultPoint2);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudOut();
	pcl::ModelCoefficients getCoefficients();
	CylinderParams& getCylinderParams();
	void setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_);

	void fitCylinder(float n_wt, int it_n, double dis, double rad, bool store);
private:
	pcl::PointCloud<pcl::PointNormal>::Ptr clouds_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out;
	pcl::ModelCoefficients::Ptr Coefficients;
	CylinderParams cylinder_params;
};
#endif // !CYLINDER_FITTER_H
