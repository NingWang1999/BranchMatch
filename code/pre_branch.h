#pragma once
#ifndef PRE_BRANCH_H
#define PRE_BRANCH_H

#include "point_cloud_preprocessor.h"
#include "cylinder_fitter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

struct ClusterInfo
{
	pcl::PointIndices indices;
	size_t inlier_amount;
	double inlier_ratio;
	CylinderParams center_axis_radius;
	float z_relative;
	bool operator<(const ClusterInfo& other) const 
	{
		return z_relative < other.z_relative;
		//return inlier_amount > other.inlier_amount;
	}
};
class BranchSegment
{
public:
	BranchSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :
		clouds_in(cloud),
		all_branch(new pcl::PointCloud<pcl::PointXYZ>)
	{}
	void ring_seg(const CylinderParams& trunk_cylinder_params,
		const pcl::ModelCoefficients::Ptr ground_plane_coefficients,
		double rmin, double rmax, double dis_zmin, double dis_zmax);

	void cluster_branch(const CylinderParams& trunk_cylinder_params, 
		float tol, int min_s, int max_s);

	std::vector<ClusterInfo> getSortedClusters();

	pcl::PointCloud<pcl::PointXYZ>::Ptr getAllBranch();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch;
	std::vector<ClusterInfo> sorted_clusters;
	Eigen::Vector4f pla_coe;
};

#endif
