#pragma once
#ifndef PRE_BRANCH_H
#define PRE_BRANCH_H

#include "point_cloud_preprocessor.h"
#include "cylinder_fitter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

//存放满足要求的聚类的包括其拟合圆柱后的信息
struct ClusterInfo
{
	pcl::PointIndices indices;
	size_t inlier_amount;
	//记录内点比
	double inlier_ratio;
	CylinderParams center_axis_radius;
	//记录每个聚类的质心相对于地面平面的Z轴高度
	float z_relative;
	// 重载小于运算符，用于比较内点数目
	//比较内点数目和Z轴的相对高度（相对地面平面）
	bool operator<(const ClusterInfo& other) const 
	{
		return z_relative < other.z_relative;
		//return inlier_amount > other.inlier_amount;
	}
};
//树枝处理。圆环切割，分割树枝点云；并筛选出适合圆柱拟合的部分树枝
class BranchSegment
{
public:
	//输入的是纯单棵树的点云（经过滤波去除地面和噪点）
	BranchSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :
		clouds_in(cloud),
		all_branch(new pcl::PointCloud<pcl::PointXYZ>)
	{}

	//r+rmin是圆环的最小圆，r+rmax是圆环的最大圆，dis_zmax是离地最高限制
	//需要有输入点云、地面平面参数和树干圆柱参数
	//高处的枝条一般较细，且树干也很细，枝干离得近;离树干远的也不行，太细，近处的为粗枝
	void ring_seg(const CylinderParams& trunk_cylinder_params,
		const pcl::ModelCoefficients::Ptr ground_plane_coefficients,
		double rmin, double rmax, double dis_zmin, double dis_zmax);

	//tol为距离阈值，min_s为最小聚类大小，max_s为最大聚类大小
	//通过聚类首先筛除点数量特别多的（切到树干部分或含分叉树枝），然后筛选圆柱拟合效果好的聚类
	void cluster_branch(const CylinderParams& trunk_cylinder_params, 
		float tol, int min_s, int max_s);

	//获取结果sorted_clusters
	std::vector<ClusterInfo> getSortedClusters();

	//获取all_branch
	//注意：最终得到的sorted_clusters，其中存储的索引是针对all_branch的
	pcl::PointCloud<pcl::PointXYZ>::Ptr getAllBranch();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_in;//输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch;//存储所有分割得到的树枝
	// 将满足内点比要求的聚类及其内点比存储到一个Vector<ClusterInfo>中
	std::vector<ClusterInfo> sorted_clusters;
	//地面参数
	Eigen::Vector4f pla_coe;
};

#endif