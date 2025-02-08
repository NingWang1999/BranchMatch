#pragma once
#ifndef ICP_CC_H
#define ICP_CC_H

// CloudCompare
#include <CCCoreLib/PointCloudTpl.h>
#include <CCCoreLib/GenericIndexedCloudPersist.h>
#include <CCCoreLib/CCGeom.h>
#include <CCCoreLib/GeometricalAnalysisTools.h>
#include <CCCoreLib/ReferenceCloud.h>
#include <CCCoreLib/RegistrationTools.h>
#include <CCCoreLib/DistanceComputationTools.h>

// 标准文件
#include <string>
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

class CC_ICP
{
public:
	CC_ICP() :
		source_(new pcl::PointCloud<pcl::PointXYZ>),
		target_(new pcl::PointCloud<pcl::PointXYZ>),
		source_clouds_(new pcl::PointCloud<pcl::PointXYZ>),
		target_clouds_(new pcl::PointCloud<pcl::PointXYZ>),
		source_results(new pcl::PointCloud<pcl::PointXYZ>),
		target_results(new pcl::PointCloud<pcl::PointXYZ>),
		T(Eigen::Affine3d::Identity())
	{}

	void setInitialClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target);
	void setPreprocessClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_clouds);
	//cc提供的ICP，可随机采样、选择重叠率
	void IcpUseCC(double RMSD_, int max_it_, int max_size_, double overlap_);
	void Trans();
	//返回T
	Eigen::Affine3d getMatix();
	//返回结果source
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceResults();

private:
	//存放最初需要配对的点云（用T的）
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
	//存放用于求出T的点云（求T的）
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_clouds_;
	//存放T结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_results;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_results;
	//存放旋转平移矩阵T
	Eigen::Affine3d T;
};

#endif // !ICP_CC_H
