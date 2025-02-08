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
	//cc
	void IcpUseCC(double RMSD_, int max_it_, int max_size_, double overlap_);
	void Trans();
	Eigen::Affine3d getMatix();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceResults();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_clouds_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_results;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_results;
	Eigen::Affine3d T;
};

#endif // !ICP_CC_H
