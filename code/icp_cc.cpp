#include"icp_cc.h"

void CC_ICP::setInitialClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
	source_ = source;
	target_ = target;
	return;
}

void CC_ICP::setPreprocessClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds,
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_clouds)
{
	source_clouds_ = source_clouds;
	target_clouds_ = target_clouds;
	return;
}

void CC_ICP::IcpUseCC(double RMSD_,int max_it_,int max_size_,double overlap_)
{
	using PointCloud = CCCoreLib::GenericIndexedCloudPersist;
	using ccPointCloud = CCCoreLib::PointCloudTpl<PointCloud>;
	using PointT = pcl::PointXYZ;

	// ****************************trans data******************************
	std::shared_ptr<ccPointCloud> ccCloud(new ccPointCloud);
	for (int i = 0; i < source_clouds_->points.size(); i++) {
		ccCloud->addPoint(CCVector3(source_clouds_->points[i].x,
			source_clouds_->points[i].y, source_clouds_->points[i].z));
	}

	std::shared_ptr<ccPointCloud> ccCloudR(new ccPointCloud);
	for (int i = 0; i < target_clouds_->points.size(); i++) {
		ccCloudR->addPoint(CCVector3(target_clouds_->points[i].x,
			target_clouds_->points[i].y, target_clouds_->points[i].z));
	}

	std::cout << "source cloud point_count：" << ccCloud->size() << std::endl
		<< "target cloud point_count：" << ccCloudR->size() << std::endl;

	// **************************pointcloud distance****************************
	unsigned iterationCount = 0;
	double finalRMS = 0.0;
	unsigned finalPointCount = 0;
	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	{
		parameters.convType = ((iterationCount != 0) ? CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE
			: CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE);
		parameters.minRMSDecrease = RMSD_;//important
		parameters.nbMaxIterations = max_it_;
		parameters.adjustScale = false;
		parameters.filterOutFarthestPoints = true;
		parameters.samplingLimit = max_size_;//important
		parameters.finalOverlapRatio = overlap_;//important
		parameters.modelWeights = nullptr;
		parameters.dataWeights = nullptr;
		parameters.transformationFilters = 0;
		parameters.maxThreadCount = 0;//important
		parameters.useC2MSignedDistances = false; //TODO
		parameters.normalsMatching = CCCoreLib::ICPRegistrationTools::NO_NORMAL; 
	}

	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
	CCCoreLib::PointProjectionTools::Transformation transform;
	result = CCCoreLib::ICPRegistrationTools::Register
	(ccCloudR.get(),
		nullptr,
		ccCloud.get(),
		parameters,
		transform,
		finalRMS,
		finalPointCount
	);

	if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR)
	{
		std::cout << "ICP配准失败！" << std::endl;
	}
	else if (result == CCCoreLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
	{
		CCCoreLib::SquareMatrixd R = transform.R;
		if (R.isValid() && R.size() == 3)
		{
			for (unsigned j = 0; j < 3; ++j)
			{
				T.matrix()(0, j) = static_cast<float>(R.m_values[0][j] * transform.s);
				T.matrix()(1, j) = static_cast<float>(R.m_values[1][j] * transform.s);
				T.matrix()(2, j) = static_cast<float>(R.m_values[2][j] * transform.s);
			}
		}

		T.matrix()(0, 3) = transform.T.x;
		T.matrix()(1, 3) = transform.T.y;
		T.matrix()(2, 3) = transform.T.z;
		T.matrix()(3, 3) = 1;
	}
	std::cout << "RMS:" << finalRMS << "。 With using " << finalPointCount << "points" << std::endl;
}

void CC_ICP::Trans()
{
	pcl::transformPointCloud(*source_, *source_results, T);
	target_results = target_;

	pcl::io::savePCDFileBinary("source_result_ICP.pcd", *source_results);
	pcl::io::savePCDFileBinary("target_result_ICP.pcd", *target_results);
	std::cout << "ICP success！" << std::endl;
}

Eigen::Affine3d CC_ICP::getMatix()
{
	return T;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CC_ICP::getSourceResults()
{
	return source_results;
}
