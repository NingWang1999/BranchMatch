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

	// ****************************转换数据******************************
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

	std::cout << "源点云点数量：" << ccCloud->size() << std::endl
		<< "目标点云点数量：" << ccCloudR->size() << std::endl;

	// **************************点云近似距离计算****************************
	unsigned iterationCount = 0;
	double finalRMS = 0.0;
	unsigned finalPointCount = 0;
	//就像在CC软件中的设置
	//Parameters结构体
	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	{
		//iterationCount为0，就使用//配准算法的收敛控制方法——选择基于最大误差的收敛
		parameters.convType = ((iterationCount != 0) ? CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE
			: CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE);
		//设置两次迭代直接的差值作为终止条件
		parameters.minRMSDecrease = RMSD_;/////////////////////////////////
		parameters.nbMaxIterations = max_it_;
		parameters.adjustScale = false;
		//！是否忽略参考点中最远的点
		parameters.filterOutFarthestPoints = true;
		//！每个云的最大点数，如果超过该限制，将随机采样？？？
		parameters.samplingLimit = max_size_;///////////////////////////////////
		//！最重要的理论重叠比率
		parameters.finalOverlapRatio = overlap_;///////////////////////////////

		parameters.modelWeights = nullptr;
		parameters.dataWeights = nullptr;
		parameters.transformationFilters = 0;
		//！最大线程数，用于加速计算
		//源码中写进行线程计算需要网格化，本代码没有使用网格化!!!
		parameters.maxThreadCount = 0;/////////////////////////////////////////
		//是否计算C2M的有符号距离，用于在部分重叠时将云移动到网格的外部
		parameters.useC2MSignedDistances = false; //TODO
		//配准过程中的法线处理——不考虑法线
		parameters.normalsMatching = CCCoreLib::ICPRegistrationTools::NO_NORMAL; 
	}

	//配准结果的类型？		.h文件中的enum RESULT_TYPE
	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
	CCCoreLib::PointProjectionTools::Transformation transform;
	result = CCCoreLib::ICPRegistrationTools::Register
	(ccCloudR.get(),
		nullptr,
		ccCloud.get(),
		parameters,
		transform,
		finalRMS,//用于接收配准过程结束时的最终均方根误差
		finalPointCount//用于接收计算最终RMS时所使用的点的数量,有助于了解配准结果的可靠性，特别是在使用随机采样等策略时
	);

	/*Eigen::Affine3d transformMat;
	transformMat.fill(0);*/

	if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR)
	{
		std::cout << "ICP配准失败！" << std::endl;
	}
	//配准成功
	else if (result == CCCoreLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
	{
		CCCoreLib::SquareMatrixd R = transform.R;
		if (R.isValid() && R.size() == 3)
		{
			for (unsigned j = 0; j < 3; ++j)
			{
				//s是缩放因子，在.h文件中
				T.matrix()(0, j) = static_cast<float>(R.m_values[0][j] * transform.s);
				T.matrix()(1, j) = static_cast<float>(R.m_values[1][j] * transform.s);
				T.matrix()(2, j) = static_cast<float>(R.m_values[2][j] * transform.s);
			}
		}

		T.matrix()(0, 3) = transform.T.x;
		T.matrix()(1, 3) = transform.T.y;
		T.matrix()(2, 3) = transform.T.z;
		//T.matrix()(3, 3) = transform.s;
		T.matrix()(3, 3) = 1;
	}
	std::cout << "RMS:" << finalRMS << "。 With using " << finalPointCount << "points" << std::endl;
	//std::cout << "ICP求得的转换矩阵：\n" << T.matrix() << std::endl;
	//T = transformMat;
}

void CC_ICP::Trans()
{
	pcl::transformPointCloud(*source_, *source_results, T);
	target_results = target_;

	pcl::io::savePCDFileBinary("source_result_ICP.pcd", *source_results);
	pcl::io::savePCDFileBinary("target_result_ICP.pcd", *target_results);
	std::cout << "ICP变换结果保存成功！" << std::endl;
}

Eigen::Affine3d CC_ICP::getMatix()
{
	return T;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CC_ICP::getSourceResults()
{
	return source_results;
}