#include"point_cloud_preprocessor.h"

void PointCloudPreprocessor::ransac_ground(int it_n, float dis, bool ground_ornot)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered0(new pcl::PointCloud<pcl::PointXYZ>);//暂存,存放平面拟合后去除结果
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//存储内点的点索引集合对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
	seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	seg.setEpsAngle(pcl::deg2rad(0.01));
	seg.setOptimizeCoefficients(true);//设置模型系数需要优化(可选)
	seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
	seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法
	seg.setMaxIterations(it_n);//设置最大迭代次数,10000
	seg.setDistanceThreshold(dis);//设置距离阈值,0.12m
	seg.setInputCloud(clouds_in);//输入点云
	seg.segment(*inliers, *ground_plane_coefficients);//存储分割结果到点索引；模型参数到coefficient
	if (inliers->indices.size() == 0)
	{
		std::cout << "Error! Couldn't found any inliers!" << std::endl;
	}
	pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
	extractor.setInputCloud(clouds_in);
	extractor.setIndices(inliers);
	//默认情况下（未调用 setNegative() 或已设置为 false），提取的是这些索引对应的点
	extractor.setNegative(ground_ornot);
	extractor.filter(*cloud_filtered0);

	//去除拟合平面的下方所有的点
	double a = ground_plane_coefficients->values[0];
	double b = ground_plane_coefficients->values[1];
	double c = ground_plane_coefficients->values[2];
	double d = ground_plane_coefficients->values[3];
	for (size_t i = 0; i < cloud_filtered0->size(); i++)
	{
		double judge = a * cloud_filtered0->points[i].x + b * cloud_filtered0->points[i].y + c * cloud_filtered0->points[i].z + d;
		if (judge > 0)
		{
			cloud_filtered->push_back(cloud_filtered0->points[i]);
		}
	}

	clouds_in = cloud_filtered;
	std::cerr << "After remove ground,There have: " << cloud_filtered->size() << " points" << std::endl;
	//pcl::io::savePCDFileBinary("no_ground_cloud.pcd", *cloud_filtered);//保存结果点云
}

void PointCloudPreprocessor::statiscal_removal(int a, float b)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//创建滤波器对象
	sor.setInputCloud(clouds_in);//输入点云
	sor.setMeanK(a);//设置在进行统计时考虑查询的邻近点数，10
	sor.setStddevMulThresh(b);//设置判断是否为离群点的阈值,1.0为一个标准差。即：当判断点的k近邻平均距离大于全局的1倍标准差+平均距离，即为离群点
	sor.filter(*statiscal_filtered);//执行滤波处理保存内点到statiscal_filtered

	clouds_in = statiscal_filtered;
	std::cerr << "After statiscal_filtering,There have: " << statiscal_filtered->size() << " points" << std::endl;
	//pcl::io::savePCDFileBinary("statiscal_cloud.pcd", *statiscal_filtered);//保存结果点云
}

void PointCloudPreprocessor::voxel_removal(float leaf_size)
{
	pcl::VoxelGrid <pcl::PointXYZ> sor;//滤波器处理对象
	sor.setInputCloud(clouds_in);//设置输入点云
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);//设置滤波器处理时采用的体素大小的参数，一般设体素大小是长宽高均为0.01
	sor.filter(*voxel_filtered);//执行下采样，下采样之后的点云数据保存到 voxel_filter 中

	clouds_in = voxel_filtered;
	std::cerr << "After voxel_filtering,There have: " << voxel_filtered->size() << " points" << std::endl;
	//pcl::io::savePCDFileBinary("voxel_cloud.pcd", *voxel_filtered);//保存结果点云
}

void PointCloudPreprocessor::cluster(float tol, int min_s, int max_s)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kd-tree实例
	tree->setInputCloud(clouds_in);//输入点云

	std::vector<pcl::PointIndices> cluster_indices;//存储聚类索引，向量
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//创建聚类实例
	ec.setClusterTolerance(tol); //设置聚类时的容忍度（距离阈值），0.05m
	ec.setMinClusterSize(min_s);//设置最小聚类点数
	ec.setMaxClusterSize(max_s);//设置最大聚类点数
	ec.setSearchMethod(tree);//设置搜索方法为kdtree对象
	ec.setInputCloud(clouds_in);//设置待聚类的点云
	ec.extract(cluster_indices);//聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个聚类的点云的下标
	//只提取点数最多的聚类
	std::size_t max_cluster_size = 0;
	int max_cluster_index = -1;
	for (std::size_t i = 0; i < cluster_indices.size(); i++)
	{
		if (cluster_indices[i].indices.size() > max_cluster_size)
		{
			max_cluster_size = cluster_indices[i].indices.size();
			max_cluster_index = i;
		}
	}
	if (max_cluster_index >= 0)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = cluster_indices[max_cluster_index].indices.begin(); pit != cluster_indices[max_cluster_index].indices.end(); pit++)
		{
			clustered_cloud->points.push_back(clouds_in->points[*pit]);
		}
		clustered_cloud->width = clustered_cloud->points.size();
		clustered_cloud->height = 1;

		clouds_in = clustered_cloud;
		std::cerr << "After clustered,There have Single_tree: " << clustered_cloud->size() << " points" << std::endl;
		//pcl::io::savePCDFileBinary("Clustered_cloud.pcd", *clustered_cloud);//保存结果点云
	}
	else
	{
		std::cerr << "No clusters found in the input point cloud." << std::endl;
	}
}

void PointCloudPreprocessor::height_filter(double a, double b)
{
	size_t j = 0;
	Eigen::Vector4f pla_coe = { ground_plane_coefficients->values[0],ground_plane_coefficients->values[1], ground_plane_coefficients->values[2], ground_plane_coefficients->values[3] };
	for (size_t i = 0; i < clouds_in->points.size(); i++)//点到平面的距离，用来截取一段树干
	{
		pcl::PointXYZ p = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z };
		if (pcl::pointToPlaneDistance(p, pla_coe) < b)//距离地面一定高度范围内的点
		{
			if (a < pcl::pointToPlaneDistance(p, pla_coe))
			{
				height_cloud->push_back(clouds_in->points[i]);
				j++;
			}
		}
	}
	height_cloud->width = static_cast<uint32_t>(j);
	height_cloud->height = 1;
	height_cloud->is_dense = false;

	clouds_in = height_cloud;
	std::cerr << "After height_filter,There have Trunk_Cloud: " << height_cloud->size() << " points" << std::endl;
	pcl::io::savePCDFileBinary("height_cloud.pcd", *height_cloud);
}


void PointCloudPreprocessor::mls_suface(int n, float r, bool store)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// 创建一个KD树
	tree->setInputCloud(clouds_in);//输入点云

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	mls.setComputeNormals(true);//设置在最小二乘计算中需要进行法线估计
	mls.setInputCloud(clouds_in);
	mls.setPolynomialOrder(n);//设置多项式的阶数，默认为2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(r);//搜索半径，构建局部模型,0.03
	mls.process(*clouds_out);// 曲面重建

	std::cout << "After MLS,There have：" << clouds_out->size() << " points" << std::endl;//test
	if (store)
	{
		//pcl::io::savePCDFileBinary("mls_trunk_cloud.pcd", *clouds_out);// 保存结果点云
	}
}

void PointCloudPreprocessor::normal_estimate(float r, bool store)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// 创建一个KD树
	ne.setInputCloud(clouds_in);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(r);
	ne.compute(*normals);
	pcl::concatenateFields(*clouds_in, *normals, *n_clouds_out);

	std::cout << "After normal_estimate,There have：" << n_clouds_out->size() << " points" << std::endl;//test
	if (store)
	{
		//pcl::io::savePCDFileBinary("n_trunk_cloud.pcd", *n_clouds_out);// 保存结果点云
	}
}

pcl::ModelCoefficients::Ptr PointCloudPreprocessor::getGroundPlane()
{
	return ground_plane_coefficients;
}

void PointCloudPreprocessor::remove_trunk(const CylinderParams& trunk_axis_, pcl::PointCloud<pcl::PointXYZ>::Ptr have_trunk_, bool store)
{

}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPreprocessor::getCloudsOut()
{
	return clouds_out;
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPreprocessor::getNCloudsOut()
{
	return n_clouds_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::getStatiscalClouds()
{
	return statiscal_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::getVoxelClouds()
{
	return voxel_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::getGroundClouds()
{
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::getHeightClouds()
{
	return height_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::getNotrunk()
{
	return no_trunk_;
}
