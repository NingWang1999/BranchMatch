#include"pre_branch.h"

void BranchSegment::ring_seg(const CylinderParams& trunk_cylinder_params,
	const pcl::ModelCoefficients::Ptr ground_plane_coefficients, 
	double rmin, double rmax, double dis_zmin, double dis_zmax)//把枝条割出来
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch_(new pcl::PointCloud<pcl::PointXYZ>);//暂存

	size_t j = 0;
	pcl::PointIndices::Ptr inliers_branch(new pcl::PointIndices);//存储分割出的所有满足要求的树枝点云索引
	
	Eigen::Vector4f line_pt = { static_cast<float>(trunk_cylinder_params.center.x()),
		static_cast<float>(trunk_cylinder_params.center.y()),
		static_cast<float>(trunk_cylinder_params.center.z()),0.0f };
	Eigen::Vector4f line_dir = { static_cast<float>(trunk_cylinder_params.axis.x()),
		static_cast<float>(trunk_cylinder_params.axis.y()),
		static_cast<float>(trunk_cylinder_params.axis.z()),0.0f };
	auto r = trunk_cylinder_params.radius;
	pla_coe = { ground_plane_coefficients->values[0],ground_plane_coefficients->values[1], ground_plane_coefficients->values[2], ground_plane_coefficients->values[3] };

	for (size_t i = 0; i < clouds_in->points.size(); i++)//点到轴线的距离用来筛选圆环，点到平面的距离，用来筛选Z轴高度
	{
		Eigen::Vector4f pt = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z,0.0f };
		pcl::PointXYZ p = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z };

		if (pow(r + rmin, 2) < pcl::sqrPointToLineDistance(pt, line_pt, line_dir) &&
			pcl::sqrPointToLineDistance(pt, line_pt, line_dir) < pow(r + rmax, 2))//在圆柱环内的点
		{
			if (pcl::pointToPlaneDistance(p, pla_coe) > dis_zmin &&
				pcl::pointToPlaneDistance(p, pla_coe) < dis_zmax)//距离地面一定高度范围内的点
			{
				all_branch_->push_back(clouds_in->points[i]);
				j++;
			}
		}
	}
	all_branch_->width = static_cast<uint32_t>(j);
	all_branch_->height = 1;
	all_branch_->is_dense = false;
	
	std::cout << "\nAll branch have been segmented!" << std::endl;
	//pcl::io::savePCDFileASCII("all_branch_raw.pcd", *all_branch_);

	//对all_branch进一步进行统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorr;//创建滤波器对象
	sorr.setInputCloud(all_branch_);//输入点云
	sorr.setMeanK(20);//设置在进行统计时考虑查询的邻近点数，10
	sorr.setStddevMulThresh(1);//设置判断是否为离群点的阈值,1.0为一个标准差。即：当判断点的k近邻平均距离大于全局的1倍标准差+平均距离，即为离群点
	sorr.filter(*all_branch);//执行滤波处理保存内点


	std::cout << "all_branch have " << all_branch->size() << " points" << std::endl;

	//pcl::io::savePCDFileASCII("all_branch.pcd", *all_branch);
}
//聚类，挑选，排序
void BranchSegment::cluster_branch(const CylinderParams& trunk_cylinder_params, 
	float tol, int min_s, int max_s)//输出文件有每个树枝的聚类、每个树枝的拟合内点、内点比符合要求的按内点比排序的树枝聚类
{
	std::vector<pcl::PointIndices> cluster_indices;//存储初始聚类索引,向量
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kd-tree实例
	tree->setInputCloud(all_branch);//输入点云
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//创建聚类实例
	ec.setClusterTolerance(tol); //设置聚类时的容忍度（距离阈值），0.02
	ec.setMinClusterSize(min_s);//设置最小聚类大小，10
	ec.setMaxClusterSize(max_s);//设置最大聚类大小，200
	ec.setSearchMethod(tree);//设置搜索方法为kdtree对象
	ec.setInputCloud(all_branch);//设置待聚类的点云
	ec.extract(cluster_indices);//聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个聚类的点云的下标

	//遍历聚类，每个聚类都拟合圆柱，筛选出拟合效果好的，按照“内点比”保存在branch_indices中
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);//暂时存放一个聚类点云
	int i = 1;
	//循环逐个判断聚类
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		clustered_cloud->clear();

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)//通过下标，逐个填充验证聚类
		{
			clustered_cloud->points.push_back(all_branch->points[*pit]);
		}
		clustered_cloud->width = clustered_cloud->points.size();//设置点云属性
		clustered_cloud->height = 1;
		clustered_cloud->is_dense = false;
		std::cout << "\n当前拟合圆柱 " << i << ":" << std::endl;
		//在拟合圆柱前进行mls平滑。mls的输出是normal类型
		PointCloudPreprocessor mls(clustered_cloud);
		mls.mls_suface(3, 0.01, 0);//mls对细小树枝影响很大很差
		//mls.normal_estimate(10, 0);
		
		//拟合圆柱,此处的法线权重，太大导致拟合不出来，太小导致内点太多，看不懂！！！！！！！！！！！！！！！！！！
		CylinderFitter branchc(mls.getCloudsOut());
		branchc.fitCylinder(0.01, 10000, 0.01, 0.03, 0);

		i++;
		//用拟合前后的点数比衡量拟合效果。
		std::cout << "IN-OUT: " << clustered_cloud->size() << " VS " << mls.getCloudsOut()->size() << " VS " << branchc.getCloudOut()->size() << std::endl;
		//内点比
		if (branchc.getCloudOut()->size() != 0)
		{
			std::cout << branchc.getCylinderParams().center << std::endl;
			std::cout << branchc.getCylinderParams().radius << std::endl;

			CylinderParams& params = branchc.getCylinderParams();

			double inlier_ratio = static_cast<double>(branchc.getCloudOut()->size()) / mls.getCloudsOut()->size();
			pcl::PointXYZ p_s = { static_cast<float>(branchc.getCylinderParams().center[0]), 
				static_cast<float>(branchc.getCylinderParams().center[1]),
				static_cast<float>(branchc.getCylinderParams().center[2]) };
			float z_dis = pcl::pointToPlaneDistance(p_s, pla_coe);
//////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if (inlier_ratio > 0.5 && branchc.getCylinderParams().radius > 0.005)//内点占比达到20%，半径太小的不可信
			{
				//std::cout << params.axis << std::endl;

				//调整树枝轴线方向，远离树干方向
				Eigen::Vector3d v_center_point;
				//求一个和树枝中心点相同高度的树干轴线点的坐标
				double mul = (branchc.getCylinderParams().center[2] - trunk_cylinder_params.center[2]) / trunk_cylinder_params.axis[2];
				v_center_point[0] = mul * trunk_cylinder_params.axis[0] + trunk_cylinder_params.center[0];
				v_center_point[1] = mul * trunk_cylinder_params.axis[1] + trunk_cylinder_params.center[1];
				v_center_point[2] = branchc.getCylinderParams().center[2];

				Eigen::Vector3d direction_s(branchc.getCylinderParams().center[0] - v_center_point[0],
					branchc.getCylinderParams().center[1] - v_center_point[1],
					branchc.getCylinderParams().center[2] - v_center_point[2]);
				double dir_ss = direction_s.dot(branchc.getCylinderParams().axis);
				if (dir_ss < 0)
				{
					params.axis[0] = -params.axis[0];
					params.axis[1] = -params.axis[1];
					params.axis[2] = -params.axis[2];
					std::cout << "branch axis should 变变变" << std::endl;
				}

				std::cout << params.axis << std::endl;

				sorted_clusters.push_back({ *it, branchc.getCloudOut()->size(), inlier_ratio, params, z_dis });//保存满足要求的聚类
			}
		}
	}

	//将branch_indices中的聚类依照“内点数目”从大到小排序存储在结构体sorted_clusters，并分别保存点云文件branch_require_match_。
	//应该在每个聚类的结构体内直接存放之前拟合圆柱得到的参数，可以节省下一遍拟合
	std::sort(sorted_clusters.begin(), sorted_clusters.end());
	
	//遍历抽取结果，将其保存（保存满足要求的聚类原始点云）
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered(new pcl::PointCloud<pcl::PointXYZ>);
	int j = 1;
	for (std::vector<ClusterInfo>::const_iterator it = sorted_clusters.begin(); it != sorted_clusters.end(); ++it)
	{
		clustered->clear();
		//通过下标，逐个填充
		for (std::vector<int>::const_iterator pit = it->indices.indices.begin(); pit != it->indices.indices.end(); pit++)
		{
			clustered->points.push_back(all_branch->points[*pit]);
		}
		//设置点云属性
		clustered->width = clustered->points.size();
		clustered->height = 1;
		clustered->is_dense = false;

		double angle = (std::acos(it->center_axis_radius.axis.dot(trunk_cylinder_params.axis)));

		//std::cout << it->center_axis_radius.axis << std::endl;

		std::cout << "潜在配对 " << j << "  包含的点云数量: " << clustered->points.size() << " data points." << std::endl;
		std::cout << "         " << " " << "        内点数量: " << it->inlier_amount << " data points." << std::endl;
		std::cout << "         " << " " << "拟合的圆柱的半径：" << it->center_axis_radius.radius << std::endl;
		std::cout << "         " << " " << "拟合的圆柱的质心高度：" << it->z_relative << std::endl;
		std::cout << "         " << " " << "拟合的圆柱的夹角：" << angle << "\n" << std::endl;

		////生成各个聚类文件名
		//std::stringstream ss;
		//ss << "branch_require_match_" << j << ".pcd";
		////保存聚类点云到文件中
		//pcl::PCDWriter writer2;
		//writer2.write<pcl::PointXYZ>(ss.str(), *clustered, false); //保存点云
		j++;
	}
}

std::vector<ClusterInfo> BranchSegment::getSortedClusters()
{
	return sorted_clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BranchSegment::getAllBranch()
{
	return all_branch;
}
