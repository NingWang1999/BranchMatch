#include"cylinder_fitter.h"

void CylinderFitter::fitCylinder(float n_wt, int it_n, double dis, double rad, bool store)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trunk(new pcl::PointCloud<pcl::PointXYZ>());//存储点云坐标信息
	pcl::PointCloud<pcl::Normal>::Ptr normal_trunk(new pcl::PointCloud<pcl::Normal>);//存储法线信息
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//存储拟合的圆柱点云索引

	pcl::copyPointCloud(*this->clouds_in, *cloud_trunk);
	pcl::copyPointCloud(*this->clouds_in, *normal_trunk);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);//对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);//设置分割模型为圆柱
	seg.setMethodType(pcl::SAC_RANSAC);//设置方法为RANSAC
	seg.setNormalDistanceWeight(n_wt);//设置表面法线权重系数,0.1
	seg.setMaxIterations(it_n);//设置最大迭代次数,10000
	seg.setDistanceThreshold(dis);//设置内点的距离阈值,0.02
	seg.setRadiusLimits(0.0, rad);//设置估计圆柱模型的半径范围,0.5
	
	seg.setInputCloud(cloud_trunk);//输入的点云对象
	seg.setInputNormals(normal_trunk);//输入的法线对象
	seg.segment(*inliers_cylinder, *this->Coefficients);//输出的索引和模型参数

	//std::cerr << "Cylinder coefficients: \n" << *Coefficients;

	pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
	extractor.setInputCloud(cloud_trunk);
	extractor.setIndices(inliers_cylinder);//默认情况下（未调用 setNegative() 或已设置为 false），提取的是这些索引对应的点
	extractor.filter(*this->clouds_out);

	std::cerr << "After cylinder_fitter,There have: " << inliers_cylinder->indices.size() << " Inner points" << std::endl;
	if (store)
	{
		//pcl::io::savePCDFileBinary("trunk_cylinder.pcd", *clouds_out);//保存结果内点点云
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CylinderFitter::getCloudOut()
{
	return clouds_out;
}

pcl::ModelCoefficients CylinderFitter::getCoefficients()
{
	return *this->Coefficients;
}

CylinderParams& CylinderFitter::getCylinderParams()
{
	//轴向量
	cylinder_params.axis = { Coefficients->values[3],Coefficients->values[4],Coefficients->values[5] };
	//对树干轴向量做出修正，使其方向为远离地面的方向
	//具体做法是保证Z轴方向为正
	//Eigen::Vector3d correct_= { static_cast<double>(max_z.x - min_z.x),static_cast<double>(max_z.y - min_z.y), static_cast<double>(max_z.z - min_z.z) };
	if (cylinder_params.axis[2] < 0)
	{
		cylinder_params.axis[0] = -cylinder_params.axis[0];
		cylinder_params.axis[1] = -cylinder_params.axis[1];
		cylinder_params.axis[2] = -cylinder_params.axis[2];
	}
	cylinder_params.axis.normalize();//单位化
	
	//轴上一点
	cylinder_params.point = { Coefficients->values[0],Coefficients->values[1],Coefficients->values[2] };
	
	//拟合的圆柱的内点质心坐标
	pcl::compute3DCentroid(*clouds_out, cylinder_params.centroid);
	
	//计算轴线上点的中心点
	//找投影点的任意轴（x轴为例）最小最大值，带入轴线直线，求出点
	// 方法一：直接投影。利用点乘，将每个点的坐标都投影到轴线上。
	Eigen::Vector4f direct_axis(Coefficients->values[3], Coefficients->values[4], Coefficients->values[5],0);
	direct_axis.normalize();
	Eigen::Vector4f point_axis(Coefficients->values[0], Coefficients->values[1], Coefficients->values[2],0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//投影clouds_hei or clouds_out???????
	for (const auto& point : clouds_out->points)
	{
		Eigen::Vector4f point_vec(point.x - point_axis[0], point.y - point_axis[1], point.z - point_axis[2],0);
		Eigen::Vector4f proj_point = point_axis + (point_vec.dot(direct_axis)) * direct_axis;
		projectedCloud->push_back(pcl::PointXYZ(proj_point(0), proj_point(1), proj_point(2)));
	}
	auto minmax_x = std::minmax_element(projectedCloud->points.begin(), projectedCloud->points.end(), compare_x);
	pcl::PointXYZ min_point = *minmax_x.first; // 最小值点
	pcl::PointXYZ max_point = *minmax_x.second; // 最大值点
	pcl::PointXYZ center_point;
	center_point.x = (static_cast<double>(min_point.x) + static_cast<double>(max_point.x)) / 2.0;
	center_point.y = (static_cast<double>(min_point.y) + static_cast<double>(max_point.y)) / 2.0;
	center_point.z = (static_cast<double>(min_point.z) + static_cast<double>(max_point.z)) / 2.0;
	cylinder_params.center = { center_point.x, center_point.y, center_point.z };

	//方法二:找到Z轴方向最大最小值，即可得到中心点Z值，带入直线方程
	pcl::PointXYZ v_center_point;
	auto minmax_z = std::minmax_element(clouds_hei->points.begin(), clouds_hei->points.end(), compare_z);
	pcl::PointXYZ min_z = *minmax_z.first;
	pcl::PointXYZ max_z = *minmax_z.second;
	double z_cnt = (static_cast<double>(min_z.z) + static_cast<double>(max_z.z)) / 2.0;
	double mul = (z_cnt - Coefficients->values[2]) / Coefficients->values[5];
	v_center_point.x = mul * Coefficients->values[3] + Coefficients->values[0];
	v_center_point.y = mul * Coefficients->values[4] + Coefficients->values[1];
	v_center_point.z = z_cnt;
	cylinder_params.v_center = { v_center_point.x, v_center_point.y, v_center_point.z };

	//半径
	cylinder_params.radius = Coefficients->values[6];
	return cylinder_params;
}

void CylinderFitter::setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_)
{
	clouds_hei = clouds_hei_;
}

