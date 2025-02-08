#include"branch_registration.h"

//计算交点坐标
pcl::PointXYZ intersectionOfTwoLines(const Eigen::Vector3d& line1Point, const Eigen::Vector3d& line1Vec,
	const Eigen::Vector3d& line2Point, const Eigen::Vector3d& line2Vec) 
{
	// 检查两直线是否平行（即向量叉积为0）,不能直接norm，会消掉原有的正负号
	double det = (line1Vec.cross(line2Vec))[2];
	if (std::abs(det) < 1e-7) 
	{
		std::cerr << "Warning: The two lines are parallel or nearly parallel." << std::endl;
		return pcl::PointXYZ{ NAN, NAN, 0 }; // 返回NaN表示无交点
	}

	// 计算交点坐标
	double t = ((Eigen::Vector3d{ line2Point[0] - line1Point[0], line2Point[1] - line1Point[1], 0 }.cross(line2Vec))[2]) / det;
	pcl::PointXYZ intersectionPoint = 
	{
		static_cast<float>(line1Point[0] + t * line1Vec[0]),
		static_cast<float>(line1Point[1] + t * line1Vec[1]),
		0
	};

	return intersectionPoint;
}
//配准
void BranchRegistration::alignCylinders_with_project(const std::pair<ClusterInfo, ClusterInfo> matched_clusters)
{
	std::cout << "\nStart branch registration!" << std::endl;
	CylinderParams source_branch = matched_clusters.first.center_axis_radius;
	CylinderParams target_branch = matched_clusters.second.center_axis_radius;

	//将轴线向量和轴上一点都投影到XOY面
	Eigen::Vector3d p_source_axis(source_branch.axis[0], source_branch.axis[1], 0);
	Eigen::Vector3d p_target_axis(target_branch.axis[0], target_branch.axis[1], 0);
	p_source_axis.normalize();
	p_target_axis.normalize();
	Eigen::Vector3d p_source_point(source_branch.point[0], source_branch.point[1], 0);
	Eigen::Vector3d p_target_point(target_branch.point[0], target_branch.point[1], 0);

	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();//旋转矩阵
	Eigen::Vector3d t(0, 0, 0);//平移向量

	//求交点
	pcl::PointXYZ cross_point = intersectionOfTwoLines(p_source_point, p_source_axis, p_target_point, p_target_axis);
	Eigen::Vector3d rotation_cross_point(static_cast<double>(cross_point.x), static_cast<double>(cross_point.y), 0);//交点坐标的投影

	double dot_product = p_source_axis.dot(p_target_axis);//点乘结果用来求夹角,注意！这里的向量模是1
	//double angle_rad = acos(std::abs(dot_product));//反余弦求夹角，结果约束为正
	double angle_rad = acos(dot_product);//反余弦求夹角,0-180

	//
	// 
	// 
	//旋转轴，有正负
	Eigen::Vector3d rotation_axis(0, 0, 1);//初始化旋转轴

	rotation_axis = p_source_axis.cross(p_target_axis);
	rotation_axis.normalize();//单位化

	std::cout << "旋转轴：\n" << rotation_axis << std::endl;

	Eigen::AngleAxisd rotation_vector(angle_rad, rotation_axis);//得到轴角，单位是弧度
	// 绕Z轴旋转的旋转矩阵
	Eigen::Matrix3d rotZ = rotation_vector.matrix();//轴角转为旋转矩阵
	//构建平移矩阵
	Eigen::Translation3d translation(-rotation_cross_point.x(), -rotation_cross_point.y(), 0);
	Eigen::Translation3d inverseTranslation(rotation_cross_point.x(), rotation_cross_point.y(), 0);

	// 构建整体变换矩阵：先平移到旋转中心，再绕Z轴旋转，最后平移回原来的位置
	//是为了把原点移到交点处，所以平移是（-交点）
	T = inverseTranslation * rotZ * translation;
	// T = rotZ * translation;错误
	std::cout << "Transform Matrix: \n" << T.matrix() << std::endl;

}

void BranchRegistration::BranchCoarseRegistration()
{
	pcl::transformPointCloud(*source_clouds, *source_aligned_clouds, T);
	*target_aligned_clouds = *target_clouds;

	std::cerr << "BranchCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("source_regis_clouds.pcd", *source_aligned_clouds);//保存结果点云
	std::cerr << "BranchCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("target_regis_clouds.pcd", *target_aligned_clouds);//保存结果点云

}

pcl::PointCloud<pcl::PointXYZ>::Ptr BranchRegistration::getSourceAlignedCloud()
{
	return source_aligned_clouds;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BranchRegistration::getTargetAlignedCloud()
{
	return target_aligned_clouds;
}

Eigen::Affine3d BranchRegistration::getMatix()
{
	return T;
}