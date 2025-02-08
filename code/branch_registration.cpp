#include"branch_registration.h"

pcl::PointXYZ intersectionOfTwoLines(const Eigen::Vector3d& line1Point, const Eigen::Vector3d& line1Vec,
	const Eigen::Vector3d& line2Point, const Eigen::Vector3d& line2Vec) 
{
	double det = (line1Vec.cross(line2Vec))[2];
	if (std::abs(det) < 1e-7) 
	{
		std::cerr << "Warning: The two lines are parallel or nearly parallel." << std::endl;
		return pcl::PointXYZ{ NAN, NAN, 0 };
	}
	double t = ((Eigen::Vector3d{ line2Point[0] - line1Point[0], line2Point[1] - line1Point[1], 0 }.cross(line2Vec))[2]) / det;
	pcl::PointXYZ intersectionPoint = 
	{
		static_cast<float>(line1Point[0] + t * line1Vec[0]),
		static_cast<float>(line1Point[1] + t * line1Vec[1]),
		0
	};

	return intersectionPoint;
}
//registration
void BranchRegistration::alignCylinders_with_project(const std::pair<ClusterInfo, ClusterInfo> matched_clusters)
{
	std::cout << "\nStart branch registration!" << std::endl;
	CylinderParams source_branch = matched_clusters.first.center_axis_radius;
	CylinderParams target_branch = matched_clusters.second.center_axis_radius;

	//progection XOY
	Eigen::Vector3d p_source_axis(source_branch.axis[0], source_branch.axis[1], 0);
	Eigen::Vector3d p_target_axis(target_branch.axis[0], target_branch.axis[1], 0);
	p_source_axis.normalize();
	p_target_axis.normalize();
	Eigen::Vector3d p_source_point(source_branch.point[0], source_branch.point[1], 0);
	Eigen::Vector3d p_target_point(target_branch.point[0], target_branch.point[1], 0);

	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();//rotation matrix
	Eigen::Vector3d t(0, 0, 0);//translation vector

	pcl::PointXYZ cross_point = intersectionOfTwoLines(p_source_point, p_source_axis, p_target_point, p_target_axis);
	Eigen::Vector3d rotation_cross_point(static_cast<double>(cross_point.x), static_cast<double>(cross_point.y), 0);

	double dot_product = p_source_axis.dot(p_target_axis);
	double angle_rad = acos(dot_product);

	Eigen::Vector3d rotation_axis(0, 0, 1);

	rotation_axis = p_source_axis.cross(p_target_axis);
	rotation_axis.normalize();

	std::cout << "rotation axis：\n" << rotation_axis << std::endl;

	Eigen::AngleAxisd rotation_vector(angle_rad, rotation_axis);
	Eigen::Matrix3d rotZ = rotation_vector.matrix();
	Eigen::Translation3d translation(-rotation_cross_point.x(), -rotation_cross_point.y(), 0);
	Eigen::Translation3d inverseTranslation(rotation_cross_point.x(), rotation_cross_point.y(), 0);

	T = inverseTranslation * rotZ * translation;
	std::cout << "Transform Matrix: \n" << T.matrix() << std::endl;

}

void BranchRegistration::BranchCoarseRegistration()
{
	pcl::transformPointCloud(*source_clouds, *source_aligned_clouds, T);
	*target_aligned_clouds = *target_clouds;

	std::cerr << "BranchCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("source_regis_clouds.pcd", *source_aligned_clouds);
	std::cerr << "BranchCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("target_regis_clouds.pcd", *target_aligned_clouds);

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
