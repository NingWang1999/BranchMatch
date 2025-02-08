#include "cylindrical_registration.h"

void CylindricalRegistration::alignCylinders(const CylinderParams& src_cylinder, const CylinderParams& tgt_cylinder)
{
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t = Eigen::Vector3d::Identity();

	t = tgt_cylinder.v_center - R * src_cylinder.v_center;

	T = Eigen::Translation3d(t) * Eigen::Matrix3d(R);

	std::cout << "Transform Matrix: \n" << T.matrix() << std::endl;
}

void CylindricalRegistration::TrunkCoarseRegistration()
{
	*target_aligned_clouds = *target_clouds;
	pcl::transformPointCloud(*source_clouds, *source_aligned_clouds, T);

	std::cerr << "TrunkCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("source_aligned_clouds.pcd", *source_aligned_clouds);
	std::cerr << "TrunkCoarseRegistration have finished!" << std::endl;
	pcl::io::savePCDFileBinary("target_aligned_clouds.pcd", *target_aligned_clouds);
}

Eigen::Affine3d CylindricalRegistration::getMatix()
{
	return T;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CylindricalRegistration::getSourceAlignedCloud()
{
	return source_aligned_clouds;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CylindricalRegistration::getTargetAlignedCloud()
{
	return target_aligned_clouds;
}

