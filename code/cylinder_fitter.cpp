#include"cylinder_fitter.h"

void CylinderFitter::fitCylinder(float n_wt, int it_n, double dis, double rad, bool store)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trunk(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr normal_trunk(new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	pcl::copyPointCloud(*this->clouds_in, *cloud_trunk);
	pcl::copyPointCloud(*this->clouds_in, *normal_trunk);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(n_wt);
	seg.setMaxIterations(it_n);
	seg.setDistanceThreshold(dis);
	seg.setRadiusLimits(0.0, rad);
	
	seg.setInputCloud(cloud_trunk);
	seg.setInputNormals(normal_trunk);
	seg.segment(*inliers_cylinder, *this->Coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(cloud_trunk);
	extractor.setIndices(inliers_cylinder);
	extractor.filter(*this->clouds_out);

	std::cerr << "After cylinder_fitter,There have: " << inliers_cylinder->indices.size() << " Inner points" << std::endl;
	if (store)
	{
		pcl::io::savePCDFileBinary("trunk_cylinder.pcd", *clouds_out);
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
	
	cylinder_params.axis = { Coefficients->values[3],Coefficients->values[4],Coefficients->values[5] };
	if (cylinder_params.axis[2] < 0)
	{
		cylinder_params.axis[0] = -cylinder_params.axis[0];
		cylinder_params.axis[1] = -cylinder_params.axis[1];
		cylinder_params.axis[2] = -cylinder_params.axis[2];
	}
	cylinder_params.axis.normalize();
	
	cylinder_params.point = { Coefficients->values[0],Coefficients->values[1],Coefficients->values[2] };
	
	pcl::compute3DCentroid(*clouds_out, cylinder_params.centroid);
	
	Eigen::Vector4f direct_axis(Coefficients->values[3], Coefficients->values[4], Coefficients->values[5],0);
	direct_axis.normalize();
	Eigen::Vector4f point_axis(Coefficients->values[0], Coefficients->values[1], Coefficients->values[2],0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& point : clouds_out->points)
	{
		Eigen::Vector4f point_vec(point.x - point_axis[0], point.y - point_axis[1], point.z - point_axis[2],0);
		Eigen::Vector4f proj_point = point_axis + (point_vec.dot(direct_axis)) * direct_axis;
		projectedCloud->push_back(pcl::PointXYZ(proj_point(0), proj_point(1), proj_point(2)));
	}
	auto minmax_x = std::minmax_element(projectedCloud->points.begin(), projectedCloud->points.end(), compare_x);
	pcl::PointXYZ min_point = *minmax_x.first;
	pcl::PointXYZ max_point = *minmax_x.second; 
	pcl::PointXYZ center_point;
	center_point.x = (static_cast<double>(min_point.x) + static_cast<double>(max_point.x)) / 2.0;
	center_point.y = (static_cast<double>(min_point.y) + static_cast<double>(max_point.y)) / 2.0;
	center_point.z = (static_cast<double>(min_point.z) + static_cast<double>(max_point.z)) / 2.0;
	cylinder_params.center = { center_point.x, center_point.y, center_point.z };

	//method 2
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
	cylinder_params.radius = Coefficients->values[6];
	return cylinder_params;
}

void CylinderFitter::setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_)
{
	clouds_hei = clouds_hei_;
}

