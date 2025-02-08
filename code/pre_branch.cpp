#include"pre_branch.h"

void BranchSegment::ring_seg(const CylinderParams& trunk_cylinder_params,
	const pcl::ModelCoefficients::Ptr ground_plane_coefficients, 
	double rmin, double rmax, double dis_zmin, double dis_zmax)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch_(new pcl::PointCloud<pcl::PointXYZ>);

	size_t j = 0;
	pcl::PointIndices::Ptr inliers_branch(new pcl::PointIndices);
	
	Eigen::Vector4f line_pt = { static_cast<float>(trunk_cylinder_params.center.x()),
		static_cast<float>(trunk_cylinder_params.center.y()),
		static_cast<float>(trunk_cylinder_params.center.z()),0.0f };
	Eigen::Vector4f line_dir = { static_cast<float>(trunk_cylinder_params.axis.x()),
		static_cast<float>(trunk_cylinder_params.axis.y()),
		static_cast<float>(trunk_cylinder_params.axis.z()),0.0f };
	auto r = trunk_cylinder_params.radius;
	pla_coe = { ground_plane_coefficients->values[0],ground_plane_coefficients->values[1], ground_plane_coefficients->values[2], ground_plane_coefficients->values[3] };

	for (size_t i = 0; i < clouds_in->points.size(); i++)
	{
		Eigen::Vector4f pt = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z,0.0f };
		pcl::PointXYZ p = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z };

		if (pow(r + rmin, 2) < pcl::sqrPointToLineDistance(pt, line_pt, line_dir) &&
			pcl::sqrPointToLineDistance(pt, line_pt, line_dir) < pow(r + rmax, 2))
		{
			if (pcl::pointToPlaneDistance(p, pla_coe) > dis_zmin &&
				pcl::pointToPlaneDistance(p, pla_coe) < dis_zmax)
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
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorr;
	sorr.setInputCloud(all_branch_);
	sorr.setMeanK(20);
	sorr.setStddevMulThresh(1);
	sorr.filter(*all_branch);


	std::cout << "all_branch have " << all_branch->size() << " points" << std::endl;
}
void BranchSegment::cluster_branch(const CylinderParams& trunk_cylinder_params, 
	float tol, int min_s, int max_s)
{
	std::vector<pcl::PointIndices> cluster_indices;
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(all_branch);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tol);
	ec.setMinClusterSize(min_s);
	ec.setMaxClusterSize(max_s);
	ec.setSearchMethod(tree);
	ec.setInputCloud(all_branch);
	ec.extract(cluster_indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	int i = 1;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		clustered_cloud->clear();

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			clustered_cloud->points.push_back(all_branch->points[*pit]);
		}
		clustered_cloud->width = clustered_cloud->points.size();
		clustered_cloud->height = 1;
		clustered_cloud->is_dense = false;
		std::cout << "\nnow cylinder " << i << ":" << std::endl;
		PointCloudPreprocessor mls(clustered_cloud);
		mls.mls_suface(3, 0.01, 0);
		
		CylinderFitter branchc(mls.getCloudsOut());
		branchc.fitCylinder(0.01, 10000, 0.01, 0.03, 0);

		i++;
		std::cout << "IN-OUT: " << clustered_cloud->size() << " VS " << mls.getCloudsOut()->size() << " VS " << branchc.getCloudOut()->size() << std::endl;
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
			if (inlier_ratio > 0.5 && branchc.getCylinderParams().radius > 0.005)
			{
				Eigen::Vector3d v_center_point;
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
					std::cout << "branch axis should be inversed" << std::endl;
				}

				std::cout << params.axis << std::endl;

				sorted_clusters.push_back({ *it, branchc.getCloudOut()->size(), inlier_ratio, params, z_dis });
			}
		}
	}
	std::sort(sorted_clusters.begin(), sorted_clusters.end());
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered(new pcl::PointCloud<pcl::PointXYZ>);
	int j = 1;
	for (std::vector<ClusterInfo>::const_iterator it = sorted_clusters.begin(); it != sorted_clusters.end(); ++it)
	{
		clustered->clear();
		for (std::vector<int>::const_iterator pit = it->indices.indices.begin(); pit != it->indices.indices.end(); pit++)
		{
			clustered->points.push_back(all_branch->points[*pit]);
		}
		clustered->width = clustered->points.size();
		clustered->height = 1;
		clustered->is_dense = false;

		double angle = (std::acos(it->center_axis_radius.axis.dot(trunk_cylinder_params.axis)));

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
