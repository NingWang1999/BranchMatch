#include"pre_branch.h"

void BranchSegment::ring_seg(const CylinderParams& trunk_cylinder_params,
	const pcl::ModelCoefficients::Ptr ground_plane_coefficients, 
	double rmin, double rmax, double dis_zmin, double dis_zmax)//��֦�������
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch_(new pcl::PointCloud<pcl::PointXYZ>);//�ݴ�

	size_t j = 0;
	pcl::PointIndices::Ptr inliers_branch(new pcl::PointIndices);//�洢�ָ������������Ҫ�����֦��������
	
	Eigen::Vector4f line_pt = { static_cast<float>(trunk_cylinder_params.center.x()),
		static_cast<float>(trunk_cylinder_params.center.y()),
		static_cast<float>(trunk_cylinder_params.center.z()),0.0f };
	Eigen::Vector4f line_dir = { static_cast<float>(trunk_cylinder_params.axis.x()),
		static_cast<float>(trunk_cylinder_params.axis.y()),
		static_cast<float>(trunk_cylinder_params.axis.z()),0.0f };
	auto r = trunk_cylinder_params.radius;
	pla_coe = { ground_plane_coefficients->values[0],ground_plane_coefficients->values[1], ground_plane_coefficients->values[2], ground_plane_coefficients->values[3] };

	for (size_t i = 0; i < clouds_in->points.size(); i++)//�㵽���ߵľ�������ɸѡԲ�����㵽ƽ��ľ��룬����ɸѡZ��߶�
	{
		Eigen::Vector4f pt = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z,0.0f };
		pcl::PointXYZ p = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z };

		if (pow(r + rmin, 2) < pcl::sqrPointToLineDistance(pt, line_pt, line_dir) &&
			pcl::sqrPointToLineDistance(pt, line_pt, line_dir) < pow(r + rmax, 2))//��Բ�����ڵĵ�
		{
			if (pcl::pointToPlaneDistance(p, pla_coe) > dis_zmin &&
				pcl::pointToPlaneDistance(p, pla_coe) < dis_zmax)//�������һ���߶ȷ�Χ�ڵĵ�
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

	//��all_branch��һ������ͳ���˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorr;//�����˲�������
	sorr.setInputCloud(all_branch_);//�������
	sorr.setMeanK(20);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�������10
	sorr.setStddevMulThresh(1);//�����ж��Ƿ�Ϊ��Ⱥ�����ֵ,1.0Ϊһ����׼��������жϵ��k����ƽ���������ȫ�ֵ�1����׼��+ƽ�����룬��Ϊ��Ⱥ��
	sorr.filter(*all_branch);//ִ���˲��������ڵ�


	std::cout << "all_branch have " << all_branch->size() << " points" << std::endl;

	//pcl::io::savePCDFileASCII("all_branch.pcd", *all_branch);
}
//���࣬��ѡ������
void BranchSegment::cluster_branch(const CylinderParams& trunk_cylinder_params, 
	float tol, int min_s, int max_s)//����ļ���ÿ����֦�ľ��ࡢÿ����֦������ڵ㡢�ڵ�ȷ���Ҫ��İ��ڵ���������֦����
{
	std::vector<pcl::PointIndices> cluster_indices;//�洢��ʼ��������,����
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd-treeʵ��
	tree->setInputCloud(all_branch);//�������
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//��������ʵ��
	ec.setClusterTolerance(tol); //���þ���ʱ�����̶ȣ�������ֵ����0.02
	ec.setMinClusterSize(min_s);//������С�����С��10
	ec.setMaxClusterSize(max_s);//�����������С��200
	ec.setSearchMethod(tree);//������������Ϊkdtree����
	ec.setInputCloud(all_branch);//���ô�����ĵ���
	ec.extract(cluster_indices);//�����ȡ���������һ�������У�������ÿ��Ԫ�ش����ȡ��һ������ĵ��Ƶ��±�

	//�������࣬ÿ�����඼���Բ����ɸѡ�����Ч���õģ����ա��ڵ�ȡ�������branch_indices��
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);//��ʱ���һ���������
	int i = 1;
	//ѭ������жϾ���
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		clustered_cloud->clear();

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)//ͨ���±꣬��������֤����
		{
			clustered_cloud->points.push_back(all_branch->points[*pit]);
		}
		clustered_cloud->width = clustered_cloud->points.size();//���õ�������
		clustered_cloud->height = 1;
		clustered_cloud->is_dense = false;
		std::cout << "\n��ǰ���Բ�� " << i << ":" << std::endl;
		//�����Բ��ǰ����mlsƽ����mls�������normal����
		PointCloudPreprocessor mls(clustered_cloud);
		mls.mls_suface(3, 0.01, 0);//mls��ϸС��֦Ӱ��ܴ�ܲ�
		//mls.normal_estimate(10, 0);
		
		//���Բ��,�˴��ķ���Ȩ�أ�̫������ϲ�������̫С�����ڵ�̫�࣬������������������������������������������
		CylinderFitter branchc(mls.getCloudsOut());
		branchc.fitCylinder(0.01, 10000, 0.01, 0.03, 0);

		i++;
		//�����ǰ��ĵ����Ⱥ������Ч����
		std::cout << "IN-OUT: " << clustered_cloud->size() << " VS " << mls.getCloudsOut()->size() << " VS " << branchc.getCloudOut()->size() << std::endl;
		//�ڵ��
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
			if (inlier_ratio > 0.5 && branchc.getCylinderParams().radius > 0.005)//�ڵ�ռ�ȴﵽ20%���뾶̫С�Ĳ�����
			{
				//std::cout << params.axis << std::endl;

				//������֦���߷���Զ�����ɷ���
				Eigen::Vector3d v_center_point;
				//��һ������֦���ĵ���ͬ�߶ȵ��������ߵ������
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
					std::cout << "branch axis should ����" << std::endl;
				}

				std::cout << params.axis << std::endl;

				sorted_clusters.push_back({ *it, branchc.getCloudOut()->size(), inlier_ratio, params, z_dis });//��������Ҫ��ľ���
			}
		}
	}

	//��branch_indices�еľ������ա��ڵ���Ŀ���Ӵ�С����洢�ڽṹ��sorted_clusters�����ֱ𱣴�����ļ�branch_require_match_��
	//Ӧ����ÿ������Ľṹ����ֱ�Ӵ��֮ǰ���Բ���õ��Ĳ��������Խ�ʡ��һ�����
	std::sort(sorted_clusters.begin(), sorted_clusters.end());
	
	//������ȡ��������䱣�棨��������Ҫ��ľ���ԭʼ���ƣ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered(new pcl::PointCloud<pcl::PointXYZ>);
	int j = 1;
	for (std::vector<ClusterInfo>::const_iterator it = sorted_clusters.begin(); it != sorted_clusters.end(); ++it)
	{
		clustered->clear();
		//ͨ���±꣬������
		for (std::vector<int>::const_iterator pit = it->indices.indices.begin(); pit != it->indices.indices.end(); pit++)
		{
			clustered->points.push_back(all_branch->points[*pit]);
		}
		//���õ�������
		clustered->width = clustered->points.size();
		clustered->height = 1;
		clustered->is_dense = false;

		double angle = (std::acos(it->center_axis_radius.axis.dot(trunk_cylinder_params.axis)));

		//std::cout << it->center_axis_radius.axis << std::endl;

		std::cout << "Ǳ����� " << j << "  �����ĵ�������: " << clustered->points.size() << " data points." << std::endl;
		std::cout << "         " << " " << "        �ڵ�����: " << it->inlier_amount << " data points." << std::endl;
		std::cout << "         " << " " << "��ϵ�Բ���İ뾶��" << it->center_axis_radius.radius << std::endl;
		std::cout << "         " << " " << "��ϵ�Բ�������ĸ߶ȣ�" << it->z_relative << std::endl;
		std::cout << "         " << " " << "��ϵ�Բ���ļнǣ�" << angle << "\n" << std::endl;

		////���ɸ��������ļ���
		//std::stringstream ss;
		//ss << "branch_require_match_" << j << ".pcd";
		////���������Ƶ��ļ���
		//pcl::PCDWriter writer2;
		//writer2.write<pcl::PointXYZ>(ss.str(), *clustered, false); //�������
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
