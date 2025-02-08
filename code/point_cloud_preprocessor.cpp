#include"point_cloud_preprocessor.h"

void PointCloudPreprocessor::ransac_ground(int it_n, float dis, bool ground_ornot)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered0(new pcl::PointCloud<pcl::PointXYZ>);//�ݴ�,���ƽ����Ϻ�ȥ�����
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�洢�ڵ�ĵ��������϶���
	pcl::SACSegmentation<pcl::PointXYZ> seg;//�����ָ����
	seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	seg.setEpsAngle(pcl::deg2rad(0.01));
	seg.setOptimizeCoefficients(true);//����ģ��ϵ����Ҫ�Ż�(��ѡ)
	seg.setModelType(pcl::SACMODEL_PLANE);//����ģ������
	seg.setMethodType(pcl::SAC_RANSAC);//�����������һ���Է���
	seg.setMaxIterations(it_n);//��������������,10000
	seg.setDistanceThreshold(dis);//���þ�����ֵ,0.12m
	seg.setInputCloud(clouds_in);//�������
	seg.segment(*inliers, *ground_plane_coefficients);//�洢�ָ�������������ģ�Ͳ�����coefficient
	if (inliers->indices.size() == 0)
	{
		std::cout << "Error! Couldn't found any inliers!" << std::endl;
	}
	pcl::ExtractIndices<pcl::PointXYZ> extractor;//����ȡ����
	extractor.setInputCloud(clouds_in);
	extractor.setIndices(inliers);
	//Ĭ������£�δ���� setNegative() ��������Ϊ false������ȡ������Щ������Ӧ�ĵ�
	extractor.setNegative(ground_ornot);
	extractor.filter(*cloud_filtered0);

	//ȥ�����ƽ����·����еĵ�
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
	//pcl::io::savePCDFileBinary("no_ground_cloud.pcd", *cloud_filtered);//����������
}

void PointCloudPreprocessor::statiscal_removal(int a, float b)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//�����˲�������
	sor.setInputCloud(clouds_in);//�������
	sor.setMeanK(a);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�������10
	sor.setStddevMulThresh(b);//�����ж��Ƿ�Ϊ��Ⱥ�����ֵ,1.0Ϊһ����׼��������жϵ��k����ƽ���������ȫ�ֵ�1����׼��+ƽ�����룬��Ϊ��Ⱥ��
	sor.filter(*statiscal_filtered);//ִ���˲��������ڵ㵽statiscal_filtered

	clouds_in = statiscal_filtered;
	std::cerr << "After statiscal_filtering,There have: " << statiscal_filtered->size() << " points" << std::endl;
	//pcl::io::savePCDFileBinary("statiscal_cloud.pcd", *statiscal_filtered);//����������
}

void PointCloudPreprocessor::voxel_removal(float leaf_size)
{
	pcl::VoxelGrid <pcl::PointXYZ> sor;//�˲����������
	sor.setInputCloud(clouds_in);//�����������
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);//�����˲�������ʱ���õ����ش�С�Ĳ�����һ�������ش�С�ǳ���߾�Ϊ0.01
	sor.filter(*voxel_filtered);//ִ���²������²���֮��ĵ������ݱ��浽 voxel_filter ��

	clouds_in = voxel_filtered;
	std::cerr << "After voxel_filtering,There have: " << voxel_filtered->size() << " points" << std::endl;
	//pcl::io::savePCDFileBinary("voxel_cloud.pcd", *voxel_filtered);//����������
}

void PointCloudPreprocessor::cluster(float tol, int min_s, int max_s)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd-treeʵ��
	tree->setInputCloud(clouds_in);//�������

	std::vector<pcl::PointIndices> cluster_indices;//�洢��������������
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//��������ʵ��
	ec.setClusterTolerance(tol); //���þ���ʱ�����̶ȣ�������ֵ����0.05m
	ec.setMinClusterSize(min_s);//������С�������
	ec.setMaxClusterSize(max_s);//�������������
	ec.setSearchMethod(tree);//������������Ϊkdtree����
	ec.setInputCloud(clouds_in);//���ô�����ĵ���
	ec.extract(cluster_indices);//�����ȡ���������һ�������У�������ÿ��Ԫ�ش����ȡ��һ������ĵ��Ƶ��±�
	//ֻ��ȡ�������ľ���
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
		//pcl::io::savePCDFileBinary("Clustered_cloud.pcd", *clustered_cloud);//����������
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
	for (size_t i = 0; i < clouds_in->points.size(); i++)//�㵽ƽ��ľ��룬������ȡһ������
	{
		pcl::PointXYZ p = { clouds_in->points[i].x,clouds_in->points[i].y,clouds_in->points[i].z };
		if (pcl::pointToPlaneDistance(p, pla_coe) < b)//�������һ���߶ȷ�Χ�ڵĵ�
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
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// ����һ��KD��
	tree->setInputCloud(clouds_in);//�������

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;// ������� (�ڶ��ֶ���������Ϊ�˴洢����, ��ʹ�ò���Ҳ��Ҫ�������)
	mls.setComputeNormals(true);//��������С���˼�������Ҫ���з��߹���
	mls.setInputCloud(clouds_in);
	mls.setPolynomialOrder(n);//���ö���ʽ�Ľ�����Ĭ��Ϊ2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(r);//�����뾶�������ֲ�ģ��,0.03
	mls.process(*clouds_out);// �����ؽ�

	std::cout << "After MLS,There have��" << clouds_out->size() << " points" << std::endl;//test
	if (store)
	{
		//pcl::io::savePCDFileBinary("mls_trunk_cloud.pcd", *clouds_out);// ����������
	}
}

void PointCloudPreprocessor::normal_estimate(float r, bool store)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// ����һ��KD��
	ne.setInputCloud(clouds_in);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(r);
	ne.compute(*normals);
	pcl::concatenateFields(*clouds_in, *normals, *n_clouds_out);

	std::cout << "After normal_estimate,There have��" << n_clouds_out->size() << " points" << std::endl;//test
	if (store)
	{
		//pcl::io::savePCDFileBinary("n_trunk_cloud.pcd", *n_clouds_out);// ����������
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
