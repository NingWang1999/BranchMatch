#include"cylinder_fitter.h"

void CylinderFitter::fitCylinder(float n_wt, int it_n, double dis, double rad, bool store)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trunk(new pcl::PointCloud<pcl::PointXYZ>());//�洢����������Ϣ
	pcl::PointCloud<pcl::Normal>::Ptr normal_trunk(new pcl::PointCloud<pcl::Normal>);//�洢������Ϣ
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//�洢��ϵ�Բ����������

	pcl::copyPointCloud(*this->clouds_in, *cloud_trunk);
	pcl::copyPointCloud(*this->clouds_in, *normal_trunk);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);//�Թ��Ƶ�ģ��ϵ����Ҫ�����Ż�
	seg.setModelType(pcl::SACMODEL_CYLINDER);//���÷ָ�ģ��ΪԲ��
	seg.setMethodType(pcl::SAC_RANSAC);//���÷���ΪRANSAC
	seg.setNormalDistanceWeight(n_wt);//���ñ��淨��Ȩ��ϵ��,0.1
	seg.setMaxIterations(it_n);//��������������,10000
	seg.setDistanceThreshold(dis);//�����ڵ�ľ�����ֵ,0.02
	seg.setRadiusLimits(0.0, rad);//���ù���Բ��ģ�͵İ뾶��Χ,0.5
	
	seg.setInputCloud(cloud_trunk);//����ĵ��ƶ���
	seg.setInputNormals(normal_trunk);//����ķ��߶���
	seg.segment(*inliers_cylinder, *this->Coefficients);//�����������ģ�Ͳ���

	//std::cerr << "Cylinder coefficients: \n" << *Coefficients;

	pcl::ExtractIndices<pcl::PointXYZ> extractor;//����ȡ����
	extractor.setInputCloud(cloud_trunk);
	extractor.setIndices(inliers_cylinder);//Ĭ������£�δ���� setNegative() ��������Ϊ false������ȡ������Щ������Ӧ�ĵ�
	extractor.filter(*this->clouds_out);

	std::cerr << "After cylinder_fitter,There have: " << inliers_cylinder->indices.size() << " Inner points" << std::endl;
	if (store)
	{
		//pcl::io::savePCDFileBinary("trunk_cylinder.pcd", *clouds_out);//�������ڵ����
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
	//������
	cylinder_params.axis = { Coefficients->values[3],Coefficients->values[4],Coefficients->values[5] };
	//����������������������ʹ�䷽��ΪԶ�����ķ���
	//���������Ǳ�֤Z�᷽��Ϊ��
	//Eigen::Vector3d correct_= { static_cast<double>(max_z.x - min_z.x),static_cast<double>(max_z.y - min_z.y), static_cast<double>(max_z.z - min_z.z) };
	if (cylinder_params.axis[2] < 0)
	{
		cylinder_params.axis[0] = -cylinder_params.axis[0];
		cylinder_params.axis[1] = -cylinder_params.axis[1];
		cylinder_params.axis[2] = -cylinder_params.axis[2];
	}
	cylinder_params.axis.normalize();//��λ��
	
	//����һ��
	cylinder_params.point = { Coefficients->values[0],Coefficients->values[1],Coefficients->values[2] };
	
	//��ϵ�Բ�����ڵ���������
	pcl::compute3DCentroid(*clouds_out, cylinder_params.centroid);
	
	//���������ϵ�����ĵ�
	//��ͶӰ��������ᣨx��Ϊ������С���ֵ����������ֱ�ߣ������
	// ����һ��ֱ��ͶӰ�����õ�ˣ���ÿ��������궼ͶӰ�������ϡ�
	Eigen::Vector4f direct_axis(Coefficients->values[3], Coefficients->values[4], Coefficients->values[5],0);
	direct_axis.normalize();
	Eigen::Vector4f point_axis(Coefficients->values[0], Coefficients->values[1], Coefficients->values[2],0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//ͶӰclouds_hei or clouds_out???????
	for (const auto& point : clouds_out->points)
	{
		Eigen::Vector4f point_vec(point.x - point_axis[0], point.y - point_axis[1], point.z - point_axis[2],0);
		Eigen::Vector4f proj_point = point_axis + (point_vec.dot(direct_axis)) * direct_axis;
		projectedCloud->push_back(pcl::PointXYZ(proj_point(0), proj_point(1), proj_point(2)));
	}
	auto minmax_x = std::minmax_element(projectedCloud->points.begin(), projectedCloud->points.end(), compare_x);
	pcl::PointXYZ min_point = *minmax_x.first; // ��Сֵ��
	pcl::PointXYZ max_point = *minmax_x.second; // ���ֵ��
	pcl::PointXYZ center_point;
	center_point.x = (static_cast<double>(min_point.x) + static_cast<double>(max_point.x)) / 2.0;
	center_point.y = (static_cast<double>(min_point.y) + static_cast<double>(max_point.y)) / 2.0;
	center_point.z = (static_cast<double>(min_point.z) + static_cast<double>(max_point.z)) / 2.0;
	cylinder_params.center = { center_point.x, center_point.y, center_point.z };

	//������:�ҵ�Z�᷽�������Сֵ�����ɵõ����ĵ�Zֵ������ֱ�߷���
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

	//�뾶
	cylinder_params.radius = Coefficients->values[6];
	return cylinder_params;
}

void CylinderFitter::setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_)
{
	clouds_hei = clouds_hei_;
}

