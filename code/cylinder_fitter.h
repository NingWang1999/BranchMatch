#pragma once
#ifndef CYLINDER_FITTER_H
#define CYLINDER_FITTER_H

#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <iostream>
#include <algorithm>
#include <pcl/common/distances.h>

static bool compare_x(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.x < b.x);
}
static bool compare_z(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.z < b.z);
}

struct CylinderParams//Բ������
{
	Eigen::Vector3d point;//����һ��
	Eigen::Vector3d axis;//������
	Eigen::Vector4f centroid;//�ڵ���������
	//Eigen::Vector3d center1;//Բ���������ϵ����ĵ�����
	Eigen::Vector3d center;//Բ���ڵ�ͶӰ���ĵ�
	Eigen::Vector3d v_center;//��ֱԲ�����ĵ�
	float radius;//�뾶
};
//Բ�������
class CylinderFitter
{
public:
	CylinderFitter(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) :
		clouds_in(cloud),
		clouds_hei(new pcl::PointCloud<pcl::PointXYZ>),
		clouds_out(new pcl::PointCloud<pcl::PointXYZ>),
		Coefficients(new pcl::ModelCoefficients)
	{
		clouds_hei = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

		// ��clouds_hei�������Ĭ�ϵ�
		pcl::PointXYZ defaultPoint1(0.0f, 0.0f, 0.0f); // ��һ��Ĭ�ϵ���ԭ��
		pcl::PointXYZ defaultPoint2(1.0f, 0.0f, 0.0f); // �ڶ���Ĭ�ϵ���X���������ϣ�������
		clouds_hei->push_back(defaultPoint1);
		clouds_hei->push_back(defaultPoint2);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudOut();
	pcl::ModelCoefficients getCoefficients();//��ȡԭʼԲ������
	CylinderParams& getCylinderParams(); //��ȡת��Բ��������������λ������
	void setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_);

	//Բ����Ϻ���
	//n_wt�Ƿ���Ȩ��ϵ����it_n��������������dis�Ǿ�����ֵ��rad��Ԥ�����뾶
	void fitCylinder(float n_wt, int it_n, double dis, double rad, bool store);
private:
	pcl::PointCloud<pcl::PointNormal>::Ptr clouds_in;//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei;//�߶Ƚ�ȡ��ֱ������ģ�δ����mls
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out;//���Բ���ڵ����
	pcl::ModelCoefficients::Ptr Coefficients;//ԭʼԲ������
	CylinderParams cylinder_params;//Բ������ת��
};
#endif // !CYLINDER_FITTER_H