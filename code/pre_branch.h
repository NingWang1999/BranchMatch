#pragma once
#ifndef PRE_BRANCH_H
#define PRE_BRANCH_H

#include "point_cloud_preprocessor.h"
#include "cylinder_fitter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

//�������Ҫ��ľ���İ��������Բ�������Ϣ
struct ClusterInfo
{
	pcl::PointIndices indices;
	size_t inlier_amount;
	//��¼�ڵ��
	double inlier_ratio;
	CylinderParams center_axis_radius;
	//��¼ÿ���������������ڵ���ƽ���Z��߶�
	float z_relative;
	// ����С������������ڱȽ��ڵ���Ŀ
	//�Ƚ��ڵ���Ŀ��Z�����Ը߶ȣ���Ե���ƽ�棩
	bool operator<(const ClusterInfo& other) const 
	{
		return z_relative < other.z_relative;
		//return inlier_amount > other.inlier_amount;
	}
};
//��֦����Բ���и�ָ���֦���ƣ���ɸѡ���ʺ�Բ����ϵĲ�����֦
class BranchSegment
{
public:
	//������Ǵ��������ĵ��ƣ������˲�ȥ���������㣩
	BranchSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :
		clouds_in(cloud),
		all_branch(new pcl::PointCloud<pcl::PointXYZ>)
	{}

	//r+rmin��Բ������СԲ��r+rmax��Բ�������Բ��dis_zmax������������
	//��Ҫ��������ơ�����ƽ�����������Բ������
	//�ߴ���֦��һ���ϸ��������Ҳ��ϸ��֦����ý�;������Զ��Ҳ���У�̫ϸ��������Ϊ��֦
	void ring_seg(const CylinderParams& trunk_cylinder_params,
		const pcl::ModelCoefficients::Ptr ground_plane_coefficients,
		double rmin, double rmax, double dis_zmin, double dis_zmax);

	//tolΪ������ֵ��min_sΪ��С�����С��max_sΪ�������С
	//ͨ����������ɸ���������ر��ģ��е����ɲ��ֻ򺬷ֲ���֦����Ȼ��ɸѡԲ�����Ч���õľ���
	void cluster_branch(const CylinderParams& trunk_cylinder_params, 
		float tol, int min_s, int max_s);

	//��ȡ���sorted_clusters
	std::vector<ClusterInfo> getSortedClusters();

	//��ȡall_branch
	//ע�⣺���յõ���sorted_clusters�����д洢�����������all_branch��
	pcl::PointCloud<pcl::PointXYZ>::Ptr getAllBranch();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_in;//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_branch;//�洢���зָ�õ�����֦
	// �������ڵ��Ҫ��ľ��༰���ڵ�ȴ洢��һ��Vector<ClusterInfo>��
	std::vector<ClusterInfo> sorted_clusters;
	//�������
	Eigen::Vector4f pla_coe;
};

#endif