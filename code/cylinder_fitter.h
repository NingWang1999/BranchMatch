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

struct CylinderParams//圆柱参数
{
	Eigen::Vector3d point;//轴上一点
	Eigen::Vector3d axis;//轴向量
	Eigen::Vector4f centroid;//内点质心坐标
	//Eigen::Vector3d center1;//圆柱的轴线上的中心点坐标
	Eigen::Vector3d center;//圆柱内点投影中心点
	Eigen::Vector3d v_center;//竖直圆柱中心点
	float radius;//半径
};
//圆柱拟合类
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

		// 给clouds_hei添加两个默认点
		pcl::PointXYZ defaultPoint1(0.0f, 0.0f, 0.0f); // 第一个默认点在原点
		pcl::PointXYZ defaultPoint2(1.0f, 0.0f, 0.0f); // 第二个默认点在X轴正方向上，举例用
		clouds_hei->push_back(defaultPoint1);
		clouds_hei->push_back(defaultPoint2);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudOut();
	pcl::ModelCoefficients getCoefficients();//获取原始圆柱参数
	CylinderParams& getCylinderParams(); //获取转换圆柱参数，经过单位化处理
	void setCloudHei(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei_);

	//圆柱拟合函数
	//n_wt是法线权重系数，it_n是最大迭代次数，dis是距离阈值，rad是预估最大半径
	void fitCylinder(float n_wt, int it_n, double dis, double rad, bool store);
private:
	pcl::PointCloud<pcl::PointNormal>::Ptr clouds_in;//输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_hei;//高度截取后直接输入的，未经过mls
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out;//输出圆柱内点点云
	pcl::ModelCoefficients::Ptr Coefficients;//原始圆柱参数
	CylinderParams cylinder_params;//圆柱参数转换
};
#endif // !CYLINDER_FITTER_H