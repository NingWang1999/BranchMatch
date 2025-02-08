#pragma once
#ifndef BRANCH_REGISTRATION_H
#define BRANCH_REGISTRATION_H

#include "cylinder_fitter.h"
#include "branch_match.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class BranchRegistration
{
public:
    BranchRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud):
        source_clouds(source_cloud),
        target_clouds(target_cloud),
        source_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        target_aligned_clouds(new pcl::PointCloud<pcl::PointXYZ>),
        T(Eigen::Affine3d::Identity())
        
    {}
    //������֦Բ�����������תƽ�Ʊ任����
    //������branch_match�õ��������֦�ĸ���Բ�������Ϣ
    //����ͶӰ����XOY�������ת��Ҫ�����Ƚ�����������׼��
    //�����������ĵ����ڵ�����֦������������ΪԶ������
    void alignCylinders_with_project(const std::pair<ClusterInfo, ClusterInfo> matched_clusters);
    //ʹ��T����source���Ʊ任��target
    void BranchCoarseRegistration();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getSourceAlignedCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTargetAlignedCloud();
    //����T
    Eigen::Affine3d getMatix();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds, target_clouds;//������׼��ĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_clouds, target_aligned_clouds;//��׼��ĵ���
    Eigen::Affine3d T;//��׼�õı任����
};

#endif