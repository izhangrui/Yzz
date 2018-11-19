#pragma once
#include <string>
#include <iostream>
#include <fstream> 


#include <opencv2/opencv.hpp> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> 
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/ransac.h>          // ����һ����
#include <pcl/sample_consensus/sac_model_plane.h> // ƽ��ģ��
#include <pcl/features/boundary.h>
#include <Eigen/Dense>
//��ȡobjģ��
#include <pcl/io/obj_io.h>  
#include <pcl/PolygonMesh.h> 
#include <pcl/io/vtk_lib_io.h>
// read ply file
#include <pcl/io/ply_io.h> 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/fpfh.h>

#include <pcl/registration/ia_ransac.h>

#include <pcl/registration/icp.h>

#include <pcl/filters/passthrough.h>


#include <vector>
#include <algorithm>

using namespace pcl;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//ģ��1 ��ȡ����ģ��==============================================================
void readFile(PointCloudT::Ptr cloud_x, string fileName);




//ģ��2 ���ƴ���==================================================================

//TR���
void segment_TR(PointCloudT::Ptr Cloud_TR, vector<PointCloudT::Ptr> cloud_part);
void measure_TR(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//������
void segment_TX(PointCloudT::Ptr Cloud_TX, vector<PointCloudT::Ptr> cloud_part);
void measure_TX(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//����
void segment_GZ(PointCloudT::Ptr Cloud_GZ, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part);
void measure_GZ(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//�ŷ���
void segment_SF(PointCloudT::Ptr Cloud_TX, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part);
void measure_SF(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);





//========================================����======================================
//ֱͨ�˲�
void passtTrougH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y,
	float x1, float x2, float y1, float y2, float z1, float z2);


//����6��Ĳ���
double pmd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base);
//��������ƽ��ȵĺ��������ء�ֻ������һ��ƽ��ͺ�
double pmd(pcl::PointCloud<PointT>::Ptr cloud_x);
//�����������ƽ�ж�
double pxd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base);


//����������������֮��н�
double v2angle(double x1, double y1, double z1, double x2, double y2, double z2);

void measureCloud(PointCloudT::Ptr cloud_x, pcl::ModelCoefficients::Ptr coefficients, int type);
//ƥ�䶨λ
void matchCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y);
//�ָ�
void segCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y, PointCloudT::Ptr cloud_z);
