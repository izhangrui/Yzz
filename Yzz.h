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
#include <pcl/sample_consensus/ransac.h>          // 采样一致性
#include <pcl/sample_consensus/sac_model_plane.h> // 平面模型
#include <pcl/features/boundary.h>
#include <Eigen/Dense>
//读取obj模型
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

//模块1 读取点云模型==============================================================
void readFile(PointCloudT::Ptr cloud_x, string fileName);




//模块2 点云处理==================================================================

//TR组件
void segment_TR(PointCloudT::Ptr Cloud_TR, vector<PointCloudT::Ptr> cloud_part);
void measure_TR(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//天线面
void segment_TX(PointCloudT::Ptr Cloud_TX, vector<PointCloudT::Ptr> cloud_part);
void measure_TX(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//光轴
void segment_GZ(PointCloudT::Ptr Cloud_GZ, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part);
void measure_GZ(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);

//伺服轴
void segment_SF(PointCloudT::Ptr Cloud_TX, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part);
void measure_SF(vector<PointCloudT::Ptr> cloud_part, vector<double>& result);





//========================================其他======================================
//直通滤波
void passtTrougH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y,
	float x1, float x2, float y1, float y2, float z1, float z2);


//计算6块的参数
double pmd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base);
//计算总体平面度的函数，重载。只用输入一块平面就好
double pmd(pcl::PointCloud<PointT>::Ptr cloud_x);
//计算天线面的平行度
double pxd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base);


//输入向量计算向量之间夹角
double v2angle(double x1, double y1, double z1, double x2, double y2, double z2);

void measureCloud(PointCloudT::Ptr cloud_x, pcl::ModelCoefficients::Ptr coefficients, int type);
//匹配定位
void matchCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y);
//分割
void segCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y, PointCloudT::Ptr cloud_z);
