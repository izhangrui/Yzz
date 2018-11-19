//#include "stdafx.h"
#include "Yzz.h"



//模块1 读取点云模型==============================================================

void readFile(PointCloudT::Ptr cloud_x, string fileName)
{
	string suffixStr = fileName.substr(fileName.find_last_of('.') + 1);
	std::string extply("ply");
	std::string extpcd("pcd");
	std::string extobj("obj");
	if (!((suffixStr == extply) || (suffixStr == extpcd) || (suffixStr == extobj))) {
		std::cout << "文件格式不支持!" << std::endl;
		std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
		return;
	}
	if (suffixStr == extply) {
		if (pcl::io::loadPLYFile(fileName, *cloud_x) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
	}
	else if (suffixStr == extobj) {
		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFileOBJ(fileName, mesh) == -1) {
			cout << "Could not read obj file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_x);
		}
	}
	else {
		if (pcl::io::loadPCDFile(fileName, *cloud_x) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
	}
}



//模块2 点云处理==================================================================

//TR组件

void segment_TR(PointCloudT::Ptr Cloud_TR, vector<PointCloudT::Ptr> cloud_part)
{

	//pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_TR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
	float measure_1sl_z = 12, measure_1sl_y = 127.5, measure_1sl_x = 415,
//	float measure_1sl_z = 26, measure_1sl_y = 105, measure_1sl_x = 493,
		measure_1sr_z = measure_1sl_z + 10,
		measure_1sr_y = measure_1sl_y - 80,
		measure_1sr_x = measure_1sl_x;


	//（第0块定位）
	float measure_01l_z = measure_1sl_z + 20.7, measure_01l_y = measure_1sl_y - 15, measure_01l_x = measure_1sl_x;
	float measure_01r_z = measure_01l_z + 7.5, measure_01r_y = measure_01l_y - 50, measure_01r_x = measure_1sl_x;

	float measure_02l_z = measure_01l_z - 126, measure_02l_y = measure_01l_y, measure_02l_x = measure_1sl_x;
	float measure_02r_z = measure_02l_z + 7.5, measure_02r_y = measure_01r_y, measure_02r_x = measure_1sl_x;



	//第01块区域直通滤波l
	passtTrougH(Cloud_TR, cloud_outl,
		measure_01l_x, measure_01l_x + 10,
		measure_01l_y - 10, measure_01l_y,
		measure_01l_z, measure_01l_z + 5);

	*cloud_view = *cloud_outl + *cloud_view;


	//第01块区域直通滤波r
	passtTrougH(Cloud_TR, cloud_outr,
		measure_01r_x, measure_01r_x + 10,
		measure_01r_y - 10, measure_01r_y,
		measure_01r_z, measure_01r_z + 5);
	*cloud_view = *cloud_outr + *cloud_view;



	//第02块区域直通滤波l
	passtTrougH(Cloud_TR, cloud_outl,
		measure_02l_x, measure_02l_x + 10,
		measure_02l_y - 10, measure_02l_y,
		measure_02l_z, measure_02l_z + 5);


	*cloud_view = *cloud_outl + *cloud_view;


	//第02块区域直通滤波r
	passtTrougH(Cloud_TR, cloud_outr,
		measure_02r_x, measure_02r_x + 10,
		measure_02r_y - 10, measure_02r_y,
		measure_02r_z, measure_02r_z + 5);

	*cloud_view = *cloud_outr + *cloud_view;
	*cloud_0 = *cloud_view;
	pcl::io::savePCDFile("cloud_0.pcd", *cloud_view);
	*cloud_part[0] = *cloud_view;
	passtTrougH(Cloud_TR, cloud_outl,
		measure_1sl_x, measure_1sl_x + 10,
		measure_1sl_y - 15, measure_1sl_y,
		measure_1sl_z, measure_1sl_z + 5);



	//第1块区域直通滤波r
	passtTrougH(Cloud_TR, cloud_outr,
		measure_1sr_x, measure_1sr_x + 10,
		measure_1sr_y - 15, measure_1sr_y,
		measure_1sr_z, measure_1sr_z + 5);

	*cloud_1 = *cloud_outr + *cloud_outl;
	if (cloud_1->points.size() < 1000)
	{
		cout << "cloud is empty" << endl;
	}
	else
	{
		*cloud_view = *cloud_1 + *cloud_view;
		*cloud_part[1] = *cloud_1;
	}

	for (int i = 2; i < 7; i++)
	{

		float measure_2sl_z = measure_1sl_z - 17.7 * (i - 1), measure_2sr_z = measure_2sl_z + 10,
			measure_2sl_x = measure_1sl_x, measure_2sr_x = measure_2sl_x,
			measure_2sr_y = measure_1sr_y, measure_2sl_y = measure_1sl_y;


		//第i块区域直通滤波l
		passtTrougH(Cloud_TR, cloud_outl,
			measure_2sl_x, measure_2sl_x + 10,
			measure_2sl_y - 15, measure_2sl_y,
			measure_2sl_z, measure_2sl_z + 5);



		//第i块区域直通滤波r
		passtTrougH(Cloud_TR, cloud_outr,
			measure_2sr_x, measure_2sr_x + 10,
			measure_2sr_y - 15, measure_2sr_y,
			measure_2sr_z, measure_2sr_z + 5);
		*cloud_i = *cloud_outr + *cloud_outl;
		//yizhicaiyang(cloud_i, cloud_i);
		if (cloud_i->points.size() < 1000)
		{
			cout << "cloud is empty" << endl;
		}
		else
		{
			*cloud_part[i] = *cloud_i;
		}
		//cout << "分割完成" << endl;
	}
}

void measure_TR(vector<PointCloudT::Ptr> cloud_part, vector<double>&result)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_sum += *cloud_part[0];
	for (int i = 1; i < 7; i++)
	{
		//TODO 计算平面度
		*cloud_sum += *cloud_part[i];
		result[i] = pmd(cloud_part[i], cloud_part[0]);
		pcl::io::savePCDFile("cloud" + to_string(i) + ".pcd", *cloud_part[i]);
	}
	//计算单块平面度
	result[7] = pmd(cloud_sum);
}


void segment_TX(PointCloudT::Ptr Cloud_TX, vector<PointCloudT::Ptr> cloud_part)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);





	//第1部分（边框定位）
	float bian1x = 463, bian1y = 53, bian1z = 47, bian2x = bian1x + 5, bian2z = bian1z - 154;
	//第2部分（右耳朵定位）
	float erduo1x = bian1x + 7, erduo1y = bian1y - 55,
		erduo1z = bian1z - 40, erduo2z = erduo1z - 10, erduo3z = erduo1z - 60;
	//第2部分（左耳朵定位）
	float lerduo1x = bian1x + 15, lerduo1y = bian1y + 85,
		lerduo1z = bian1z - 41, lerduo2z = erduo1z - 11, lerduo3z = erduo1z - 61;





	//第1块区域直通滤波上
	passtTrougH(Cloud_TX, cloud_out, bian1x, bian1x + 20, bian1y, bian1y + 45, bian1z, bian1z + 4.5);

	*cloud_out1 = *cloud_out + *cloud_out1;

	//第1块区域直通滤波下
	passtTrougH(Cloud_TX, cloud_out, bian1x, bian1x + 20, bian1y, bian1y + 45, bian2z, bian2z + 4.5);

	*cloud_out1 = *cloud_out1 + *cloud_out;



	//第2块右区域直通滤波

	passtTrougH(Cloud_TX, cloud_out, erduo1x, erduo1x + 20, erduo1y, erduo1y + 3.5, erduo1z, erduo1z + 5);

	*cloud_outr = *cloud_out + *cloud_outr;
	cout << "1size= " << cloud_out->size() << endl;

	passtTrougH(Cloud_TX, cloud_out, erduo1x, erduo1x + 20, erduo1y, erduo1y + 3.5, erduo2z, erduo2z + 5);

	*cloud_outr = *cloud_out + *cloud_outr;
	cout << "2size= " << cloud_out->size() << endl;

	passtTrougH(Cloud_TX, cloud_out, erduo1x, erduo1x + 20, erduo1y, erduo1y + 3.5, erduo3z, erduo3z + 5);

	*cloud_outr = *cloud_out + *cloud_outr;
	cout << "3size= " << cloud_out->size() << endl;
	/*cout << "总体平面度是" << endl;
	pmd(cloud_outr, cloud_outr);*/
	passtTrougH(Cloud_TX, cloud_out, lerduo1x, lerduo1x + 20, lerduo1y, lerduo1y + 3.5, lerduo1z, lerduo1z + 5);

	*cloud_outl = *cloud_out + *cloud_outl;
	cout << "l1size= " << cloud_out->size() << endl;

	passtTrougH(Cloud_TX, cloud_out, lerduo1x, lerduo1x + 20, lerduo1y, lerduo1y + 3.5, lerduo2z, lerduo2z + 5);

	*cloud_outl = *cloud_out + *cloud_outl;
	cout << "l2size= " << cloud_out->size() << endl;

	passtTrougH(Cloud_TX, cloud_out, lerduo1x, lerduo1x + 20, lerduo1y, lerduo1y + 3.5, lerduo3z, lerduo3z + 5);

	*cloud_outl = *cloud_out + *cloud_outl;
	cout << "l3size= " << cloud_out->size() << endl;






	*cloud_out = *cloud_outl + *cloud_outr;

	*cloud_part[0] = *cloud_out;
	*cloud_part[1] = *cloud_out1;

}

void measure_TX(vector<PointCloudT::Ptr> cloud_part, vector<double>& result)
{
	//测量以及数据处理
	vector<ModelCoefficients::Ptr> coe(2);
	for (int i = 0; i < 2; i++)
	{
		coe[i].reset(new ModelCoefficients);
		measureCloud(cloud_part[i], coe[i], 0);
	}

	double cnormal[2][3];//基准面和天线面的法向
	for (int i = 0; i<2; i++)
		for (int j = 0; j < 3; j++)
		{
			cnormal[i][j] = coe[i]->values[j];
		}
	double angle_radar = v2angle(cnormal[0][0], cnormal[0][1], cnormal[0][2], cnormal[1][0], cnormal[1][1], cnormal[1][2]);
	//计算平行度
	result[0] = pxd(cloud_part[0],cloud_part[1]);
}

void segment_GZ(PointCloudT::Ptr Cloud_GZ, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part)
{
	matchCloud(Cloud_GZ, model);
	for (int i = 0; i < 1; i++)
	{
		segCloud(Cloud_GZ, model_part[i], cloud_part[i]);
	}
}

void measure_GZ(vector<PointCloudT::Ptr> cloud_part, vector<double>& result)
{
	vector<ModelCoefficients::Ptr> coe(1);
	for (int i = 0; i < 1; i++)
	{
		coe[i].reset(new ModelCoefficients);
		measureCloud(cloud_part[i], coe[i], 3);//空间圆周模型
	}
	cout << *coe[0] << endl;
	result[0] = coe[0]->values[4];//圆周的法向
	result[1] = coe[0]->values[5];
	result[2] = coe[0]->values[6];
}

void segment_SF(PointCloudT::Ptr Cloud_SF, vector<PointCloudT::Ptr> cloud_part, PointCloudT::Ptr model, vector<PointCloudT::Ptr> model_part)
{
	matchCloud(Cloud_SF, model);//配准
	for (int i = 0; i < cloud_part.size(); i++)
	{
		segCloud(Cloud_SF, model_part[i], cloud_part[i]);//分割
	}
}

void measure_SF(vector<PointCloudT::Ptr> cloud_part, vector<double>& result)
{
	//测量伺服轴，暂时用的是用平面法向来求夹角，没有用圆心
	vector<ModelCoefficients::Ptr> coe(5);//5个平面的参数
	for (int i = 0; i < 5; i++)
	{
		coe[i].reset(new ModelCoefficients);
		measureCloud(cloud_part[i], coe[i], 0);//测量得到5个平面的参数
	}
	double cnormal[5][3];//计算法向 5个平面的法向
	for (int i = 0; i<5; i++)
		for (int j = 0; j < 3; j++)
		{
			cnormal[i][j] = coe[i]->values[j];
		}
	cout << endl;
	cout << *coe[0] << endl;
	cout << *coe[1] << endl;
	double angle_radar = 0.0;
	double angle_radar_sum = 0.0;
	//通过法向计算夹角
	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < 4; j++)
		{
			angle_radar = v2angle(cnormal[i][0], cnormal[i][1], cnormal[i][2], cnormal[j][0], cnormal[j][1], cnormal[j][2]);
			angle_radar_sum += angle_radar;
			cout << angle_radar << endl;
		}
	}
	result[0] = angle_radar_sum / 4;//伺服轴夹角
}















//========================================其他======================================
//直通滤波
void passtTrougH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y,
	float x1, float x2, float y1, float y2, float z1, float z2)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_x);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(z1, z2);
	pass.filter(*cloud_temp1);



	pass.setInputCloud(cloud_temp1);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y1, y2);
	pass.filter(*cloud_temp);

	pass.setInputCloud(cloud_temp);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(x1, x2);
	pass.filter(*cloud_y);
}

//平面度计算

double pmd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base)
{
	//计算平面度
	int para = 0;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//
//	pcl::SACSegmentation<pcl::PointXYZ> sac;
//	sac.setInputCloud(cloud_base);    
//	sac.setMethodType(0);
//	sac.setModelType(0);
//	sac.setDistanceThreshold(0.05);
//	sac.setMaxIterations(50);
//	sac.setOptimizeCoefficients(true);
////	sac.setProbability(0.95);
//	sac.segment(*inliers, *coefficients);
	
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	Eigen::VectorXf  coefficients;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_base));   //针对平面模型的对象
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.05);    //与平面距离小于0.01 的点称为局内点考虑    
	ransac.computeModel();                //执行随机参数估计   
	ransac.getInliers(inliers);            //存储估计所得的局内点
	ransac.getModelCoefficients(coefficients);

//	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
//
//	pcl::SACSegmentation<pcl::PointXYZ> sac1;
//	sac1.setInputCloud(cloud_x);    
//	sac1.setMethodType(0);
//	sac1.setModelType(0);
//	sac1.setDistanceThreshold(0.016);
//	sac1.setMaxIterations(50);
//	sac1.setOptimizeCoefficients(true);
////	sac1.setProbability(0.95);
//	sac1.segment(*inliers1, *coefficients1);
	std::vector<int> inliers1;  //存储局内点集合的点的索引的向量
	Eigen::VectorXf  coefficients1;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p1(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_x));   //针对平面模型的对象
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac1(model_p1);
	ransac1.setDistanceThreshold(0.05);    //与平面距离小于0.01 的点称为局内点考虑    
	ransac1.computeModel();                //执行随机参数估计   
	ransac1.getInliers(inliers1);            //存储估计所得的局内点
	ransac1.getModelCoefficients(coefficients1);
//	cout << coefficients1 << endl;
	if (para == 0)
	{
		double pa, pb, pc, pd, pe, d_sum = 0, d_max = 0, d, d_min = 1000, dd_sum = 0;
		
		if (coefficients1[0] <0)
		{
			coefficients1[0] = -coefficients1[0];
			coefficients1[1] = -coefficients1[1];
			coefficients1[2] = -coefficients1[2];
			coefficients1[3] = -coefficients1[3];
		}
		if (coefficients[0] <0)
		{
			coefficients[0] = -coefficients[0];
			coefficients[1] = -coefficients[1];
			coefficients[2] = -coefficients[2];
			coefficients[3] = -coefficients[3];
		}
		
	    pa = coefficients[0];
		pb = coefficients[1];
		pc = coefficients[2];
		pd = (coefficients[3]);
		
		pe = sqrt(pa*pa + pb*pb + pc*pc);//得到平面方程
		for (size_t i = 0; i < cloud_x->size(); i++)
		{
			d = (pa*cloud_x->points[i].x + pb*cloud_x->points[i].y + pc*cloud_x->points[i].z + (pd)) / pe;
			dd_sum += d;
			d_sum += d*d;
			if (d > d_max)
			{
				d_max = d;
			}
			if (d < d_min)
			{
				d_min = d;
			}
		}
		
		cout << sqrt(d_sum / cloud_x->size()) << "    " << dd_sum / cloud_x->size() << "    ";
		cout << d_min << "    " << d_max << "    " << coefficients[3] - coefficients1[3] << endl;
	}
	return coefficients[3] - coefficients1[3];
}


double pmd(pcl::PointCloud<PointT>::Ptr cloud_x)
{
	//计算平面方程参数
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	Eigen::VectorXf  coefficients;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_x));   //针对平面模型的对象
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.05);    //与平面距离小于0.01 的点称为局内点考虑    
	ransac.computeModel();                //执行随机参数估计   
	ransac.getInliers(inliers);            //存储估计所得的局内点
	ransac.getModelCoefficients(coefficients);


	//计算所有点到平面上的距离
	double pa, pb, pc, pd, pe, d_sum = 0, d_max = 0, d, d_min = 1000, dd_sum = 0;
	if (coefficients[0] <0)
	{
		coefficients[0] = -coefficients[0];
		coefficients[1] = -coefficients[1];
		coefficients[2] = -coefficients[2];
		coefficients[3] = -coefficients[3];
	}

	pa = coefficients[0];
	pb = coefficients[1];
	pc = coefficients[2];
	pd = coefficients[3];

	pe = sqrt(pa*pa + pb*pb + pc*pc);
	for (int i = 0; i < cloud_x->points.size(); i++)
	{
		d = (pa*cloud_x->points[i].x + pb*cloud_x->points[i].y + pc*cloud_x->points[i].z + (pd)) / pe;
		dd_sum += d;
		d_sum += d*d;
		if (d > d_max)
		{
			d_max = d;
		}
		if (d < d_min)
		{
			d_min = d;
		}
	}
	cout << "标准差" << "    " << "平行度" << "    " << "最小值" << "    " << "最大值" << endl;
	cout << sqrt(d_sum / cloud_x->size()) << "    " << dd_sum / cloud_x->size() << "    ";
	cout << d_min << "    " << d_max << "    " << endl;
	//返回最大值最小值
	return -d_min+d_max;
}

double pxd(pcl::PointCloud<PointT>::Ptr cloud_x, pcl::PointCloud<PointT>::Ptr cloud_base)
{
	//计算天线平面的平行度

	//计算基准平面的平面方程
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	Eigen::VectorXf  coefficients;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_base));   //针对平面模型的对象
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.05);    //与平面距离小于0.01 的点称为局内点考虑    
	ransac.computeModel();                //执行随机参数估计   
	ransac.getInliers(inliers);            //存储估计所得的局内点
	ransac.getModelCoefficients(coefficients);



	double pa, pb, pc, pd, pe, d_sum = 0, d_max = 0, d, d_min = 1000, dd_sum = 0;
	if (coefficients[0] <0)
	{
		coefficients[0] = -coefficients[0];
		coefficients[1] = -coefficients[1];
		coefficients[2] = -coefficients[2];
		coefficients[3] = -coefficients[3];
	}

	pa = coefficients[0];
	pb = coefficients[1];
	pc = coefficients[2];
	pd = coefficients[3];
	pe = sqrt(pa*pa + pb*pb + pc*pc);

	//所有点到计算到基准平面的距离
	for (int i = 0; i < cloud_x->points.size(); i++)
	{
		d = (pa*cloud_x->points[i].x + pb*cloud_x->points[i].y + pc*cloud_x->points[i].z + (pd)) / pe;
		dd_sum += d;
		d_sum += d*d;
		if (d > d_max)
		{
			d_max = d;
		}
		if (d < d_min)
		{
			d_min = d;
		}
	}
	//cout << "标准差" << "    " << "平均值" << "    ";
	cout<<"最大值" <<"    "<<"最小值" << endl;
	//cout << sqrt(d_sum / cloud_x->size()) << "    " << dd_sum / cloud_x->size() << "    ";
	cout << d_min << "    " << d_max << "    " << endl;
	return d_max-d_min;
}


double v2angle(double x1, double y1, double z1, double x2, double y2, double z2)
{
	//计算夹角函数
	//输入两个向量，计算向量夹角
	//cout << x1 << y1 << z1 << x2 << y2 << z2 << endl;
	double norm1 = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
	double norm2 = sqrt(x2 * x2 + y2 * y2 + z2 * z2);
	//cout <<"norm_x is:"<< norm_x << endl;
	return acos((x1 * x2 + y1 * y2 + z1 * z2) / (norm1*norm2)) * 180 / 3.141592654;
}

void measureCloud(PointCloudT::Ptr cloud_x, pcl::ModelCoefficients::Ptr coefficients, int type)
{
	//一致采样测量
	//根据type的不同，选择测量不同的模型
	//type=0 是平面 type=3是圆周
	if (cloud_x->points.size() < 3)
		return;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> sac;
	sac.setInputCloud(cloud_x);
	sac.setMethodType(0);
	sac.setModelType(type);
	sac.setDistanceThreshold(0.05);
	sac.setMaxIterations(1000);
	sac.setProbability(0.95);
	sac.segment(*inliers, *coefficients);
}

void matchCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y)
{
	//icp配准函数
	//输入为cloud_x为待配准点云
	//cloud_y基准点云
	//得到转换后的cloud_x  也就是说，这个函数的功能是把cloud_x进行了转换
	double psample = 1;//采样参数
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(psample, psample, psample);
	voxel_grid.setInputCloud(cloud_x);
	PointCloudT::Ptr cloud_src(new PointCloudT);
	voxel_grid.filter(*cloud_src);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid1;
	voxel_grid1.setLeafSize(psample, psample, psample);
	voxel_grid1.setInputCloud(cloud_y);
	PointCloudT::Ptr cloud_tar(new PointCloudT);
	voxel_grid1.filter(*cloud_tar);


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	cout << "starting ..." << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_tar);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tar);
	icp.setMaximumIterations(600);  //最大迭代次数
									//icp.setRANSACIterations(300);
									//icp.setRANSACOutlierRejectionThreshold(0.02);
									//icp.setEuclideanFitnessEpsilon(0.05);//前后两次迭代误差的差值
	icp.setTransformationEpsilon(1e-10); //上次转换与当前转换的差值；
	icp.setMaxCorrespondenceDistance(4); //忽略在此距离之外的点，对配准影响较大
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << "iteration num: " << icp.nr_iterations_ << endl;;
	Eigen::Matrix4d transMatrix;
	transMatrix = icp.getFinalTransformation().cast<double>();
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	transformPointCloud(*cloud_x, *cloud_temp, transMatrix);
	*cloud_x = *cloud_temp;
}


void segCloud(PointCloudT::Ptr cloud_x, PointCloudT::Ptr cloud_y, PointCloudT::Ptr cloud_z)
{
	//邻近分割函数
	//输入cloud_x为原点云，cloud_y为模板点云，cloud_z为风格得到的点云


	//cloud_x为整体点云
	//cloud_y为模板点云
	//cloud_y建立kdtree，查找离cloud_y近的点
	if (cloud_x->points.size() == 0 || cloud_y->points.size() == 0)
		return;
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_y);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 1.5;
	for (size_t i = 0; i < cloud_x->points.size(); ++i)
	{
		if (kdtree.radiusSearch(cloud_x->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			cloud_temp->push_back(cloud_x->points[i]);
		}
	}
	*cloud_z = *cloud_temp;
}
