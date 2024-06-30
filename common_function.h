#pragma once
//#define BOOST_TYPEOF_EMULATION
//#include <pcl/registration/icp.h> 

#include <iostream>
#include <opencv2\opencv.hpp>
#include<opencv.hpp>
#include <gdal.h>
#include <gdal_priv.h>
#include<stdio.h>
#include <algorithm>

//Tricp
#include <pcl/recognition/trimmed_icp.h>//tricp头文件
#include <pcl/recognition/ransac_based/auxiliary.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>


#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <cstring>
#include <string.h>
#include <stdio.h>

#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件

//均匀采样
#include <pcl/keypoints/uniform_sampling.h>

//ICP

//#include <pcl/filters/random_sample.h>
//#include <pcl/console/time.h>
//#include "doICP.h"

//icp

using namespace std;

#define num_pt 50000


//误差 距离等权聚类融合，还有交会角,但是所有点都进行加权，不区分好的交会角或者差的交会角，XYZINormal，取网格中心位于好的交会角的邻近30个点高程平均作为搜索点高程，搜索一定半径的所有点进行加权
pcl::PointCloud<pcl::PointXYZ> weighted_rasterize_xyzinormal2(string out_prefix, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process, double grid_width, float radius)
{

	double x_min = 1000000000; double y_min = 1000000000; double z_min = 1000000000;
	double x_max = -1000000000; double y_max = -1000000000;	double z_max = -1000000000;

	for (size_t i = 0; i < cloud_process->points.size(); ++i)
	{
		if (cloud_process->points[i].x <= x_min) x_min = cloud_process->points[i].x;
		if (cloud_process->points[i].y <= y_min) y_min = cloud_process->points[i].y;
		if (cloud_process->points[i].z <= z_min) z_min = cloud_process->points[i].z;
		if (cloud_process->points[i].x >= x_max) x_max = cloud_process->points[i].x;
		if (cloud_process->points[i].y >= y_max) y_max = cloud_process->points[i].y;
		if (cloud_process->points[i].z >= z_max) z_max = cloud_process->points[i].z;
	}
	cout << "x_min=" << x_min << "y_min=" << y_min << endl;
	cout << "x_max=" << x_max << "y_max=" << y_max << endl;
	cout << "z_min=" << z_min << "z_max=" << z_max << endl;
	//x_min = -4407750; x_max = -4407599; //231212myadd
	//y_min = 252717; y_max = 252918;	 //231212myadd
	//x_min = -4407523; x_max = -4407506; //231218myadd tube
	//y_min = 252751; y_max = 252776;	 //231218myadd tube
	/*设置格网参数*/
	double pt0_x = (int)x_min - 10; double pt0_y = (int)y_min - 10;
	double pt1_x = (int)x_max + 1 + 10; double pt1_y = (int)y_max + 1 + 10;
	int grid_num = ((int)(pt1_x - pt0_x) / grid_width) * int((pt1_y - pt0_y) / grid_width);//向下
	//int rows = (int)((pt1_y - pt0_y) / grid_width);
	//int x_num = (pt1_x - pt0_x) / int(grid_width);//列数
	//int y_num = (pt1_y - pt0_y) / int(grid_width);//行数
	//int grid_num = x_num*y_num;

	//将点云按照高程值进行分组
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);
	// 遍历输入点云，筛选高程小于 -1600 的点
	for (const auto& point : cloud_process->points) {
		if (point.z < -1580.0) {  //< -820.0 mtp,  < -1580.0 mhp
			cloud_low->points.push_back(point);
		}
		if (point.z >= -820.0) {  // >= -840.0 mtp,  >= -1600.0 mhp
			cloud_high->points.push_back(point);
		}
	}
	
	// 两个点云的数据互换cloud_high和cloud_process
	// 创建一个临时点云用于存储 cloud1 的数据
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	*temp_cloud = *cloud_process;  // 复制 cloud1 的数据到 temp_cloud
	// 将 cloud2 的数据赋值给 cloud1
	//*cloud_process = *cloud_high;	 //240622myadd
	// 将 temp_cloud 的数据赋值给 cloud2
	*cloud_low = *temp_cloud;

	//投影cloud
	pcl::PointCloud<pcl::PointXYZINormal> cloud1 = *cloud_process;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::copyPointCloud(cloud1, *cloud2);
	for (auto& pt : *cloud2)               //cloud2 平面
	{
		pt.z = 0.0f;
	}

	vector<vector<double>> grid;   //存放grid索引+高程值

	/*新方法：1)遍历网格中心 2）检索该点半径范围点*/

	// 初始化kdTree（平面kd树）
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	// 设置要搜索的点云
	std::vector<int>pointIdxRadiusSearch;													//设置在半径内搜索近邻
	std::vector<float>pointRadiusSquareDistance;											//半径内的近邻对应的平方距离
	kdtree.setInputCloud(cloud2);

	//三维tree
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree2;
	int point_num = 30;	   //搜索最邻近的30个点
	kdtree2.setInputCloud(cloud_process);

	vector<pcl::PointXYZINormal> radius_points;

	for (int i = 0; i < grid_num; ++i)
	{
		int grid_num_y = (int)(pt1_y - pt0_y) / grid_width;// y_num;
		int grid_num_x_i = (int)(i / grid_num_y);
		int grid_num_y_i = (int)(i - grid_num_x_i * grid_num_y);
		double i_y_start = grid_num_y_i * grid_width + pt0_y;
		double i_x_start = grid_num_x_i * grid_width + pt0_x;
		double i_y_center = i_y_start + grid_width / (2.0);
		double i_x_center = i_x_start + grid_width / (2.0);


		pcl::PointXYZINormal searchpoint, searchpoint2;
		searchpoint.x = i_x_center; searchpoint2.x = i_x_center;  //平面栅格中心
		searchpoint.y = i_y_center;	searchpoint2.y = i_y_center;
		searchpoint.z = 0;

		//z值用最邻近点的z值代替（求平面最邻近点，取该点的z值）
		std::vector<int> neighbors;   //存储临近点索引
		std::vector<float> squaredDistances; //存储距离
		int neighborCount = kdtree.nearestKSearch(searchpoint, point_num, neighbors, squaredDistances);	//搜索平面最近邻点
		//searchpoint2.z = cloud_process->points[neighbors[0]].z;	 //用最临近的point_num个点里面考虑交会角来取搜索点的高程，原来这样是不可靠的
		int truenearnum = 0;  double true_heightall = 0;  int badnearnum = 0;  double bad_heightall = 0;
		for (int i = 0; i < point_num; i++) {
			if (cloud_process->points[neighbors[i]].normal_z == 1) {
				truenearnum = truenearnum + 1;
				true_heightall = true_heightall + cloud_process->points[neighbors[i]].z;
			}
			else {
				badnearnum = badnearnum + 1;
				bad_heightall = bad_heightall + cloud_process->points[neighbors[i]].z;
			}
		}
		if (truenearnum > 0) {
			searchpoint2.z = double(true_heightall / truenearnum);
		}
		else {
			searchpoint2.z = double(bad_heightall / badnearnum);
		}
		/*if (i == 64) {
			cout << "searchpoint2.z" << searchpoint2.z << endl;
		}*/
		if (kdtree2.radiusSearch(searchpoint2, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0)//输出搜索到的近邻与对应的半径信息
		{
			vector<double> grid_temp;
			double sum = 0;
			double dis_weight_all = 0;
			double error_weight_all = 0;
			double slew_weight_all = 0;
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
			{
				/*cout << "size=" << pointIdxRadiusSearch.size() << endl;
				cout << "dis=" << (sqrt(pointRadiusSquareDistance[j])) << endl;
				cout << "dis_weight=" << double(1 / (sqrt(pointRadiusSquareDistance[j]))) << endl;
				cout << "error_weight=" << double(1 / cloud_process->points[pointIdxRadiusSearch[j]].intensity) << endl;
				cout << "slew_weight=" << double(cloud_process->points[pointIdxRadiusSearch[j]].normal_z) << endl;*/
				dis_weight_all = dis_weight_all + double(1 / (sqrt(pointRadiusSquareDistance[j])));
				error_weight_all = error_weight_all + double(1 / cloud_process->points[pointIdxRadiusSearch[j]].intensity);
				slew_weight_all = slew_weight_all + double(cloud_process->points[pointIdxRadiusSearch[j]].normal_z);
			}
			//slew_weight_all = slew_weight_all + 0.00001; //为了防止slew_weight_all=0，这样的话slew_weight=-nan
			/*cout << "dis_weight_all=" << dis_weight_all << endl;
			cout << "error_weight_all=" << error_weight_all << endl;
			cout << "slew_weight_all=" << slew_weight_all << endl;*/
			vector<pcl::PointXYZINormal> truepoint;	 vector<double> truedis;
			vector<pcl::PointXYZINormal> middlepoint;	vector<double> middledis;
			vector<pcl::PointXYZINormal> low_confpoint;	  vector<double> low_confdis;
			vector<pcl::PointXYZINormal> pointrun;

			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
			{
				
				/*sum = sum + 0.333 * cloud_process->points[pointIdxRadiusSearch[j]].z * ((double(double(1 / (sqrt(pointRadiusSquareDistance[j]))) / dis_weight_all)) +
						(double(double(1 / cloud_process->points[pointIdxRadiusSearch[j]].intensity) / error_weight_all)) + (double(double(cloud_process->points[pointIdxRadiusSearch[j]].normal_z) / slew_weight_all)));*/
				/*sum = sum + 1 * cloud_process->points[pointIdxRadiusSearch[j]].z * (
						(double(double(1 / cloud_process->points[pointIdxRadiusSearch[j]].intensity) / error_weight_all)) + (double(double(cloud_process->points[pointIdxRadiusSearch[j]].normal_z) / slew_weight_all)));*/
				sum = sum + cloud_process->points[pointIdxRadiusSearch[j]].z * (0.1* (double(double(1 / (sqrt(pointRadiusSquareDistance[j]))) / dis_weight_all)) +
					0.5*(double(double(1 / cloud_process->points[pointIdxRadiusSearch[j]].intensity) / error_weight_all)) + 0.4*(double(double(cloud_process->points[pointIdxRadiusSearch[j]].normal_z) / slew_weight_all)));

			}
			
			//if (sum != 0) {
				grid_temp.push_back(i);
				grid_temp.push_back(sum);
				grid.push_back(grid_temp);
			//}
		}
		else
		{
			//半径范围内没有点
			continue;
		}
	}
	vector<int> grid_first;
	for (int k = 0; k < grid.size(); k++)
	{
		grid_first.push_back(int(grid[k][0]));
	}


	//ground （二维栅格）
	pcl::PointCloud<pcl::PointXYZ> ground_cloud;
	double groundx_max = -100000000; double groundy_max = -1000000;  double groundx_min = 100000000; double groundy_min = 100000000;
	for (int i = 0; i < grid.size(); ++i)
	{
		int index = grid[i][0];
		int grid_num_y = (int)(pt1_y - pt0_y) / grid_width;
		if (index < grid_num_y) continue;    //把第一列去掉？
		if (index > grid_num - grid_num_y) continue; //最后一列？
		int mod_x = index - ((int)(index / grid_num_y)) * grid_num_y;
		if (mod_x == 0 || mod_x == grid_num_y - 1) continue; //去行
		int x_num_i = (int)index / grid_num_y;
		int y_num_i = index - (index / grid_num_y) * grid_num_y;
		double x_start = x_num_i * grid_width + pt0_x;
		double y_start = y_num_i * grid_width + pt0_y;
		pcl::PointXYZ p1_temp;
		p1_temp.x = x_start; p1_temp.y = y_start; p1_temp.z = grid[i][1];
		ground_cloud.push_back(p1_temp);
		if (x_start < groundx_min) groundx_min = x_start;
		if (y_start < groundy_min) groundy_min = y_start;
		if (x_start > groundx_max) groundx_max = x_start;
		if (y_start > groundy_max) groundy_max = y_start;
		//cout << "x_start=" << x_start << "y_start" << y_start << endl;
	}
	cout << "x_min=" << groundx_min << "y_min" << groundy_min << "x_max=" << groundx_max << "y_max" << groundy_max << endl;
	//原始点云投影 231209myadd
	//读取投影的坐标系
	//std::string DEMfilepath1 = out_prefix + "ref_DEM/" + "ref_DEM.tif";// "E:/tri_RJH/code/ldem_80s_20m_div2_clip4.tif";//ldem_clip_30m_+5.tif;ldem_clip_30m_+10.tif
	//GDALDataset* poDataset1;   //GDAL数据集
	//GDALAllRegister();  //注册所有的驱动
	//poDataset1 = (GDALDataset*)GDALOpen(DEMfilepath1.c_str(), GA_ReadOnly);
	//if (poDataset1 == NULL)
	//{
	//	cout << "fail in open files!!!" << endl;
	//}
	//GDALRasterBand* poBand1 = poDataset1->GetRasterBand(1);//获取图像波段
	//int nBands1 = poDataset1->GetRasterCount();
	//int nImgSizeX1 = poDataset1->GetRasterXSize();
	//int nImgSizeY1 = poDataset1->GetRasterYSize();
	//int depth1 = GDALGetDataTypeSize(poDataset1->GetRasterBand(1)->GetRasterDataType()) / 32;
	//const char* sProRef1 = poDataset1->GetProjectionRef();//获取投影信息              
	//double trans1[6];//获取坐标变换系数//获取坐标信息
	//CPLErr aaa1 = poDataset1->GetGeoTransform(trans1);
	//OGRSpatialReference* oSrcSrs1 = new OGRSpatialReference(); //建立空间参考
	//char* c = nullptr;                       //初始化char*类型
	//c = const_cast<char*>(sProRef1);
	//oSrcSrs1->importFromWkt(&c);///GetProjection返回投影信息，以wkt格式输出，最后用ImportFromWkt方法读取字符串并创建地理坐标系、基准面、投影方法、分辨率等。
	//char* pszProjWKT = NULL;
	//char* pszProj4 = NULL;
	//char* pszProjPrettyWKT = NULL;
	//oSrcSrs1->exportToWkt(&pszProjWKT);
	//oSrcSrs1->exportToPrettyWkt(&pszProjPrettyWKT);
	//oSrcSrs1->exportToProj4(&pszProj4);
	//OGRSpatialReference* poTmpSRS1 = oSrcSrs1->CloneGeogCS();//创建地理坐标（经纬度）
	//OGRCoordinateTransformation* poCT1 = NULL;
	//poCT1 = OGRCreateCoordinateTransformation(poTmpSRS1, oSrcSrs1);//建立投影坐标系和地理坐标系的转换关系
	//pcl::PointCloud<pcl::PointXYZ> proj_cloud_original;
	//ofstream outfile1, outfile2;
	///*outfile1.open( "D:/ASP_files/XSY_code/test_lavatube/MTP/mosaic_result/oriPCD_trierror.txt");
	//outfile2.open( "D:/ASP_files/XSY_code/test_lavatube/MTP/mosaic_result/oriPCD_Heighterror.txt");*/
	//for (int j = 0; j < ground_cloud.size(); j++)
	//{
	//	Vector3 pos_BLH1 = XYZ_to_BHL(ground_cloud[j].x, ground_cloud[j].y, ground_cloud[j].z);//要改
	//	double L_lon, L_lat;
	//	L_lon = pos_BLH1.y();
	//	L_lat = pos_BLH1.x();
	//	poCT1->Transform(1, &L_lon, &L_lat);

	//	pcl::PointXYZ pp;
	//	pp.x = L_lon; pp.y = L_lat; pp.z = pos_BLH1.z(); //pp.intensity = ground_cloud[j].normal_x;
	//	proj_cloud_original.push_back(pp);

	//	//输出误差两个字段的信息
	//	/*outfile1 << std::setprecision(10) << ground_cloud[j].normal_x << endl;
	//	outfile2 << std::setprecision(10) << ground_cloud[j].normal_y << endl;*/
	//}
	//outfile1.close();
	//outfile2.close();
	////pcl::io::savePCDFileBinary<pcl::PointXYZI>(out_prefix  + "mosaic_result/xyzi_proj_original.pcd", proj_cloud_original);

	//return proj_cloud_original;  //231209myadd
	return ground_cloud;
}

//240620针对熔岩管部分的点云融合，融合时考虑同一个格网内按照不同高程值进行划分
pcl::PointCloud<pcl::PointXYZ> weighted_rasterize_xyzinormal3_low_error(string out_prefix, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process, double grid_width, float radius) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);

	for (const auto& point : cloud_process->points) {
		if (point.z < 820.0) {   //< -820.0 mtp,  < -1580.0 mhp
			cloud_low->points.push_back(point);
		}
		if (point.z >= -1600.0) {
			cloud_high->points.push_back(point);
		}
	}

	double x_min = 1000000000; double y_min = 1000000000; double z_min = 1000000000;
	double x_max = -1000000000; double y_max = -1000000000; double z_max = -1000000000;

	for (size_t i = 0; i < cloud_low->points.size(); ++i) {
		if (cloud_low->points[i].x <= x_min) x_min = cloud_low->points[i].x;
		if (cloud_low->points[i].y <= y_min) y_min = cloud_low->points[i].y;
		if (cloud_low->points[i].z <= z_min) z_min = cloud_low->points[i].z;
		if (cloud_low->points[i].x >= x_max) x_max = cloud_low->points[i].x;
		if (cloud_low->points[i].y >= y_max) y_max = cloud_low->points[i].y;
		if (cloud_low->points[i].z >= z_max) z_max = cloud_low->points[i].z;
	}

	std::cout << "x_min=" << x_min << " y_min=" << y_min << std::endl;
	std::cout << "x_max=" << x_max << " y_max=" << y_max << std::endl;
	std::cout << "z_min=" << z_min << " z_max=" << z_max << std::endl;

	x_min = -4407531; x_max = -4407416; //231212myadd	tube mtp
	y_min = 252690; y_max = 252828;	 //231212myadd tube mtp
	//x_min = 3625717; x_max = 3625794;	 //mhp
	//y_min = 427287; y_max = 427356;		 //mhp

	double pt0_x = (int)x_min - 10; double pt0_y = (int)y_min - 10;
	double pt1_x = (int)x_max + 1 + 10; double pt1_y = (int)y_max + 1 + 10;
	int grid_num_x = (int)((pt1_x - pt0_x) / grid_width);
	int grid_num_y = (int)((pt1_y - pt0_y) / grid_width);
	int grid_num = grid_num_x * grid_num_y;

	std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> grid(grid_num_x, std::vector<std::vector<pcl::PointXYZINormal>>(grid_num_y));

	// 将点按格网分类
	for (const auto& point : cloud_low->points) {
		int grid_x = (int)((point.x - pt0_x) / grid_width);
		int grid_y = (int)((point.y - pt0_y) / grid_width);
		if (grid_x >= 0 && grid_x < grid_num_x && grid_y >= 0 && grid_y < grid_num_y) {
			grid[grid_x][grid_y].push_back(point);
		}
	}

	pcl::PointCloud<pcl::PointXYZ> ground_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	kdtree.setInputCloud(cloud_low);

	// 遍历每个格网
	for (int i = 0; i < grid_num_x; ++i) {
		for (int j = 0; j < grid_num_y; ++j) {
			auto& cell_points = grid[i][j];
			if (cell_points.empty()) continue;

			// 获取格网内点的高程区间
			std::vector<double> heights;
			for (const auto& point : cell_points) {
				heights.push_back(point.z);
			}
			std::sort(heights.begin(), heights.end());

			double min_height = heights.front();
			double max_height = heights.back();

			// 按0.5米的高程间距依次更换搜索点进行加权处理
			for (double z = min_height; z <= max_height; z += 0.5) {
				pcl::PointXYZINormal searchpoint;
				searchpoint.x = pt0_x + i * grid_width + grid_width / 2.0;
				searchpoint.y = pt0_y + j * grid_width + grid_width / 2.0;
				searchpoint.z = z;

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquareDistance;
				float min_distance = 1e-2; // 防止距离值过小
				float distance = 1e-2;
				if (kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0) {
					double sum = 0;
					double dis_weight_all = 0;
					double error_weight_all = 0;
					double slew_weight_all = 0;

					for (const auto& idx : pointIdxRadiusSearch) {
						//cout << idx << endl;
						//cout << pointIdxRadiusSearch.size() << " " << pointIdxRadiusSearch[0] << endl;
						//cout << pointRadiusSquareDistance[idx] << " " << 1 / (sqrt(pointRadiusSquareDistance[idx])) << endl;
						distance = pointRadiusSquareDistance[idx];
						if (pointRadiusSquareDistance[idx] < min_distance) {
							distance = min_distance;
						}
						dis_weight_all += double(1 / (sqrt(distance)));
						error_weight_all += double(1 / cloud_low->points[idx].intensity);
						slew_weight_all += double(cloud_low->points[idx].normal_z);
					}

					for (const auto& idx : pointIdxRadiusSearch) {
						float distance = pointRadiusSquareDistance[idx];
						if (pointRadiusSquareDistance[idx] < min_distance) {
							distance = min_distance;
						}
						sum += cloud_low->points[idx].z * (0.1 * (double(1 / (sqrt(distance)) / dis_weight_all)) +
							0.5 * (double(1 / cloud_low->points[idx].intensity) / error_weight_all) +
							0.4 * (double(cloud_low->points[idx].normal_z) / slew_weight_all));
					}

					pcl::PointXYZ p1_temp;
					p1_temp.x = searchpoint.x;
					p1_temp.y = searchpoint.y;
					p1_temp.z = sum;
					ground_cloud.push_back(p1_temp);
				}
			}
		}
	}

	std::cout << "x_min=" << pt0_x << " y_min=" << pt0_y << " x_max=" << pt1_x << " y_max=" << pt1_y << std::endl;


	return ground_cloud;
}

//240620针对熔岩管部分的点云融合，融合时考虑同一个格网内按照不同高程值进行划分
pcl::PointCloud<pcl::PointXYZ> weighted_rasterize_xyzinormal3_low(string out_prefix, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process, double grid_width, float radius) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);

	for (const auto& point : cloud_process->points) {
		if (point.z < -820.0) {   //< -820.0 mtp,  < -1580.0 mhp
			cloud_low->points.push_back(point);
		}
		if (point.z >= -1600.0) {
			cloud_high->points.push_back(point);
		}
	}

	double x_min = 1000000000; double y_min = 1000000000; double z_min = 1000000000;
	double x_max = -1000000000; double y_max = -1000000000; double z_max = -1000000000;

	for (size_t i = 0; i < cloud_low->points.size(); ++i) {
		if (cloud_low->points[i].x <= x_min) x_min = cloud_low->points[i].x;
		if (cloud_low->points[i].y <= y_min) y_min = cloud_low->points[i].y;
		if (cloud_low->points[i].z <= z_min) z_min = cloud_low->points[i].z;
		if (cloud_low->points[i].x >= x_max) x_max = cloud_low->points[i].x;
		if (cloud_low->points[i].y >= y_max) y_max = cloud_low->points[i].y;
		if (cloud_low->points[i].z >= z_max) z_max = cloud_low->points[i].z;
	}

	std::cout << "x_min=" << x_min << " y_min=" << y_min << std::endl;
	std::cout << "x_max=" << x_max << " y_max=" << y_max << std::endl;
	std::cout << "z_min=" << z_min << " z_max=" << z_max << std::endl;

	x_min = -4407531; x_max = -4407416; //231212myadd	tube mtp
	y_min = 252690; y_max = 252828;	 //231212myadd tube mtp
	//x_min = 3625717; x_max = 3625794;	 //mhp
	//y_min = 427287; y_max = 427356;		 //mhp

	double pt0_x = (int)x_min - 10; double pt0_y = (int)y_min - 10;
	double pt1_x = (int)x_max + 1 + 10; double pt1_y = (int)y_max + 1 + 10;
	int grid_num_x = (int)((pt1_x - pt0_x) / grid_width);
	int grid_num_y = (int)((pt1_y - pt0_y) / grid_width);
	int grid_num = grid_num_x * grid_num_y;

	std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> grid(grid_num_x, std::vector<std::vector<pcl::PointXYZINormal>>(grid_num_y));

	// 将点按格网分类
	for (const auto& point : cloud_low->points) {
		int grid_x = (int)((point.x - pt0_x) / grid_width);
		int grid_y = (int)((point.y - pt0_y) / grid_width);
		if (grid_x >= 0 && grid_x < grid_num_x && grid_y >= 0 && grid_y < grid_num_y) {
			grid[grid_x][grid_y].push_back(point);
		}
	}

	pcl::PointCloud<pcl::PointXYZ> ground_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	kdtree.setInputCloud(cloud_low);

	// 遍历每个格网
	for (int i = 0; i < grid_num_x; ++i) {
		for (int j = 0; j < grid_num_y; ++j) {
			auto& cell_points = grid[i][j];
			if (cell_points.empty()) continue;

			// 获取格网内点的高程区间
			std::vector<double> heights;
			for (const auto& point : cell_points) {
				heights.push_back(point.z);
			}
			std::sort(heights.begin(), heights.end());

			double min_height = heights.front();
			double max_height = heights.back();

			// 按0.5米的高程间距依次更换搜索点进行加权处理
			for (double z = min_height; z <= max_height; z += 0.5) {
				pcl::PointXYZINormal searchpoint;
				searchpoint.x = pt0_x + i * grid_width + grid_width / 2.0;
				searchpoint.y = pt0_y + j * grid_width + grid_width / 2.0;
				searchpoint.z = z;

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquareDistance;
				float min_distance = 1e-2; // 防止距离值过小
				if (kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0) {
					double sum = 0;
					double dis_weight_all = 0;
					double error_weight_all = 0;
					double slew_weight_all = 0;

					for (const auto& idx : pointIdxRadiusSearch) {
						//cout << idx << endl;
						//cout << pointIdxRadiusSearch.size() << " " << pointIdxRadiusSearch[0] << endl;
						//cout << pointRadiusSquareDistance[idx] << " " << 1 / (sqrt(pointRadiusSquareDistance[idx])) << endl;
						float distance = pointRadiusSquareDistance[idx];
						if (pointRadiusSquareDistance[idx] < min_distance) {
							distance = min_distance;
						}
						dis_weight_all += double(1 / (sqrt(distance)));
						error_weight_all += double(1 / cloud_low->points[idx].intensity);
						slew_weight_all += double(cloud_low->points[idx].normal_z);
					}

					for (const auto& idx : pointIdxRadiusSearch) {
						float distance = pointRadiusSquareDistance[idx];
						if (pointRadiusSquareDistance[idx] < min_distance) {
							distance = min_distance;
						}
						sum += cloud_low->points[idx].z * (0.1 * (double(1 / (sqrt(distance)) / dis_weight_all)) +
							0.5 * (double(1 / cloud_low->points[idx].intensity) / error_weight_all) +
							0.4 * (double(cloud_low->points[idx].normal_z) / slew_weight_all));
					}

					pcl::PointXYZ p1_temp;
					p1_temp.x = searchpoint.x;
					p1_temp.y = searchpoint.y;
					p1_temp.z = sum;
					ground_cloud.push_back(p1_temp);
				}
			}
		}
	}

	std::cout << "x_min=" << pt0_x << " y_min=" << pt0_y << " x_max=" << pt1_x << " y_max=" << pt1_y << std::endl;

	return ground_cloud;
}


