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
#include <pcl/recognition/trimmed_icp.h>//tricpͷ�ļ�
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
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�

//���Ȳ���
#include <pcl/keypoints/uniform_sampling.h>

//ICP

//#include <pcl/filters/random_sample.h>
//#include <pcl/console/time.h>
//#include "doICP.h"

//icp

using namespace std;

#define num_pt 50000


//��� �����Ȩ�����ںϣ����н����,�������е㶼���м�Ȩ�������ֺõĽ���ǻ��߲�Ľ���ǣ�XYZINormal��ȡ��������λ�ںõĽ���ǵ��ڽ�30����߳�ƽ����Ϊ������̣߳�����һ���뾶�����е���м�Ȩ
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
	/*���ø�������*/
	double pt0_x = (int)x_min - 10; double pt0_y = (int)y_min - 10;
	double pt1_x = (int)x_max + 1 + 10; double pt1_y = (int)y_max + 1 + 10;
	int grid_num = ((int)(pt1_x - pt0_x) / grid_width) * int((pt1_y - pt0_y) / grid_width);//����
	//int rows = (int)((pt1_y - pt0_y) / grid_width);
	//int x_num = (pt1_x - pt0_x) / int(grid_width);//����
	//int y_num = (pt1_y - pt0_y) / int(grid_width);//����
	//int grid_num = x_num*y_num;

	//�����ư��ո߳�ֵ���з���
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);
	// ����������ƣ�ɸѡ�߳�С�� -1600 �ĵ�
	for (const auto& point : cloud_process->points) {
		if (point.z < -1580.0) {  //< -820.0 mtp,  < -1580.0 mhp
			cloud_low->points.push_back(point);
		}
		if (point.z >= -820.0) {  // >= -840.0 mtp,  >= -1600.0 mhp
			cloud_high->points.push_back(point);
		}
	}
	
	// �������Ƶ����ݻ���cloud_high��cloud_process
	// ����һ����ʱ�������ڴ洢 cloud1 ������
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	*temp_cloud = *cloud_process;  // ���� cloud1 �����ݵ� temp_cloud
	// �� cloud2 �����ݸ�ֵ�� cloud1
	//*cloud_process = *cloud_high;	 //240622myadd
	// �� temp_cloud �����ݸ�ֵ�� cloud2
	*cloud_low = *temp_cloud;

	//ͶӰcloud
	pcl::PointCloud<pcl::PointXYZINormal> cloud1 = *cloud_process;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::copyPointCloud(cloud1, *cloud2);
	for (auto& pt : *cloud2)               //cloud2 ƽ��
	{
		pt.z = 0.0f;
	}

	vector<vector<double>> grid;   //���grid����+�߳�ֵ

	/*�·�����1)������������ 2�������õ�뾶��Χ��*/

	// ��ʼ��kdTree��ƽ��kd����
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	// ����Ҫ�����ĵ���
	std::vector<int>pointIdxRadiusSearch;													//�����ڰ뾶����������
	std::vector<float>pointRadiusSquareDistance;											//�뾶�ڵĽ��ڶ�Ӧ��ƽ������
	kdtree.setInputCloud(cloud2);

	//��άtree
	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree2;
	int point_num = 30;	   //�������ڽ���30����
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
		searchpoint.x = i_x_center; searchpoint2.x = i_x_center;  //ƽ��դ������
		searchpoint.y = i_y_center;	searchpoint2.y = i_y_center;
		searchpoint.z = 0;

		//zֵ�����ڽ����zֵ���棨��ƽ�����ڽ��㣬ȡ�õ��zֵ��
		std::vector<int> neighbors;   //�洢�ٽ�������
		std::vector<float> squaredDistances; //�洢����
		int neighborCount = kdtree.nearestKSearch(searchpoint, point_num, neighbors, squaredDistances);	//����ƽ������ڵ�
		//searchpoint2.z = cloud_process->points[neighbors[0]].z;	 //�����ٽ���point_num�������濼�ǽ������ȡ������ĸ̣߳�ԭ�������ǲ��ɿ���
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
		if (kdtree2.radiusSearch(searchpoint2, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0)//����������Ľ������Ӧ�İ뾶��Ϣ
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
			//slew_weight_all = slew_weight_all + 0.00001; //Ϊ�˷�ֹslew_weight_all=0�������Ļ�slew_weight=-nan
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
			//�뾶��Χ��û�е�
			continue;
		}
	}
	vector<int> grid_first;
	for (int k = 0; k < grid.size(); k++)
	{
		grid_first.push_back(int(grid[k][0]));
	}


	//ground ����άդ��
	pcl::PointCloud<pcl::PointXYZ> ground_cloud;
	double groundx_max = -100000000; double groundy_max = -1000000;  double groundx_min = 100000000; double groundy_min = 100000000;
	for (int i = 0; i < grid.size(); ++i)
	{
		int index = grid[i][0];
		int grid_num_y = (int)(pt1_y - pt0_y) / grid_width;
		if (index < grid_num_y) continue;    //�ѵ�һ��ȥ����
		if (index > grid_num - grid_num_y) continue; //���һ�У�
		int mod_x = index - ((int)(index / grid_num_y)) * grid_num_y;
		if (mod_x == 0 || mod_x == grid_num_y - 1) continue; //ȥ��
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
	//ԭʼ����ͶӰ 231209myadd
	//��ȡͶӰ������ϵ
	//std::string DEMfilepath1 = out_prefix + "ref_DEM/" + "ref_DEM.tif";// "E:/tri_RJH/code/ldem_80s_20m_div2_clip4.tif";//ldem_clip_30m_+5.tif;ldem_clip_30m_+10.tif
	//GDALDataset* poDataset1;   //GDAL���ݼ�
	//GDALAllRegister();  //ע�����е�����
	//poDataset1 = (GDALDataset*)GDALOpen(DEMfilepath1.c_str(), GA_ReadOnly);
	//if (poDataset1 == NULL)
	//{
	//	cout << "fail in open files!!!" << endl;
	//}
	//GDALRasterBand* poBand1 = poDataset1->GetRasterBand(1);//��ȡͼ�񲨶�
	//int nBands1 = poDataset1->GetRasterCount();
	//int nImgSizeX1 = poDataset1->GetRasterXSize();
	//int nImgSizeY1 = poDataset1->GetRasterYSize();
	//int depth1 = GDALGetDataTypeSize(poDataset1->GetRasterBand(1)->GetRasterDataType()) / 32;
	//const char* sProRef1 = poDataset1->GetProjectionRef();//��ȡͶӰ��Ϣ              
	//double trans1[6];//��ȡ����任ϵ��//��ȡ������Ϣ
	//CPLErr aaa1 = poDataset1->GetGeoTransform(trans1);
	//OGRSpatialReference* oSrcSrs1 = new OGRSpatialReference(); //�����ռ�ο�
	//char* c = nullptr;                       //��ʼ��char*����
	//c = const_cast<char*>(sProRef1);
	//oSrcSrs1->importFromWkt(&c);///GetProjection����ͶӰ��Ϣ����wkt��ʽ����������ImportFromWkt������ȡ�ַ�����������������ϵ����׼�桢ͶӰ�������ֱ��ʵȡ�
	//char* pszProjWKT = NULL;
	//char* pszProj4 = NULL;
	//char* pszProjPrettyWKT = NULL;
	//oSrcSrs1->exportToWkt(&pszProjWKT);
	//oSrcSrs1->exportToPrettyWkt(&pszProjPrettyWKT);
	//oSrcSrs1->exportToProj4(&pszProj4);
	//OGRSpatialReference* poTmpSRS1 = oSrcSrs1->CloneGeogCS();//�����������꣨��γ�ȣ�
	//OGRCoordinateTransformation* poCT1 = NULL;
	//poCT1 = OGRCreateCoordinateTransformation(poTmpSRS1, oSrcSrs1);//����ͶӰ����ϵ�͵�������ϵ��ת����ϵ
	//pcl::PointCloud<pcl::PointXYZ> proj_cloud_original;
	//ofstream outfile1, outfile2;
	///*outfile1.open( "D:/ASP_files/XSY_code/test_lavatube/MTP/mosaic_result/oriPCD_trierror.txt");
	//outfile2.open( "D:/ASP_files/XSY_code/test_lavatube/MTP/mosaic_result/oriPCD_Heighterror.txt");*/
	//for (int j = 0; j < ground_cloud.size(); j++)
	//{
	//	Vector3 pos_BLH1 = XYZ_to_BHL(ground_cloud[j].x, ground_cloud[j].y, ground_cloud[j].z);//Ҫ��
	//	double L_lon, L_lat;
	//	L_lon = pos_BLH1.y();
	//	L_lat = pos_BLH1.x();
	//	poCT1->Transform(1, &L_lon, &L_lat);

	//	pcl::PointXYZ pp;
	//	pp.x = L_lon; pp.y = L_lat; pp.z = pos_BLH1.z(); //pp.intensity = ground_cloud[j].normal_x;
	//	proj_cloud_original.push_back(pp);

	//	//�����������ֶε���Ϣ
	//	/*outfile1 << std::setprecision(10) << ground_cloud[j].normal_x << endl;
	//	outfile2 << std::setprecision(10) << ground_cloud[j].normal_y << endl;*/
	//}
	//outfile1.close();
	//outfile2.close();
	////pcl::io::savePCDFileBinary<pcl::PointXYZI>(out_prefix  + "mosaic_result/xyzi_proj_original.pcd", proj_cloud_original);

	//return proj_cloud_original;  //231209myadd
	return ground_cloud;
}

//240620������ҹܲ��ֵĵ����ںϣ��ں�ʱ����ͬһ�������ڰ��ղ�ͬ�߳�ֵ���л���
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

	// ���㰴��������
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

	// ����ÿ������
	for (int i = 0; i < grid_num_x; ++i) {
		for (int j = 0; j < grid_num_y; ++j) {
			auto& cell_points = grid[i][j];
			if (cell_points.empty()) continue;

			// ��ȡ�����ڵ�ĸ߳�����
			std::vector<double> heights;
			for (const auto& point : cell_points) {
				heights.push_back(point.z);
			}
			std::sort(heights.begin(), heights.end());

			double min_height = heights.front();
			double max_height = heights.back();

			// ��0.5�׵ĸ̼߳�����θ�����������м�Ȩ����
			for (double z = min_height; z <= max_height; z += 0.5) {
				pcl::PointXYZINormal searchpoint;
				searchpoint.x = pt0_x + i * grid_width + grid_width / 2.0;
				searchpoint.y = pt0_y + j * grid_width + grid_width / 2.0;
				searchpoint.z = z;

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquareDistance;
				float min_distance = 1e-2; // ��ֹ����ֵ��С
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

//240620������ҹܲ��ֵĵ����ںϣ��ں�ʱ����ͬһ�������ڰ��ղ�ͬ�߳�ֵ���л���
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

	// ���㰴��������
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

	// ����ÿ������
	for (int i = 0; i < grid_num_x; ++i) {
		for (int j = 0; j < grid_num_y; ++j) {
			auto& cell_points = grid[i][j];
			if (cell_points.empty()) continue;

			// ��ȡ�����ڵ�ĸ߳�����
			std::vector<double> heights;
			for (const auto& point : cell_points) {
				heights.push_back(point.z);
			}
			std::sort(heights.begin(), heights.end());

			double min_height = heights.front();
			double max_height = heights.back();

			// ��0.5�׵ĸ̼߳�����θ�����������м�Ȩ����
			for (double z = min_height; z <= max_height; z += 0.5) {
				pcl::PointXYZINormal searchpoint;
				searchpoint.x = pt0_x + i * grid_width + grid_width / 2.0;
				searchpoint.y = pt0_y + j * grid_width + grid_width / 2.0;
				searchpoint.z = z;

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquareDistance;
				float min_distance = 1e-2; // ��ֹ����ֵ��С
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


