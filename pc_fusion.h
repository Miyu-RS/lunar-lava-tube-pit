#pragma once
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//���Ƶ���
#include <pcl/common/io.h>

//#include <opencv2\opencv.hpp>
#include <iostream>
//#include <iomanip> 
//#include <gdal.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <fstream>
#include <sstream>
//#include "pixel_to_vector.h"
#include <vector>
//#include "camera_model.h"
#include "time.h"
#include "omp.h"
#include <thread>
#include <mutex>
//#include <pcl/keypoints/uniform_sampling.h>



//#include <pcl/features/normal_3d.h>

#include "common_function.h"

vector<string> ReadLines_string2(string filename)   //ֱ�Ӷ������ļ�
{

	ifstream infile;
	infile.open(filename.data());   //���ļ����������ļ��������� 
	assert(infile.is_open());   //��ʧ��,�����������Ϣ,����ֹ�������� 

	string c;
	vector<string> v;
	while (!infile.eof())
	{
		infile >> c;
		v.push_back(c);
	}
	infile.close();

	stringstream input;

	return v;

}


void pc_fusion_main1_mhp(string out_prefix)
{
	//string out_prefix = "F:/44images/1113test/";

	//��ȡ�б�
	vector<string> pairs_content;

	//pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_data/0627test3pair.txt");
	//pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_lavatube/MTP/16_6_8_12pairs.txt");
	pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_lavatube/MHP/240609_4pair.txt");	  // /MHP/new6pair.txt,  MTP/16_6_8_12_10pairs.txt 
	vector<int> pairs_index;//�������
	vector<vector<string>> image_names;

	for (int i = 0; i < pairs_content.size() / 3; i++)
	{
		pairs_index.push_back(atoi(pairs_content[i * 3].c_str()));
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr mergecloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud(new pcl::PointCloud<pcl::PointXYZINormal>);  //231209myadd
	pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/" + to_string(pairs_index[0]) + "/" + to_string(pairs_index[0]) + "_xyzfilter_proj2_regis.pcd", *mergecloud); // �ܶ��� _xyzfilter_proj2_regis_clip.pcd�Լ�������_xyzfilter_proj2_regis.pcd
	// Ϊÿ��������������ĸ��������
	for (size_t i = 0; i < mergecloud->size(); ++i) {
		mergecloud->points[i].normal_z = 0.559; // ����Ϊ 16 ��ʾ�������16
	}
	for (int i = 1; i < pairs_content.size() / 3; i++)
	{
		//pcl::PointCloud<pcl::PointXYZ>::Ptr mergecloud2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud2(new pcl::PointCloud<pcl::PointXYZINormal>);  //231209myadd
		pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/" + to_string(pairs_index[i]) + "/" + to_string(pairs_index[i]) + "_xyzfilter_proj2_regis.pcd", *mergecloud2);
		if (i == 1) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.199; // ����Ϊ 6 ��ʾ�������6��������Ϊ0������ֻʣ���������6�ĵ�ʱ�佻���Ȩ���ܺ���0����ȷ������0�õ��ľ���nan
			}
		}
		if (i == 2) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.573; // ����Ϊ 8 ��ʾ�������8
			}
		}
		if (i == 3) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.384; // ����Ϊ 12 ��ʾ�������12
			}
		}
		if (i == 4) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.573; // ����Ϊ 12 ��ʾ�������12
			}
		}
		if (i == 5) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.384; // ����Ϊ 12 ��ʾ�������12
			}
		}
		*mergecloud = *mergecloud + *mergecloud2;
	}
	//pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "mosaic_result/pitmergecloud.pcd", *mergecloud);
	//����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_tisu = junyun_singlePCD_XYZ_2(mergecloud, 8);  //�ռ�������8�׵���˼
	//pcl::io::savePCDFileBinary(out_prefix + "mosaic_result/original_tisu.pcd", *cloud_merge_tisu);	//�����ںϷ�ʽ���غ�����ļ�Ȩ

	//ƽ��
	// �Ե����ز���  
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZI>); // ��������ڵ�KD-tree
	//pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls_filter;  // ������С����ʵ�ֵĶ���
	//mls_filter.setInputCloud(cloud_merge_tisu);   // ������������
	//mls_filter.setComputeNormals(false);    // ��������С���˼������Ƿ���Ҫ�洢����ķ���
	//mls_filter.setPolynomialOrder(2);   // 2�׶���ʽ���
	//mls_filter.setSearchMethod(treeSampling);   //����KD-tree��Ϊ��������
	//mls_filter.setSearchRadius(50);   //����������ϵ�K���ڰ뾶
	//mls_filter.process(*cloud_smoothed);    //���

	//����ز������
	//pcl::io::savePCDFileBinary(out_prefix + "n_align/block6/"+"remain_smoothedPC_radius50.pcd", *cloud_smoothed);
	//pcl::io::savePCDFileBinary<pcl::PointXYZI>(out_prefix + "n_align/block6/16_24new_54_tisu8.pcd", *cloud_merge_tisu);

	//�ϲ���������,ԭ�������ݼ�¼
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/testtubeclip1580_2_2_0.1-0.5-0.4.pcd", *mergecloud_low);
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/testplane_0.5_1_0.1-0.5-0.4.pcd", *mergecloud_high);
	//*mergecloud_low = *mergecloud_low + *mergecloud_high;
	//pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/pitmergecloud.pcd", *mergecloud_low);
	//�µ����ݼ�¼ 240622
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/13/newlow/lowpart1570_newtest5_2_1.pcd", *mergecloud_low);
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/13/high_clip_mosaic_lowgigh_mean33_PC.pcd", *mergecloud_high);
	//*mergecloud_low = *mergecloud_low + *mergecloud_high;
	//pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "PC/13/newlow/MHHpit1570_21mergecloud.pcd", *mergecloud_low);

	//�ռ������Ȩ
	pcl::PointCloud<pcl::PointXYZ> ground;
	ground = weighted_rasterize_xyzinormal2(out_prefix, mergecloud, 0.5, 1); //���ҹܶ����ϲ��ֽ����ں�
	//ground = weighted_rasterize_xyzinormal3_low(out_prefix, mergecloud, 0.5, 1);	//���ҹܶ����ֽ����ں�
	pcl::io::savePCDFileBinary<pcl::PointXYZ>(out_prefix + "mosaic_result/240609test/allpart_newtest4_0.5_1.pcd", ground);


}

void pc_fusion_main1_mtp(string out_prefix)
{
	//string out_prefix = "F:/44images/1113test/";

	//��ȡ�б�
	vector<string> pairs_content;

	//pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_data/0627test3pair.txt");
	pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_lavatube/MTP/16_6_8_12_10pairs.txt");	 //16_6_8_12_10pairs.txt
	//pairs_content = ReadLines_string2("D:/ASP_files/XSY_code/test_lavatube/MHP/240609_4pair.txt");	  // /MHP/new6pair.txt,  MTP/16_6_8_12_10pairs.txt 
	vector<int> pairs_index;//�������
	vector<vector<string>> image_names;

	for (int i = 0; i < pairs_content.size() / 3; i++)
	{
		pairs_index.push_back(atoi(pairs_content[i * 3].c_str()));
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr mergecloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud(new pcl::PointCloud<pcl::PointXYZINormal>);  //231209myadd
	pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/" + to_string(pairs_index[0]) + "/" + to_string(pairs_index[0]) + "_xyzfilter_proj2_regis.pcd", *mergecloud); //  _xyzi_filter_SOR_50_100_pit.pcd
	// Ϊÿ��������������ĸ��������
	for (size_t i = 0; i < mergecloud->size(); ++i) {
		mergecloud->points[i].normal_z = 1; // ����Ϊ 16 ��ʾ�������16
	}
	for (int i = 1; i < pairs_content.size() / 3; i++)
	{
		//pcl::PointCloud<pcl::PointXYZ>::Ptr mergecloud2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud2(new pcl::PointCloud<pcl::PointXYZINormal>);  //231209myadd
		pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/" + to_string(pairs_index[i]) + "/" + to_string(pairs_index[i]) + "_xyzfilter_proj2_regis.pcd", *mergecloud2);
		if (i == 1) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.107; // ����Ϊ 6 ��ʾ�������6��������Ϊ0������ֻʣ���������6�ĵ�ʱ�佻���Ȩ���ܺ���0����ȷ������0�õ��ľ���nan
			}
		}
		if (i == 2) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.650; // ����Ϊ 8 ��ʾ�������8
			}
		}
		if (i == 3) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.883; // ����Ϊ 12 ��ʾ�������12
			}
		}
		if (i == 4) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.487; // ����Ϊ 12 ��ʾ�������12
			}
		}
		if (i == 5) {
			for (size_t i = 0; i < mergecloud2->size(); ++i) {
				mergecloud2->points[i].normal_z = 0.384; // ����Ϊ 12 ��ʾ�������12
			}
		}
		*mergecloud = *mergecloud + *mergecloud2;
	}
	//pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "mosaic_result/pitmergecloud.pcd", *mergecloud);
	//����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_tisu = junyun_singlePCD_XYZ_2(mergecloud, 8);  //�ռ�������8�׵���˼
	//pcl::io::savePCDFileBinary(out_prefix + "mosaic_result/original_tisu.pcd", *cloud_merge_tisu);	//�����ںϷ�ʽ���غ�����ļ�Ȩ

	//ƽ��
	// �Ե����ز���  
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZI>); // ��������ڵ�KD-tree
	//pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls_filter;  // ������С����ʵ�ֵĶ���
	//mls_filter.setInputCloud(cloud_merge_tisu);   // ������������
	//mls_filter.setComputeNormals(false);    // ��������С���˼������Ƿ���Ҫ�洢����ķ���
	//mls_filter.setPolynomialOrder(2);   // 2�׶���ʽ���
	//mls_filter.setSearchMethod(treeSampling);   //����KD-tree��Ϊ��������
	//mls_filter.setSearchRadius(50);   //����������ϵ�K���ڰ뾶
	//mls_filter.process(*cloud_smoothed);    //���

	//����ز������
	//pcl::io::savePCDFileBinary(out_prefix + "n_align/block6/"+"remain_smoothedPC_radius50.pcd", *cloud_smoothed);
	//pcl::io::savePCDFileBinary<pcl::PointXYZI>(out_prefix + "n_align/block6/16_24new_54_tisu8.pcd", *cloud_merge_tisu);

	//�ϲ���������
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/testtubeclip1580_2_2_0.1-0.5-0.4.pcd", *mergecloud_low);
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	//pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/testplane_0.5_1_0.1-0.5-0.4.pcd", *mergecloud_high);
	//*mergecloud_low = *mergecloud_low + *mergecloud_high;
	//pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "mosaic_result/0107test/pitmergecloud.pcd", *mergecloud_low);
	 //240622�ϲ���������
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_low(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/21/newlow/newlowpart_test5_2_1.pcd", *mergecloud_low);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergecloud_high(new pcl::PointCloud<pcl::PointXYZINormal>);  //240108myadd
	pcl::io::loadPCDFile<pcl::PointXYZINormal>(out_prefix + "PC/21/high_clip_original_0.5_3_new_nofill_mean33.pcd", *mergecloud_high);
	*mergecloud_low = *mergecloud_low + *mergecloud_high;
	pcl::io::savePCDFileBinary<pcl::PointXYZINormal>(out_prefix + "PC/21/newlow/newMTPpitmergecloud.pcd", *mergecloud_low);
	//�ռ������Ȩ
	pcl::PointCloud<pcl::PointXYZ> ground;
	//ground = weighted_rasterize_xyzinormal2(out_prefix, mergecloud, 0.5, 1); //���ҹܶ����ϲ��ֽ����ں�
	ground = weighted_rasterize_xyzinormal3_low(out_prefix, mergecloud, 2, 1);	//�µ����ҹܶ����ֽ����ںϣ�ͬһ���������ָ߳�ֵ���м�Ȩ
	pcl::io::savePCDFileBinary<pcl::PointXYZ>(out_prefix + "mosaic_result/240609test/newlowpart_test5_2_1.pcd", ground);


}

