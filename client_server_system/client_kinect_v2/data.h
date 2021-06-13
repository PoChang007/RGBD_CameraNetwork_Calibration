// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <windows.h>

using namespace std;

#define CLIENTID 1
int save_index = 0;

/*Create the local file*/
char path_file[200];
ofstream out_txt;

int save_image = 1;
int track_signal = 0;
int total_num;
#define NUMBER 3000
cv::Mat path_mat;
cv::Mat sphere_center_total[NUMBER];
std::vector<float> radius_total;
std::vector<std::vector<cv::Point3f>> cloud_l_imgs /*[NUMBER]*/;

std::vector<cv::Point2i> r_blob;
std::vector<cv::Point2i> l_blob;
std::vector<cv::Point3f> cloudtemp;
std::vector<cv::Point3f> b;
cv::Mat mean_cloud;
std::vector<cv::Point3f> l_pts, r_pts, final_pts;

cv::Mat rgb_l_imgs;
cv::Mat PointCloudX;
cv::Mat PointCloudY;
cv::Mat PointCloudZ;

vector<int> arr_idx; //the vector stores all the index values of point cloud