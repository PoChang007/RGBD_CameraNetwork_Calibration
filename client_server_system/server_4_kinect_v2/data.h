// Copyright 2017 University of Kentucky 
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include <stdio.h>
#include <string>
#include<iostream>
#include <sstream>

using namespace std;

#define WIDTH 960
#define HEIGHT 540

#define WIDTH1 1024
#define HEIGHT1 768

#define GLWidth 800
#define GLHeight 600

#define CLIENT1
#define CLIENT2
#define CLIENT3
#define CLIENT4

#define Client_Total 4

string file_name("img_");
string GLResultPrefix("Output_");
string temp;
//cv::Mat RGBImage_1;
//cv::Mat depthImage_1;
//cv::Mat RGBImage_2;
//cv::Mat depthImage_2;
cv::Mat virimg(HEIGHT1, WIDTH1, CV_8UC3);
cv::Mat big_virimg(HEIGHT1, WIDTH1, CV_8UC3);
std::vector<cv::Point2i> l_blob;
cv::Mat l_thr;

cv::Mat RGBImage[Client_Total];  /*For image transfer*/
cv::Mat depthImage[Client_Total];
cv::Mat PointCloudX[Client_Total];
cv::Mat PointCloudY[Client_Total];
cv::Mat PointCloudZ[Client_Total];
cv::Mat rgbImage[Client_Total];  /*For OpenGL rendering*/
cv::Mat xyzImage[Client_Total];

std::vector<int64_t> kinect_1_cloud;
std::vector<int64_t> kinect_2_cloud;
std::vector<int64_t> kinect_3_cloud;
std::vector<int64_t> kinect_4_cloud;

int path_obtain_signal = 0;

cv::Mat GLResults(GLHeight, GLWidth, CV_8UC4);

cv::Mat z_buffer; //a 640*480 matrix that stores the z value for each pixel (enable depth)
cv::Mat z_buffer_copy; //for interpolation use
cv::Mat z_bufferFinal; //for 1024 * 768 Use


cv::Mat final_eye_center_float(3, 1, CV_32F);//used for computation
cv::Mat final_eye_center(3, 1, CV_16UC1); //used for sending out to each kinect
cv::Mat temp_eye(3, 1, CV_16UC1);

int swith[2] = {1, 0};
int idxcounter = 0;
int img_index = 0;
string server_file_name("Server_");
string temp_file;

int interpolation_sign = 0;
int recieve_valid_eye_sign = 0; //used to tell whether the eye center is detected by at least one of the client

int offset = 1;

#ifdef CLIENT1
	RemoteKinect kinect_1 = RemoteKinect("172.31.40.146");  //kinect 1  18 128 137 244
#endif

	//float R21[3][3] = {{0.137774, -0.511457, 0.848192},
	//{0.564712, 0.744096, 0.35696},
	//{-0.813707, 0.429805, 0.391343}};
	//float T21[] = {-1824.11, -509.204, 851.235};
	//	float R31[3][3] = {{-0.219571, 0.348955, -0.911054}, 
	//{-0.500928, 0.761019, 0.412216},
	//{0.837174, 0.546883, 0.00770371}};
	//float T31[] = {2038.68, -1189.9, 2315.01};

#ifdef CLIENT2
	RemoteKinect kinect_2 = RemoteKinect("172.31.40.128"); //kinect 2  129 

	float R21[3][3] = { { -0.998964 ,- 0.0448149 ,0.00790334 },
	{ -0.0294494, 0.504248, - 0.863056 },
	{ 0.0346926, - 0.862395 ,- 0.505046 } };
	float T21[] = { 616.841,
		1293.88,
		3122.18
	};

#endif

#ifdef CLIENT3
	RemoteKinect kinect_3 = RemoteKinect("172.31.40.18"); //kinect 3 183

	float R31[3][3] = { { -0.0995075, - 0.68663, - 0.720165 },
	{ 0.599517, 0.536268, - 0.594134 },
	{ 0.794152, - 0.490872, 0.358284 } };
	float T31[] = { 1554.72,		1009.56,		1238.67 };

#endif

#ifdef CLIENT4
	RemoteKinect kinect_4 = RemoteKinect("172.31.40.244"); //kinect 4 183


	float R41[3][3] = { { 0.182828, - 0.442151, - 0.87811 },
	{ 0.56048, 0.780683, - 0.276399 },
	{ 0.807735, - 0.441629, 0.390548 } };
	float T41[] = { 1126.64,		520.916,		945.361	};


#endif




cv::Mat depth_pixels_scalar; //stores the camera coordinate after color_inverse * depth_pixels
cv::Mat depth_pixels_scalar_tran; //The transpose matrix of depth_pixels_scalar

cv::Mat color_inverse(3, 3, CV_32F);
cv::Mat depth_pixels(3, WIDTH * HEIGHT, CV_32F);




/*For extrinsic calibration use*/

cv::Mat path_mat[Client_Total]; //the path list recieved from each client

struct coordinate{
	int idx;
	float x;
	float y;
	float z;
};

struct path{
	vector<coordinate> coords;
};


path clients[Client_Total];   //store the noisy path including those z = -1000
path proxy_clients[2]; // this is for pair-wise use that make sure the each point in the client cooresponds to
                       // the same point in the other proxy_client
path valid_clients[2]; // the difference between this one and clients is that it only keep those points whose z values are positive

