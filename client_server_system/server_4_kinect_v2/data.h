// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>

using namespace std;

#define WIDTH 960
#define HEIGHT 540

#define WIDTH1 1024
#define HEIGHT1 768

#define GLWidth 800
#define GLHeight 600

// 4 cameras are used here. The number of cameras can be adjusted
#define CLIENT1
#define CLIENT2
#define CLIENT3
#define CLIENT4
#define Client_Total 4

string file_name("img_");
string GLResultPrefix("Output_");
string temp;
std::vector<cv::Point2i> l_blob;
cv::Mat l_thr;

cv::Mat RGBImage[Client_Total]; /*For image transfer*/
cv::Mat PointCloudX[Client_Total];
cv::Mat PointCloudY[Client_Total];
cv::Mat PointCloudZ[Client_Total];
cv::Mat rgbImage[Client_Total]; /*For OpenGL rendering*/
cv::Mat xyzImage[Client_Total];

int path_obtain_signal = 0;

cv::Mat GLResults(GLHeight, GLWidth, CV_8UC4);

int idxcounter = 0;
int offset = 1;

// Set up ip address for each client. Each client connets to a RGB-D camera
#ifdef CLIENT1
RemoteKinect kinect_1 = RemoteKinect("172.31.40.146"); //kinect #1
#endif

// We use the first camera as the reference camera in the camera network

#ifdef CLIENT2
RemoteKinect kinect_2 = RemoteKinect("172.31.40.128"); //kinect #2

// Give R and T a random default value
float R21[3][3] = {{-0.998964, -0.0448149, 0.00790334},
				   {-0.0294494, 0.504248, -0.863056},
				   {0.0346926, -0.862395, -0.505046}};
float T21[] = {616.841,
			   1293.88,
			   3122.18};

#endif

#ifdef CLIENT3
RemoteKinect kinect_3 = RemoteKinect("172.31.40.18"); //kinect #3

float R31[3][3] = {{-0.0995075, -0.68663, -0.720165},
				   {0.599517, 0.536268, -0.594134},
				   {0.794152, -0.490872, 0.358284}};
float T31[] = {1554.72, 1009.56, 1238.67};

#endif

#ifdef CLIENT4
RemoteKinect kinect_4 = RemoteKinect("172.31.40.244"); //kinect #4

float R41[3][3] = {{0.182828, -0.442151, -0.87811},
				   {0.56048, 0.780683, -0.276399},
				   {0.807735, -0.441629, 0.390548}};
float T41[] = {1126.64, 520.916, 945.361};

#endif

cv::Mat depth_pixels_scalar;	  //stores the camera coordinate after color_inverse * depth_pixels
cv::Mat depth_pixels_scalar_tran; //The transpose matrix of depth_pixels_scalar

cv::Mat color_inverse(3, 3, CV_32F);
cv::Mat depth_pixels(3, WIDTH *HEIGHT, CV_32F);

/*For extrinsic calibration use*/

cv::Mat path_mat[Client_Total]; //the path list received from each client

struct coordinate
{
	int idx;
	float x;
	float y;
	float z;
};

struct path
{
	vector<coordinate> coords;
};

path clients[Client_Total]; //store the noisy path including those z = -1000
path proxy_clients[2];		// this is for pair-wise use that make sure the each point in the client cooresponds to
							// the same point in the other proxy_client
path valid_clients[2];		// the difference between this one and clients is that it only keep those points whose z values are positive
