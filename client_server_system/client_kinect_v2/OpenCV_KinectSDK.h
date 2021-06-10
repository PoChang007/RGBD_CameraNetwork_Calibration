// Copyright 2017 University of Kentucky 
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include <iostream>
#include "Microsoft_grabber2.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <deque>

typedef unsigned short XnUInt16;
typedef unsigned char XnUInt8;
typedef unsigned long long int uint64_t;
typedef signed long long int int64_t;

//#define NUMBER 300;


class SaveKinectData {
private:
	//std::deque<cv::Mat*> imgs;
	//std::deque<cv::Mat*> depths;
	//std::deque<cv::Mat*> RGBs;
	//std::deque<std::vector<cv::Point3f>*> clouds;
	HANDLE hCaptureThread, hSaveThread, hDoneSaving;
	//KinectGrabber *kc;
	
	std::string direc;

protected:
	//cv::Mat rgb_l_imgs[150];
	//cv::Mat depth_l_imgs[150];

	
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	//int indexc = 0;
	//std::vector<cv::Point2i> l_blob;
	cv::Mat l_thr;
	//cv::Mat RGBImg = cv::Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC3);
	int pt_rand = 50;
	//std::vector<cv::Point3f> l_pts, r_pts /*final_pts*/;
	
	int ransac_max = 5000;
	/*store the 3D point cloud in matrix form (the same data as l_pts*/
	cv::Mat pts_mat;
	cv::Mat sphere_center;
	float sphere_radius;
	float estimated_radius = 203.0f;/*203.0f;*/
	float radius_min_range = (estimated_radius - 40) * (estimated_radius - 40);  //40
	float radius_max_range = (estimated_radius + 40) * (estimated_radius + 40); //40
	//FILE *fp;
	cv::Mat send_data;
	char *m_depthStream;
	unsigned long m_depthSize;
	char *m_jpegStream;
	unsigned long m_jpegSize;

	char *m_pathStream;
	unsigned long m_pathSize;

	char *m_pointcloudX_Stream;
	char *m_pointcloudY_Stream;
	char *m_pointcloudZ_Stream;
	unsigned long m_pointcloudX_depthSize;
	unsigned long m_pointcloudY_depthSize;
	unsigned long m_pointcloudZ_depthSize;

	std::vector<unsigned char> m_jpegBuf;
	std::vector<int64_t> dp;

	cv::Mat m_img;
	cv::Mat m_RGB;
	cv::Mat m_depth;
	cv::Mat m_cloud;
	std::vector<cv::Point3f> cloud;

	cv::Mat Point_Cloud_X;
	cv::Mat Point_Cloud_Y;
	cv::Mat Point_Cloud_Z;

public:
	int FrameIndex = 0;

	bool kinect_init;
	int count;
	bool quit;
	//std::vector<cv::Point3f> final_pts;
	int csock;
	KinectGrabber *kc;
	//std::deque<cv::Mat*> imgs;
	//std::deque<cv::Mat*> depths;
	SaveKinectData(std::string _directory) : quit(false), kinect_init(false), count(0), direc(_directory) { }
	SaveKinectData(KinectGrabber *_kc, std::string _directory) : quit(false), kinect_init(true), kc(_kc), count(0), direc(_directory) { };
	
	//void StartSavingInternal();
	void StartCapturingInternal();
	void Start();
	void Stop();
	void regionDetection(cv::Mat &left, int i);
	void findSphereBlobs(cv::Mat &Stored_Image, int index);
	void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);
	void compute3Dpoints(cv::Mat &PointCloudX, cv::Mat &PointCloudY, cv::Mat &PointCloudZ, int index);
	void sphereFitting(int index);
	void sphereRefine(int index);
	void computeSpehereCenter(std::vector<cv::Point3f> &cur_pts);

	void closeTimeFile();
	void openTimeFile();
	void setSaveSignal(int x);
	unsigned long getJPEGSize() const { return m_jpegSize; };
	unsigned long getDepthStreamSize() const { return m_depthSize; };
	unsigned long getPointCLoudX_StreamSize() const { return m_pointcloudX_depthSize; };
	unsigned long getPointCLoudY_StreamSize() const { return m_pointcloudY_depthSize; };
	unsigned long getPointCLoudZ_StreamSize() const { return m_pointcloudZ_depthSize; };

	// Get internal buffer that stores the JPEG bitstream
	char *getJPEG() { return m_jpegStream; };
	// Get internal buffer that stores the depth bitstream
	char *getDepthStream() { return m_depthStream; };
	// Get internal buffer that stores the point cloud bitstream
	std::vector<int64> getcloud() const { return dp; };
	char *getPointCLoudX_Stream() { return m_pointcloudX_Stream; };
	char *getPointCLoudY_Stream() { return m_pointcloudY_Stream; };
	char *getPointCLoudZ_Stream() { return m_pointcloudZ_Stream; };

	unsigned long getPathStreamSize();
	char *getPathStream() { return m_pathStream; };
    
	void sphereTracking();
	void reallocateTracking();
	void resetTracking();
};