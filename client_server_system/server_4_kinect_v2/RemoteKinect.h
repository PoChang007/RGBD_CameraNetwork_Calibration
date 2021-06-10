// Copyright 2017 University of Kentucky 
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#ifndef RemoteKinect_H_
#define RemoteKinect_H_

#include <vector>

//#include <winsock2.h>
#include <windows.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <conio.h>
#include <cv.h>
#include <highgui.h>  
#include <cxcore.h>

#include "DepthCodec.h"
typedef unsigned long long int uint64_t;
typedef signed long long int int64_t;
//////////////////////////////////////////////////////////
// Class RemoteKinect
//
// This classes allows a client to connect to a remote
// machine running a remoteKinect server and retrieve
// RGB and depth images.
//
// The following is a sample code segment on how to use
// this class:
//
// // RemoteKinect requires the depth compression codec
// xn::Codec depthCodec;
// // ... need initialization ...
// 
// // Say you have 3 kinects; initialize them with the IP address and port of 
// // of the machines with kinects, as well as the depth compressor
// static const int numKinectMachines = 3;
// RemoteKinect kinectMachines[numKinectMachines] = {
//   RemoteKinect("123.45.67.87",3000),
//   RemoteKinect("123.45.67.88",3000),
//   RemoteKinect("123.45.67.89",3000)};
//
// // Outer loop over every frame
// int frameNo = 0;
// int status;
// while (1) {
//   // startTransfer is a non-blocking call that initiates the transfer
//   // in a separate thread
//   if (frameNo==0) 
//      if ((status=kinectMachines[0].startTransfer()) != 0)
//        fprintf(stderr,"Problem with Kinect 0\n");
//
//   
//
//   // Inner loop over every kinect
//   cv::Mat RGBframe, Depthframe;
//   for (int i=0; i<numKinectMachines; i++) {
//
//     // getData is a blocking call to get all the frames. Provided that
//     // the startTransfer was called in advance, the data should be ready.
//     if ((status=kinectMachines[i].getData(RGBframe, Depthframe)) != 0)
//       fprintf(stderr,"Problem with Kinect %d\n",i);
//
//     // start the transfer from the next machine while working on the
//     // the data from the current one
//     kinectMachines[(i+1)%numKinectMachines].startTransfer();
//     
//     // Problem with this kinect, move on to the next one.
//     if (status != 0) continue;
//     // .... Process RGBframe and Depthframe ...
//   }
//
//   frameNo++;
// }

class RemoteKinect {

public:

	// constructor 
	// The constructor initiates the IP connection. If it is successful, then 
	// it will allocate necessary memory space for network transmission
	RemoteKinect(char *ipAddr, int port = 3000);
	
	// destructor
	// Cleanup : kill thread, send 9999 (unsigned long) to server to kill process
	// close socket
	virtual ~RemoteKinect();

	// startTransfer
	// It is a non-blocking call that starts a thread to grab the next 
	// color and depth images from the remote kinect. It returns 0 if okay
	// or negative error codes based on the types of problems. 
	int startTransfer();

	// getData
	// It is a blocking call that retrieves the actual color and depth images.
	// It returns 0 if okay or negative error codes based on the types of 
	// problems.
	int getData(cv::Mat &RGBframe, /*cv::Mat &depthframe,*/ cv::Mat &pointcloudX_frame, cv::Mat &pointcloudY_frame, cv::Mat &pointcloudZ_frame/*, std::vector<int64_t> &cloudFrame*/);

	void passEyeCenter(cv::Mat toServer); //used to send server eye center to different clients, each client call this function once
	void recieveEyeCenter(char* data); //used to recieve detected eye center from each client
	cv::Mat getRecievedEyeCenter();  //used in the main function, it will be called by each client
	void setSaveSignal();
	void setCalibConnectionSignal(); //control calibration capturing procedure
	void setPathTrackingSignal(int x); //control calibration tracking procedure
	int getPathTrackingSignal();

	short int calib_conn[3];

	
	cv::Mat _pathMat;

protected:
	int _status;      // Current status of the connection
	int _hsock;       // Socket for communication
	HANDLE _thread;   // thread created by getData
	bool _threadRunning; // True if there is a thread running
	DepthCodec _depthCodec; // For decompressing the depth bitstream
	DepthCodec _pointcloudX_Codec; // For decompressing the Point Cloud X bitstream
	DepthCodec _pointcloudY_Codec; // For decompressing the Point Cloud Y bitstream
	DepthCodec _pointcloudZ_Codec; // For decompressing the Point Cloud Z bitstream

	// Buffer 
	cv::Mat _RGBFrame;
	cv::Mat _depthFrame;
	cv::Mat _pointcloudX_Frame;
	cv::Mat _pointcloudY_Frame;
	cv::Mat _pointcloudZ_Frame;

	cv::Mat _dataBuffer;
	cv::Mat _pointcloudX_dataBuffer;
	cv::Mat _pointcloudY_dataBuffer;
	cv::Mat _pointcloudZ_dataBuffer;
	std::vector<int64_t> _cloudBuffer;


	/*For eye center use*/
	cv::Mat local_eye_center;
	cv::Mat recieved_eye_center;


	/*For instruction to clients use*/
	unsigned long save_img_signal;


	/*bitstream for eye_center*/
	char *centerStream; //for sending
	char* eye_center_buf; //for recieving
	unsigned long centerSize;




	// getRemoteData
	// This is the real work behind getData following this protocol
	// (1) Send 1 unsigned long (value = 1) to get JPEG bitstream
	// (2) Receive 1 unsigned long on the size of the JPEG bitstream
	// (3) Create a large enough buffer
	// (4) Download the jpeg bitstream
	// (5) Uncompress JPEG bitstream to create image data
	// (6) Use dimension to initialize depth data
	// (7) Send 1 unsigned long (value = 2) to get depth bitstream
	// (8) Receive 1 unsigned long on the size of the depth bitstream
	// (9) Create a large enough buffer
	// (10) Download the depth bitstream
	// (11) Uncompress depth data
	static DWORD WINAPI getRemoteData(void *basept);
};

#endif RemoteKinect_H_