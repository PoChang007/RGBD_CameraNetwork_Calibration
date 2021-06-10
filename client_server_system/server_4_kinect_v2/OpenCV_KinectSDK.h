#include <iostream>
#include "Microsoft_grabber2.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <deque>

class SaveKinectData {
private:
	std::deque<cv::Mat*> imgs;
	std::deque<cv::Mat*> depths;
	//std::deque<cv::Mat> colors_k;
	//std::deque<cv::Mat> depths_k;
	HANDLE hCaptureThread, hSaveThread, hDoneSaving;
	KinectGrabber *kc;
	bool kinect_init;
	bool quit;
	int count;
	std::string direc;
	HANDLE						kinectHandle_1;
	HANDLE						kinectHandle_2;
	HANDLE						kinectHandle_3;
public:
	//std::deque<cv::Mat*> imgs;
	//std::deque<cv::Mat*> depths;
	SaveKinectData(std::string _directory) : quit(false), kinect_init(false), count(0), direc(_directory) { }
	SaveKinectData(KinectGrabber *_kc, std::string _directory) : quit(false), kinect_init(true), kc(_kc), count(0), direc(_directory) { };
	void StartSavingInternal();
	void StartCapturingInternal();
	void Start();
	void Stop();

	void kinectThread_1();
	static DWORD WINAPI kinectStaticThread_1(PVOID lpParam);


	void kinectThread_2();
	static DWORD WINAPI kinectStaticThread_2(PVOID lpParam);

	void kinectThread_3();
	static DWORD WINAPI kinectStaticThread_3(PVOID lpParam);
};