/* Copyright (C) 2012 Steven Hickson

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA  */

#pragma once

#ifndef __OPENCV_MICROSOFT_GRABBER__
#define __OPENCV_MICROSOFT_GRABBER__

#include <string>
#include <deque>
#include <iostream>
#include <fstream>
#include <sstream>

#include <assert.h>
#include <windows.h>
#include <vector>
#include <algorithm>
#include <objbase.h>
#include <Kinect.h>

#include <opencv2/opencv.hpp>

using namespace cv;

class KinectGrabber
{
public:
	KinectGrabber(const int instance = 0);
	// const Mode& depth_mode = OpenNI_Default_Mode,
	// const Mode& image_mode = OpenNI_Default_Mode);

	/** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
	~KinectGrabber() throw();

	/** \brief Start the data acquisition. */
	void
	start();

	/** \brief Stop the data acquisition. */
	void
	stop();

	/** \brief Check if the data acquisition is still running. */
	bool
	isRunning() const;

	std::string
	getName() const;

	/** \brief Obtain the number of frames per second (FPS). */
	float
	getFramesPerSecond() const;

	//Kinect Camera Settings
	bool CameraSettingsSupported;

	void GetColor(cv::Mat &color);
	void GetDepth(cv::Mat &depth);
	void GetCloud(/*std::vector<cv::Point3f> &cloud,*/ cv::Mat &Point_Cloud_X, cv::Mat &Point_Cloud_Y, cv::Mat &Point_Cloud_Z);
	void GetRGB(cv::Mat &RGB);

	//Used internally, do not call!
	void ProcessThreadInternal();
	cv::Mat ConvertMat(const RGBQUAD *pBuffer, int nWidth, int nHeight);

protected:
	//int indexc = 0;
	//std::vector<cv::Point2i> l_blob;
	RGBQUAD *TEPdep;
	unsigned char b, g, r;
	bool m_depthStarted, m_videoStarted, m_audioStarted, m_infraredStarted, m_person, m_preregistered;
	// Current Kinect
	IKinectSensor *m_pKinectSensor;

	//IColorFrameReader*      m_pColorFrameReader;
	ICoordinateMapper *m_pCoordinateMapper;
	IMultiSourceFrameReader *m_pMultiSourceFrameReader;

	static const int cColorWidth = 1920;
	static const int cColorHeight = 1080;
	static const int cDepthWidth = 512;
	static const int cDepthHeight = 424;
	cv::Size m_colorSize, m_depthSize;
	RGBQUAD *m_pColorRGBX;
	UINT16 *m_pDepthBuffer;
	//std::vector<UINT16> m_pDepthBuffer;

	std::vector<UINT16> depthBuffer;
	std::vector<RGBQUAD> colorBuffer;
	cv::Mat m_colorImage, m_depthImage, m_RGBImage, colorImg;
	cv::Mat bufferMat;
	std::vector<cv::Point3f> l_pts;
	std::vector<cv::Point3f> l_ptstempp;

	cv::Mat PointCloud_X;
	cv::Mat PointCloud_Y;
	cv::Mat PointCloud_Z;

	cv::Mat PointCloud_X_temp;
	cv::Mat PointCloud_Y_temp;
	cv::Mat PointCloud_Z_temp;

#define COLOR_PIXEL_TYPE CV_8UC4
#define DEPTH_PIXEL_TYPE CV_16UC1

	HANDLE hStopEvent, hKinectThread, hDepthMutex, hColorMutex, hCloudMutex, hRGBMutex;
	bool m_depthUpdated, m_colorUpdated, m_infraredUpdated, m_skeletonUpdated;
	LONGLONG m_rgbTime, m_depthTime, m_infraredTime;

	void Release();
	bool GetCameraSettings();
	void GetNextFrame();
};

#endif //__PCL_IO_MICROSOFT_GRABBER__