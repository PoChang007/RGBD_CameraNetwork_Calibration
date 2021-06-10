
/*
Copyright (C) 2014 Steven Hickson

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
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "Microsoft_grabber2.h"

using namespace std;
using namespace cv;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

DWORD ProcessThread(LPVOID pParam) {
	KinectGrabber *p = (KinectGrabber*) pParam;
	p->ProcessThreadInternal();
	return 0;
}

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }

KinectGrabber::KinectGrabber(const int instance) {
	HRESULT hr;
	int num = 0;
	m_person = m_depthStarted = m_videoStarted = m_audioStarted = m_infraredStarted = false;
	hStopEvent = NULL;
	hKinectThread = NULL;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) {
		throw exception("Error could not get default kinect sensor");
	}

	if (m_pKinectSensor) {
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
			if (SUCCEEDED(hr))
			{
				m_videoStarted = m_depthStarted = true;
			} else
				throw exception("Failed to Open Kinect Multisource Stream");
		}
	}

	if (!m_pKinectSensor || FAILED(hr)) {
		throw exception("No ready Kinect found");
	}
	m_colorSize = Size(cColorWidth, cColorHeight);
	m_depthSize = Size(cDepthWidth, cDepthHeight);
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	//m_pDepthBuffer = new UINT16[cDepthWidth * cDepthHeight];
}

void KinectGrabber::start() {
	hDepthMutex = CreateMutex(NULL,false,NULL);
	if(hDepthMutex == NULL)
		throw exception("Could not create depth mutex");
	hColorMutex = CreateMutex(NULL,false,NULL);
	if(hColorMutex == NULL)
		throw exception("Could not create color mutex");
	hStopEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
	hKinectThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread, this, 0, NULL );

}

void KinectGrabber::stop() 
{
	exit(-1);
	CloseHandle(hStopEvent);
}


KinectGrabber::~KinectGrabber() {
	Release();
}

bool KinectGrabber::GetCameraSettings() {
	return false;
}

void KinectGrabber::ProcessThreadInternal() {
	bool quit = false;
	while(!quit) {
		// Wait for any of the events to be signalled
		if(WaitForSingleObject(hStopEvent,1) == WAIT_OBJECT_0)
			quit = true;
		else {
			//Get the newest frame info
			GetNextFrame();
		}
	}
}



void KinectGrabber::Release() {
	try {
		//clean up stuff here
		stop();
		if(m_pKinectSensor) {
			//Shutdown NUI and Close handles
			if (m_pMultiSourceFrameReader)
				SafeRelease(m_pMultiSourceFrameReader);
			if(m_pCoordinateMapper)
				SafeRelease(m_pCoordinateMapper);
			// close the Kinect Sensor
			if (m_pKinectSensor)
				m_pKinectSensor->Close();

			SafeRelease(m_pKinectSensor);
		}
	} catch(...) {
		//destructor never throws
	}
}

string KinectGrabber::getName () const {
	return std::string ("Kinect2Grabber");
}

float KinectGrabber::getFramesPerSecond () const {
	return 30.0f;
}

void KinectGrabber::GetNextFrame() {
	if (!m_pMultiSourceFrameReader)
	{
		return;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	//IBodyIndexFrame* pBodyIndexFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	//if (SUCCEEDED(hr))
	//{
	//	IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

	//	hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
	//	}

	//	SafeRelease(pBodyIndexFrameReference);
	//}

	//if (SUCCEEDED(hr))
	if (pColorFrame && pDepthFrame)
	{
		INT64 nDepthTime = 0;
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		//IFrameDescription* pBodyIndexFrameDescription = NULL;
		//int nBodyIndexWidth = 0;
		//int nBodyIndexHeight = 0;
		//UINT nBodyIndexBufferSize = 0;
		//BYTE *pBodyIndexBuffer = NULL;

		// get depth frame data

		hr = pDepthFrame->get_RelativeTime(&nDepthTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			//m_pDepthBuffer = new UINT16[cDepthWidth * cDepthHeight];
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &m_pDepthBuffer);
			//pDepthFrame->CopyFrameDataToArray(nDepthBufferSize,m_pDepthBuffer);
			WaitForSingleObject(hDepthMutex,INFINITE);
			m_depthImage.release();
			Mat tmp = Mat(m_depthSize, DEPTH_PIXEL_TYPE, m_pDepthBuffer, Mat::AUTO_STEP);
			m_depthImage = tmp.clone(); //need to deep copy because of the call to SafeRelease(pDepthFrame) to prevent access violation
			ReleaseMutex(hDepthMutex);
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
			if(SUCCEEDED(hr)) {
				WaitForSingleObject(hColorMutex,INFINITE);
				m_colorImage.release();
				Mat tmp = Mat(m_colorSize, COLOR_PIXEL_TYPE, pColorBuffer, Mat::AUTO_STEP);
				m_colorImage = tmp.clone();
				ReleaseMutex(hColorMutex);
			}
		}

		//// get body index frame data

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		//SafeRelease(pBodyIndexFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	//SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
}

#pragma endregion

//Camera Functions
#pragma region Camera

void KinectGrabber::GetColor(Mat &image) {
	WaitForSingleObject(hColorMutex,INFINITE);
	image = m_colorImage;
	ReleaseMutex(hColorMutex);
}

#pragma endregion

//Depth Functions
#pragma region Depth

void KinectGrabber::GetDepth(Mat &image) {
	WaitForSingleObject(hDepthMutex,INFINITE);
	image = m_depthImage;
	ReleaseMutex(hDepthMutex);
}

#pragma endregion

#pragma region Cloud

#pragma endregion
