#include "Microsoft_grabber2.h"

using namespace std;
using namespace cv;
int indexx = 0;
int DownSample_scale = 2;

// Safe release for interfaces
template <class Interface>
inline void SafeRelease(Interface *&pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

DWORD ProcessThread(LPVOID pParam)
{
	KinectGrabber *p = (KinectGrabber *)pParam;
	p->ProcessThreadInternal();
	return 0;
}

template <typename T>
inline T Clamp(T a, T minn, T maxx)
{
	return (a < minn) ? minn : ((a > maxx) ? maxx : a);
}

KinectGrabber::KinectGrabber(const int instance)
{
	HRESULT hr;
	int num = 0;
	m_person = m_depthStarted = m_videoStarted = m_audioStarted = m_infraredStarted = false;
	hStopEvent = NULL;
	hKinectThread = NULL;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		throw exception("Error could not get default kinect sensor");
	}

	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
			if (SUCCEEDED(hr))
			{
				m_videoStarted = m_depthStarted = true;
			}
			else
				throw exception("Failed to Open Kinect Multisource Stream");
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		throw exception("No ready Kinect found");
	}
	m_colorSize = Size(cColorWidth, cColorHeight);
	m_depthSize = Size(cDepthWidth, cDepthHeight);
}
cv::Mat KinectGrabber::ConvertMat(const RGBQUAD *pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar *p_mat = img.data;

	const RGBQUAD *pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}

void KinectGrabber::start()
{
	hDepthMutex = CreateMutex(NULL, false, NULL);
	if (hDepthMutex == NULL)
		throw exception("Could not create depth mutex");
	hColorMutex = CreateMutex(NULL, false, NULL);
	if (hColorMutex == NULL)
		throw exception("Could not create color mutex");
	hCloudMutex = CreateMutex(NULL, false, NULL);
	if (hCloudMutex == NULL)
		throw exception("Could not create cloud mutex");
	hRGBMutex = CreateMutex(NULL, false, NULL);
	if (hRGBMutex == NULL)
		throw exception("Could not create RGB mutex");
	hStopEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	hKinectThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread, this, 0, NULL);
}

void KinectGrabber::stop()
{
	//stop the ProcessThread
	if (hStopEvent != NULL)
	{
		//signal the process to stop
		SetEvent(hStopEvent);
		if (hKinectThread != NULL)
		{
			WaitForSingleObject(hKinectThread, INFINITE);
			CloseHandle(hKinectThread);
			hKinectThread = NULL;
		}
		CloseHandle(hStopEvent);
		hStopEvent = NULL;
		CloseHandle(hDepthMutex);
		hDepthMutex = NULL;
		CloseHandle(hColorMutex);
		hColorMutex = NULL;
		CloseHandle(hCloudMutex);
		hCloudMutex = NULL;
		CloseHandle(hRGBMutex);
		hRGBMutex = NULL;
	}
}

KinectGrabber::~KinectGrabber()
{
	Release();
}

bool KinectGrabber::GetCameraSettings()
{
	return false;
}

void KinectGrabber::ProcessThreadInternal()
{
	bool quit = false;
	while (!quit)
	{
		// Wait for any of the events to be signalled
		if (WaitForSingleObject(hStopEvent, 1) == WAIT_OBJECT_0)
			quit = true;
		else
		{
			//Get the newest frame info
			GetNextFrame();
		}
	}
}

void KinectGrabber::Release()
{
	try
	{
		//clean up stuff here
		stop();
		if (m_pKinectSensor)
		{
			//Shutdown NUI and Close handles
			if (m_pMultiSourceFrameReader)
				SafeRelease(m_pMultiSourceFrameReader);
			if (m_pCoordinateMapper)
				SafeRelease(m_pCoordinateMapper);
			// close the Kinect Sensor
			if (m_pKinectSensor)
				m_pKinectSensor->Close();

			SafeRelease(m_pKinectSensor);
		}
	}
	catch (...)
	{
		//destructor never throws
	}
}

string KinectGrabber::getName() const
{
	return std::string("Kinect2Grabber");
}

float KinectGrabber::getFramesPerSecond() const
{
	return 30.0f;
}

void KinectGrabber::GetNextFrame()
{
	if (!m_pMultiSourceFrameReader)
	{
		return;
	}

	IMultiSourceFrame *pMultiSourceFrame = NULL;
	IDepthFrame *pDepthFrame = NULL;
	IColorFrame *pColorFrame = NULL;
	//IBodyIndexFrame* pBodyIndexFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference *pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference *pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

		INT64 nDepthTime = 0;
		IFrameDescription *pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;

		IFrameDescription *pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

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
		//cv::Mat bufferMat(nColorHeight, nColorWidth, CV_8UC4);
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE **>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE *>(pColorBuffer), ColorImageFormat_Bgra);
				/*bufferMat.data = pColorBuffer;*/
			}
			else
			{
				hr = E_FAIL;
			}
		}

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

		unsigned int bufferSize = nDepthWidth * nDepthHeight * sizeof(unsigned short);
		cv::Mat bufferMatDepth(nDepthHeight, nDepthWidth, CV_16UC1);
		cv::Mat coordinateMapperMat(cColorHeight, cColorWidth, CV_16UC1);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16 **>(&bufferMatDepth.data));
		}

		TEPdep = new RGBQUAD[cDepthWidth * cDepthHeight];

		if (pColorFrame && pDepthFrame && m_pCoordinateMapper && SUCCEEDED(hr))
		{
			// Color Frame to Depth Space
			std::vector<DepthSpacePoint> depthSpacePoints(cColorHeight * cColorWidth);
			hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthHeight * nDepthWidth, reinterpret_cast<UINT16 *>(bufferMatDepth.data), cColorHeight * cColorWidth, &depthSpacePoints[0]);

			PointCloud_X_temp.release();
			PointCloud_Y_temp.release();
			PointCloud_Z_temp.release();

			// Create Matrices for point clouds
			PointCloud_X_temp = Mat::zeros((cColorHeight / DownSample_scale) * (cColorWidth / DownSample_scale), 1, CV_16UC1);
			PointCloud_Y_temp = Mat::zeros((cColorHeight / DownSample_scale) * (cColorWidth / DownSample_scale), 1, CV_16UC1);
			PointCloud_Z_temp = Mat::zeros((cColorHeight / DownSample_scale) * (cColorWidth / DownSample_scale), 1, CV_16UC1);

			Mat tmpc = Mat(m_colorSize, COLOR_PIXEL_TYPE, pColorBuffer, Mat::AUTO_STEP);
			Mat DownSampleColor = Mat::zeros((cColorHeight / DownSample_scale), (cColorWidth / DownSample_scale), CV_8UC3);

			for (int y = 0; y < cColorHeight; y = y + DownSample_scale)
			{
				for (int x = 0; x < cColorWidth; x = x + DownSample_scale)
				{

					unsigned int index = y * cColorWidth + x;
					DepthSpacePoint point = depthSpacePoints[index];
					int depthX = static_cast<int>(std::floor(point.X + 0.5));
					int depthY = static_cast<int>(std::floor(point.Y + 0.5));
					if ((depthX >= 0) && (depthX < nDepthWidth) && (depthY >= 0) && (depthY < nDepthHeight))
					{
						coordinateMapperMat.at<UINT16>(y, x) = bufferMatDepth.at<UINT16>(depthY, depthX);

						cv::Point3f point_3D;

						// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
						CameraSpacePoint cameraSpacePoint = {0.0f, 0.0f, 0.0f};
						UINT16 depth = bufferMatDepth.at<UINT16>(depthY, depthX);
						hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(point, depth, &cameraSpacePoint);

						point_3D.x = cameraSpacePoint.X;
						point_3D.y = cameraSpacePoint.Y;
						point_3D.z = cameraSpacePoint.Z;

						float temp_X = cameraSpacePoint.X * 1000.f;
						float temp_Y = cameraSpacePoint.Y * 1000.f;
						float temp_Z = cameraSpacePoint.Z * 1000.f;

						PointCloud_X_temp.ptr<short int>((y / DownSample_scale) * (cColorWidth / DownSample_scale) + x / DownSample_scale)[0] = (temp_X > 0) ? (short int)(temp_X + 0.5) : (short int)(temp_X - 0.5);
						PointCloud_Y_temp.ptr<short int>((y / DownSample_scale) * (cColorWidth / DownSample_scale) + x / DownSample_scale)[0] = (temp_Y > 0) ? (short int)(temp_Y + 0.5) : (short int)(temp_Y - 0.5);
						PointCloud_Z_temp.ptr<short int>((y / DownSample_scale) * (cColorWidth / DownSample_scale) + x / DownSample_scale)[0] = (temp_Z > 0) ? (short int)(temp_Z + 0.5) : (short int)(temp_Z - 0.5);

						DownSampleColor.ptr<uchar>(y / DownSample_scale)[3 * (x / DownSample_scale)] = tmpc.ptr<uchar>(y)[4 * x];
						DownSampleColor.ptr<uchar>(y / DownSample_scale)[3 * (x / DownSample_scale) + 1] = tmpc.ptr<uchar>(y)[4 * x + 1];
						DownSampleColor.ptr<uchar>(y / DownSample_scale)[3 * (x / DownSample_scale) + 2] = tmpc.ptr<uchar>(y)[4 * x + 2];
					}
				}
			}

			if (SUCCEEDED(hr))
			{

				WaitForSingleObject(hCloudMutex, INFINITE);

				PointCloud_X.release();
				PointCloud_Y.release();
				PointCloud_Z.release();
				PointCloud_X_temp.copyTo(PointCloud_X);
				PointCloud_Y_temp.copyTo(PointCloud_Y);
				PointCloud_Z_temp.copyTo(PointCloud_Z);

				ReleaseMutex(hCloudMutex);

				WaitForSingleObject(hRGBMutex, INFINITE);

				m_RGBImage.release();
				m_RGBImage = DownSampleColor.clone(); //need to deep copy because of the call to SafeRelease(pDepthFrame) to prevent access violation

				ReleaseMutex(hRGBMutex);

				// Original Depth
				WaitForSingleObject(hDepthMutex, INFINITE);

				m_depthImage.release();
				m_depthImage = bufferMatDepth.clone();

				ReleaseMutex(hDepthMutex);

				WaitForSingleObject(hColorMutex, INFINITE);

				m_colorImage.release();
				m_colorImage = tmpc.clone();

				ReleaseMutex(hColorMutex);
				//	index++;
			}
			colorImg.release();
			tmpc.release();
			bufferMat.release();
			PointCloud_X_temp.release();
			PointCloud_Y_temp.release();
			PointCloud_Z_temp.release();
		}

		if (m_pColorRGBX)
		{
			delete[] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
		if (TEPdep)
		{
			delete[] TEPdep;
			TEPdep = NULL;
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	//SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
}

#pragma endregion

//Camera Functions
#pragma region Camera

void KinectGrabber::GetColor(Mat &image)
{
	WaitForSingleObject(hColorMutex, INFINITE);
	image = m_colorImage;
	ReleaseMutex(hColorMutex);
}

#pragma endregion

//Depth Functions
#pragma region Depth

void KinectGrabber::GetDepth(Mat &image)
{
	WaitForSingleObject(hDepthMutex, INFINITE);
	image = /*coordinateMapperDepthMat;*/ m_depthImage;
	ReleaseMutex(hDepthMutex);
}

#pragma endregion

#pragma region Cloud

void KinectGrabber::GetCloud(/*std::vector<cv::Point3f> &cloud,*/ cv::Mat &Point_Cloud_X, cv::Mat &Point_Cloud_Y, cv::Mat &Point_Cloud_Z)
{
	WaitForSingleObject(hCloudMutex, INFINITE);
	//cloud = /*coordinateMapperDepthMat;*/l_pts;
	PointCloud_X.copyTo(Point_Cloud_X);
	PointCloud_Y.copyTo(Point_Cloud_Y);
	PointCloud_Z.copyTo(Point_Cloud_Z);
	ReleaseMutex(hCloudMutex);
}

#pragma endregion

#pragma region RGB

void KinectGrabber::GetRGB(Mat &RGB)
{
	WaitForSingleObject(hRGBMutex, INFINITE);
	RGB = /*coordinateMapperDepthMat;*/ m_RGBImage;
	ReleaseMutex(hRGBMutex);
}

#pragma endregion
