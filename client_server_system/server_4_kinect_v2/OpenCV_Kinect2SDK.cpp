#include "OpenCV_KinectSDK.h"
#include "DepthCodec.h"
#include "data.h"
#include <fstream>
#include <sstream>

typedef unsigned short      XnUInt16;
typedef unsigned char XnUInt8;
#define WIDTH1 1920
#define HEIGHT1 1080
#define WIDTH 512
#define HEIGHT 424
#define JPEGQUALITY 40 // (worst) 1-100 (best)
int index1 = 0;
int index2 = 0;
int index = 0;
using namespace std;
using namespace cv;

void saveDepth(cv::Mat depth, char* filename)
{
	cv::Mat depth_jpg = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			depth_jpg.ptr<uchar>(y)[x] = 255 * depth.ptr<short int>(y)[x] / 5000/*1200*//*4600*/;
		}
	}

	cv::imwrite(filename, depth_jpg);
}

void saveCloud(std::vector<int64_t> cloud, char* filename)
{
	ofstream out(filename);
	for (unsigned i = 0; i < cloud.size(); i++)
	{
		out << cloud[i];
		out << "\n";
	}

	out.close();
}

void saveAutoImages()
{
	/*Save 1st RGB image*/
	sprintf(filename, "captured/kinect_1_rgb_%02d.jpg", index);
	cv::imwrite(filename, kinect_1_rgb);

	/*Save 1st depth image*/
	sprintf(filename, "captured/kinect_1_dep_%02d.dat", index);
	fp = fopen(filename, "wb");
	fwrite(kinect_1_dep.data, 2, HEIGHT*WIDTH, fp);
	fclose(fp);

	sprintf(filename, "captured/kinect_1_dep_%02d.jpg", index);
	saveDepth(kinect_1_dep, filename);


	/*Save 2nd RGB image*/
	sprintf(filename, "captured/kinect_2_rgb_%02d.jpg", index);
	cv::imwrite(filename, kinect_2_rgb);


	/*Save 2nd depth image*/
	sprintf(filename, "captured/kinect_2_dep_%02d.dat", index);
	fp = fopen(filename, "wb");
	fwrite(kinect_2_dep.data, 2, HEIGHT*WIDTH, fp);
	fclose(fp);

	sprintf(filename, "captured/kinect_2_dep_%02d.jpg", index);
	saveDepth(kinect_2_dep, filename);



	///*Save 3rd RGB image*/
	//sprintf(filename, "captured/kinect_3_rgb_%02d.jpg", index);
	//cv::imwrite(filename, kinect_3_rgb);


	///*Save 3rd depth image*/
	//sprintf(filename, "captured/kinect_3_dep_%02d.dat", index);
	//fp = fopen(filename, "wb");
	//fwrite(kinect_3_dep.data, 2, HEIGHT*WIDTH, fp);
	//fclose(fp);

	//sprintf(filename, "captured/kinect_3_dep_%02d.jpg", index);
	//saveDepth(kinect_3_dep, filename);
	index++;
}

DWORD ProcessThread1(LPVOID pParam) {
	SaveKinectData *p = (SaveKinectData*) pParam;
	p->StartCapturingInternal();
	return 0;
}


DWORD ProcessThread2(LPVOID pParam) {
	SaveKinectData *p = (SaveKinectData*) pParam;
	p->StartSavingInternal();
	return 0;
}

DWORD WINAPI  SaveKinectData::kinectStaticThread_1(PVOID lpParam)
{
	SaveKinectData* context = static_cast<SaveKinectData*>(lpParam);
	//if (context)
	//{
		context->kinectThread_1();
	//}
	return 0;
}

void  SaveKinectData::kinectThread_1()
{
	hDoneSaving = CreateEvent(NULL, FALSE, FALSE, NULL);
	while (transfer_sign == 1)
	{
		//kinect 1
		int status_1 = kinect_1.startTransfer();
		if (status_1 == 0)
			fprintf(stderr, "Capture image %d\n", index1);
		else {
			fprintf(stderr, "Problem with connection. Abort\n");
			exit(-1);
		}
		kinect_1.getData(kinect_1_rgb, kinect_1_dep, kinect_1_cloud);
		/*Save 1st RGB image*/
		sprintf(filename, "captured/kinect_1_rgb_%02d.jpg", index1);
		cv::imwrite(filename, kinect_1_rgb);

		/*Save 1st depth image*/
		sprintf(filename, "captured/kinect_1_dep_%02d.dat", index1);
		fp = fopen(filename, "wb");
		fwrite(kinect_1_dep.data, 2, HEIGHT*WIDTH, fp);
		fclose(fp);

		sprintf(filename, "captured/kinect_1_dep_%02d.jpg", index1);
		saveDepth(kinect_1_dep, filename);

		sprintf(filename, "captured/kinect_1_cloud_%02d.txt", index1);
		saveCloud(kinect_1_cloud, filename);
		//saveAutoImages();
		++index1;
		//if (waitKey(10) == VK_ESCAPE){
		//	break;
		//}

	}
	SetEvent(hDoneSaving);
	//return 0;
}

//DWORD WINAPI  SaveKinectData::kinectStaticThread_2(PVOID lpParam)
//{
//	SaveKinectData* context = static_cast<SaveKinectData*>(lpParam);
//	//if (context)
//	//{
//	context->kinectThread_2();
//	//}
//	return 0;
//}
//
//void  SaveKinectData::kinectThread_2()
//{
//	//hDoneSaving = CreateEvent(NULL, FALSE, FALSE, NULL);
//	while (transfer_sign == 1)
//	{
//		//kinect 1
//		int status_2 = kinect_2.startTransfer();
//		if (status_2 == 0)
//			fprintf(stderr, "okay.\n Press ESC to quit\n");
//		else {
//			fprintf(stderr, "Problem with connection. Abort\n");
//			//exit(-1);
//		}
//		kinect_2.getData(kinect_2_rgb, kinect_2_dep);
//		/*Save 2nd RGB image*/
//		sprintf(filename, "captured/kinect_2_rgb_%02d.jpg", index2);
//		cv::imwrite(filename, kinect_2_rgb);
//
//
//		/*Save 2nd depth image*/
//		sprintf(filename, "captured/kinect_2_dep_%02d.dat", index2);
//		fp = fopen(filename, "wb");
//		fwrite(kinect_2_dep.data, 2, HEIGHT*WIDTH, fp);
//		fclose(fp);
//
//		sprintf(filename, "captured/kinect_2_dep_%02d.jpg", index2);
//		saveDepth(kinect_2_dep, filename);
//		//saveAutoImages();
//		index2++;
//		//if (waitKey(10) == VK_ESCAPE){
//		//	break;
//		//}
//
//	}
//	//SetEvent(hDoneSaving);
//	//return 0;
//}

//DWORD WINAPI  SaveKinectData::kinectStaticThread_3(PVOID lpParam)
//{
//	SaveKinectData* context = static_cast<SaveKinectData*>(lpParam);
//	//if (context)
//	//{
//	context->kinectThread_3();
//	//}
//	return 0;
//}

//void  SaveKinectData::kinectThread_3()
//{
//	hDoneSaving = CreateEvent(NULL, FALSE, FALSE, NULL);
//	while (transfer_sign == 1)
//	{
//		//kinect 3
//		int status_3 = kinect_3.startTransfer();
//		if (status_3 == 0)
//			fprintf(stderr, "okay.\n Press ESC to quit\n");
//		else {
//			fprintf(stderr, "Problem with connection. Abort\n");
//			//exit(-1);
//		}
//		kinect_3.getData(kinect_3_rgb, kinect_3_dep);
//		saveAutoImages();
//
//		//if (waitKey(10) == VK_ESCAPE){
//		//	break;
//		//}
//
//	}
//	SetEvent(hDoneSaving);
//	//return 0;
//}


void SaveKinectData::StartCapturingInternal() {
	if(!kinect_init) {
		kc = new KinectGrabber();
		kinect_init = true;
		kc->start();
	}
	Sleep(100);
	while(!quit) {
		Mat *img = new Mat();
		Mat *depth = new Mat();
		kc->GetDepth(*depth);
		kc->GetColor(*img);
		if(!img->empty())
			imgs.push_back(img);
		if(!depth->empty())
			depths.push_back(depth);
		SleepEx(250,false);
	}
}



void SaveKinectData::StartSavingInternal() {
	hDoneSaving = CreateEvent(NULL, FALSE, FALSE, NULL);
	while (!quit || !imgs.empty() || !depths.empty()) {
		if (!imgs.empty() && !depths.empty()) {
			Mat* img = imgs.front();
			Mat* depth = depths.front();
			imgs.pop_front();
			depths.pop_front();

			sprintf(filename, "captured/kinect_1_rgb_%02d.jpg", count);
			imwrite(filename, *img);

			/*Save 1st depth image*/
			sprintf(filename, "captured/kinect_1_dep_%02d.dat", count);
			fp = fopen(filename, "wb");
			fwrite(depth, 2, HEIGHT*WIDTH, fp);
			fclose(fp);

			sprintf(filename, "captured/kinect_1_dep_%02d.jpg", count);
			saveDepth(*depth, filename);

			img->release();
			delete img;
			depth->release();
			delete depth;
			++count;
		}
	}
	SetEvent(hDoneSaving);
}



void SaveKinectData::Start() {
	quit = false;
	imgs.clear();
	depths.clear();
	//colors_k.clear();
	//depths_k.clear();
	count = 0;
	
	//hCaptureThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread1, this, 0, NULL );
	//hSaveThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread2, this, 0, NULL );
	kinectHandle_1 = CreateThread(NULL, 0, kinectStaticThread_1, (PVOID)this, 0, NULL);
	//kinectHandle_2 = CreateThread(NULL, 0, kinectStaticThread_2, (PVOID)this, 0, NULL);
	//kinectHandle_3 = CreateThread(NULL, 0, kinectStaticThread_3, (PVOID)this, 0, NULL);
	

}

void SaveKinectData::Stop() {
	//doesn't work right now, really need to fix that
	//quit = true;
	//kc->stop();
	//WaitForSingleObject(hDoneSaving,INFINITE);
	CloseHandle(hDoneSaving);
	hDoneSaving = NULL;
	exit(-1);
}

void DisplayImages() {
	try {
		Mat image, depth, depth_colored;
		KinectGrabber grabber;
		grabber.start();
		while (1) {
			//grabber.GetNextFrame();
			grabber.GetDepth(depth);
			grabber.GetColor(image);
			if (!image.empty() && !depth.empty())
			{
				imshow("image", image);
				imshow("depth", depth);
			}
			cvWaitKey(1);
		}
	}
	catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
}


void SaveImages(string out) {
	SaveKinectData save(out);
	save.Start();
	cout << "starting save" << endl;
	cin.get();
	cout << "Got end signal!" << endl;
	exit(0);
	//save.Stop();
}

int main () {
	SaveImages(string("./captured/"));
	//DisplayImages();
	return 0;
}