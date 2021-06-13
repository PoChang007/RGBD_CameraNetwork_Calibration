// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include "OpenCV_KinectSDK.h"
#include "DepthCodec.h"
#include "data.h"

#define WIDTH 960
#define HEIGHT 540
#define WIDTH1 1920
#define HEIGHT1 1080
#define JPEGQUALITY 40 // (worst) 1-100 (best)

char filename[200];
int sphere_sign = 0;
SaveKinectData save("./captured/");
using namespace std;
using namespace cv;

extern ofstream out_txt;
DWORD ProcessThread1(LPVOID pParam)
{
	SaveKinectData *p = (SaveKinectData *)pParam;
	p->StartCapturingInternal();
	return 0;
}

void SaveKinectData::FindBlobs(const cv::Mat &binary, std::vector<std::vector<cv::Point2i>> &blobs)
{
	blobs.clear();

	// Fill the label_image with the blobs
	// 0  - background
	// 1  - unlabelled foreground
	// 2+ - labelled foreground

	cv::Mat label_image;
	binary.assignTo(label_image, CV_32FC1); // weird it doesn't support CV_32S!

	int label_count = 2; // starts at 2 because 0,1 are used already

	for (int y = 0; y < binary.rows; y++)
	{
		for (int x = 0; x < binary.cols; x++)
		{
			if ((int)label_image.at<float>(y, x) != 255)
			{
				continue;
			}

			cv::Rect rect;
			cv::floodFill(label_image, cv::Point(x, y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);

			std::vector<cv::Point2i> blob;

			for (int i = rect.y; i < (rect.y + rect.height); i++)
			{
				for (int j = rect.x; j < (rect.x + rect.width); j++)
				{
					if ((int)label_image.at<float>(i, j) != label_count)
					{
						continue;
					}

					blob.push_back(cv::Point2i(j, i));
				}
			}

			blobs.push_back(blob);

			label_count++;
		}
	}
}

/*Find the corresponding 2D point on the sphere of both images*/
void SaveKinectData::findSphereBlobs(cv::Mat &Stored_Image, int index)
{
	/*This part is to find connecting parts on the left image*/
	std::vector<std::vector<cv::Point2i>> blobs;
	FindBlobs(l_thr, blobs);

	/*Find the largest connected points of left image*/
	int max_pts = 0;
	int max_idx = 0;
	for (int i = 0; i < (int)blobs.size(); i++)
	{
		if ((int)blobs[i].size() > max_pts)
		{
			max_idx = i;
			max_pts = blobs[i].size();
		}
	}
	if (blobs.size() > 0)
	{
		l_blob = blobs[max_idx];
	}

	/********************* For testing purpose ***************************************/

	/*Generate slide window to go through all the pixels*/
	cv::Mat l_thr_3c, r_thr_3c; /*3-channel version of l_thr, r_thr*/
	l_thr_3c.create(l_thr.rows, l_thr.cols, CV_8UC3);

	cv::Mat BinaryImage = cv::Mat::zeros(l_thr.rows, l_thr.cols, CV_8UC3);

	Stored_Image.copyTo(l_thr_3c);

	for (int i = 0; i < (int)l_blob.size(); i++)
	{
		int x = l_blob[i].x;
		int y = l_blob[i].y;

		l_thr_3c.ptr<uchar>(y)[3 * x] = 255;
		l_thr_3c.ptr<uchar>(y)[3 * x + 1] = 0;
		l_thr_3c.ptr<uchar>(y)[3 * x + 2] = 0;

		BinaryImage.ptr<uchar>(y)[3 * x] = 255;
		BinaryImage.ptr<uchar>(y)[3 * x + 1] = 255;
		BinaryImage.ptr<uchar>(y)[3 * x + 2] = 255;
	}

	/*Save the images to file*/
	{
		char filename[200];
		sprintf(filename, "tracking/img_%d_%03d.jpg", CLIENTID, index);
		cv::imwrite(filename, l_thr_3c);

		sprintf(filename, "tracking/Binaryimg_%d_%03d.jpg", CLIENTID, index);
		cv::imwrite(filename, BinaryImage);
	}
}

void SaveKinectData::regionDetection(cv::Mat &left, int i)
{
	cv::Mat l_hsv;

	cv::cvtColor(left, l_hsv, CV_BGR2HSV);

	/*store the result after applying threshold*/
	//cv::inRange(l_hsv, cv::Scalar(10, 160, 150), cv::Scalar(80, 255, 255), l_thr);
	cv::inRange(l_hsv, cv::Scalar(10, 130, 60), cv::Scalar(80, 255, 255), l_thr);

	//cv::erode(l_thr, l_thr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1)));

	//char filename[200];
	//sprintf(filename, "tracking/initial_%d_%03d.jpg", CLIENTID, i);
	//cv::imwrite(filename, l_thr);

	/*Find the corresponding 2D point blob on both images*/
	findSphereBlobs(left, i);

	if (i % 10 == 0)
		printf("%02d region detection is done\n", i);
}

void SaveKinectData::compute3Dpoints(cv::Mat &PointCloudX, cv::Mat &PointCloudY, cv::Mat &PointCloudZ, int index)
{
	/*At the begining, clear the stored points*/
	l_pts.clear();
	sphere_sign = 0; // in the begining set the sphere sign as 0;
	arr_idx.clear();
	sphere_radius = 0; //as I use the radius as condition for ransac purpose, so set it to zero for each new frame
	final_pts.clear();

	/*compute the left image*/
	char filename[200];
	sprintf(filename, "tracking/kinect_%d_%02d.txt", CLIENTID, index);
	ofstream out(filename);

	for (int i = 0; i < (int)l_blob.size(); i++)
	{
		int x_2d = l_blob[i].x;
		int y_2d = l_blob[i].y;

		cv::Point3f pt;
		pt.x = (float)PointCloudX.ptr<short int>(y_2d)[x_2d];
		pt.y = (float)PointCloudY.ptr<short int>(y_2d)[x_2d];
		pt.z = (float)PointCloudZ.ptr<short int>(y_2d)[x_2d];

		/*check whether the depth is measurable*/
		if (pt.z != 0)
		{

			if (pt.z < 3500) // Threshold the maximum depth value (e.g. 2500, 5500, etc.)
			{
				l_pts.push_back(pt);

				/*save the data*/
				out << pt.x;
				out << " ";
				out << pt.y;
				out << " ";
				out << pt.z;
				out << "\n";
			}
		}
	}
	out.close();
}

void SaveKinectData::sphereFitting(int index)
{
	/* initialize random seed: */
	int total = l_pts.size();
	if (total > pt_rand)
	{
		r_pts = l_pts;

		int random = 0;
		while (random < ransac_max && sphere_sign == 0)
		{
			std::random_shuffle(r_pts.begin(), r_pts.end());

			vector<cv::Point3f>::const_iterator first = r_pts.begin();
			vector<cv::Point3f>::const_iterator last = r_pts.begin() + pt_rand;
			std::vector<cv::Point3f> p_pts(first, last);

			/*Compute the sphere centers*/
			computeSpehereCenter(p_pts);

			/*Check whether a possible center is found*/
			if (sphere_radius > (estimated_radius - 20) && sphere_radius < (estimated_radius + 20))
			{
				/*further refine the sphere center and radius by encompassing more points*/
				sphereRefine(index);
			}
			random++;
		}
	}
	else
	{
		sphere_center = cv::Mat::zeros(3, 1, CV_32FC1);
	}
}

/*Compute the sphere center based on the point cloud*/
void SaveKinectData::computeSpehereCenter(std::vector<cv::Point3f> &cur_pts)
{
	/*change vector into matrix*/
	pts_mat = cv::Mat::zeros(cur_pts.size(), 3, CV_32FC1);
	for (int row = 0; row < pts_mat.rows; row++)
	{
		pts_mat.ptr<float>(row)[0] = cur_pts[row].x /**1000.0f*/;
		pts_mat.ptr<float>(row)[1] = cur_pts[row].y /**1000.0f*/;
		pts_mat.ptr<float>(row)[2] = cur_pts[row].z /**1000.0f*/;
	}
	cv::Mat col_1, col_2, col_3; //store each column of the point cloud matrix

	pts_mat(cv::Range(0, pts_mat.rows), cv::Range(0, 1)).copyTo(col_1);
	pts_mat(cv::Range(0, pts_mat.rows), cv::Range(1, 2)).copyTo(col_2);
	pts_mat(cv::Range(0, pts_mat.rows), cv::Range(2, 3)).copyTo(col_3);

	/*Compute the part from Matlab*/
	cv::Mat A = cv::Mat::zeros(3, 3, CV_32FC1);

	/*  Compute a_11  */
	cv::Mat col_1_mean = col_1 - cv::mean(col_1)(0);
	cv::Mat col_1_dot_1_mean;
	cv::multiply(col_1, col_1_mean, col_1_dot_1_mean, 1.0);
	A.ptr<float>(0)[0] = 2 * (float)cv::mean(col_1_dot_1_mean)(0);

	/*  Compute a_12  */
	cv::Mat col_2_mean = col_2 - cv::mean(col_2)(0);
	cv::Mat col_1_dot_2_mean;
	cv::multiply(col_1, col_2_mean, col_1_dot_2_mean, 1.0);
	A.ptr<float>(0)[1] = 2 * (float)cv::mean(col_1_dot_2_mean)(0);

	/*  Compute a_13  */
	cv::Mat col_3_mean = col_3 - cv::mean(col_3)(0);
	cv::Mat col_1_dot_3_mean;
	cv::multiply(col_1, col_3_mean, col_1_dot_3_mean, 1.0);
	A.ptr<float>(0)[2] = 2 * (float)cv::mean(col_1_dot_3_mean)(0);

	/*  Compute a_21  */
	cv::Mat col_2_dot_1_mean;
	cv::multiply(col_2, col_1_mean, col_2_dot_1_mean, 1.0);
	A.ptr<float>(1)[0] = 2 * (float)cv::mean(col_2_dot_1_mean)(0);

	/*  Compute a_22  */
	cv::Mat col_2_dot_2_mean;
	cv::multiply(col_2, col_2_mean, col_2_dot_2_mean, 1.0);
	A.ptr<float>(1)[1] = 2 * (float)cv::mean(col_2_dot_2_mean)(0);

	/*  Compute a_23  */
	cv::Mat col_2_dot_3_mean;
	cv::multiply(col_2, col_3_mean, col_2_dot_3_mean, 1.0);
	A.ptr<float>(1)[2] = 2 * (float)cv::mean(col_2_dot_3_mean)(0);

	/*  Compute a_31  */
	cv::Mat col_3_dot_1_mean;
	cv::multiply(col_3, col_1_mean, col_3_dot_1_mean, 1.0);
	A.ptr<float>(2)[0] = 2 * (float)cv::mean(col_3_dot_1_mean)(0);

	/*  Compute a_32  */
	cv::Mat col_3_dot_2_mean;
	cv::multiply(col_3, col_2_mean, col_3_dot_2_mean, 1.0);
	A.ptr<float>(2)[1] = 2 * (float)cv::mean(col_3_dot_2_mean)(0);

	/*  Compute a_33  */
	cv::Mat col_3_dot_3_mean;
	cv::multiply(col_3, col_3_mean, col_3_dot_3_mean, 1.0);
	A.ptr<float>(2)[2] = 2 * (float)cv::mean(col_3_dot_3_mean)(0);

	/*Compute Matrix B*/
	cv::Mat B = cv::Mat::zeros(3, 1, CV_32FC1);
	cv::Mat sqr_11, sqr_22, sqr_33;
	cv::multiply(col_1, col_1, sqr_11, 1.0);
	cv::multiply(col_2, col_2, sqr_22, 1.0);
	cv::multiply(col_3, col_3, sqr_33, 1.0);
	cv::Mat sqr_sum = sqr_11 + sqr_22 + sqr_33;
	cv::Mat b1, b2, b3;
	cv::multiply(sqr_sum, col_1_mean, b1, 1.0);
	cv::multiply(sqr_sum, col_2_mean, b2, 1.0);
	cv::multiply(sqr_sum, col_3_mean, b3, 1.0);

	B.ptr<float>(0)[0] = (float)cv::mean(b1)(0);
	B.ptr<float>(1)[0] = (float)cv::mean(b2)(0);
	B.ptr<float>(2)[0] = (float)cv::mean(b3)(0);

	/*Compute the center*/
	cv::Mat center;
	cv::solve(A, B, center, cv::DECOMP_SVD);

	/*Compute the radius*/
	cv::Mat sum_cols;
	cv::Mat col_1_sqr, col_2_sqr, col_3_sqr;
	cv::multiply((col_1 - center.ptr<float>(0)[0]), (col_1 - center.ptr<float>(0)[0]), col_1_sqr);
	cv::multiply((col_2 - center.ptr<float>(1)[0]), (col_2 - center.ptr<float>(1)[0]), col_2_sqr);
	cv::multiply((col_3 - center.ptr<float>(2)[0]), (col_3 - center.ptr<float>(2)[0]), col_3_sqr);
	sum_cols = col_1_sqr + col_2_sqr + col_3_sqr;
	float radius = cv::sqrt(cv::mean(sum_cols)(0));

	/*I will do more code here to verify the radius and center, here copy the temporary data to global*/
	center.copyTo(sphere_center);
	sphere_radius = radius;

	//printf("[%f, %f, %f]\n", center.ptr<float>(0)[0], center.ptr<float>(1)[0], center.ptr<float>(2)[0]);
	//printf("radius: %.5f\n", radius);
}

void SaveKinectData::sphereRefine(int index)
{
	/*Compute the distance between the estimated center and all the points*/
	cv::Mat all_pts_mat(l_pts.size(), 3, CV_32FC1);

	for (int i = 0; i < all_pts_mat.rows; i++)
	{
		all_pts_mat.ptr<float>(i)[0] = l_pts[i].x;
		all_pts_mat.ptr<float>(i)[1] = l_pts[i].y;
		all_pts_mat.ptr<float>(i)[2] = l_pts[i].z;
	}

	cv::Mat center_tran, center_mat;
	cv::transpose(sphere_center, center_tran);
	cv::repeat(center_tran, all_pts_mat.rows, 1, center_mat);

	cv::Mat pts_cen = all_pts_mat - center_mat;

	cv::Mat pts_cen_sqr = pts_cen.mul(pts_cen, 1.0);

	cv::Mat dis_mat;

	cv::reduce(pts_cen_sqr, dis_mat, 1, CV_REDUCE_SUM);

	/*Get the final point set*/
	final_pts.clear();
	for (int i = 0; i < dis_mat.rows; i++)
	{
		if (dis_mat.ptr<float>(i)[0] < radius_max_range && dis_mat.ptr<float>(i)[0] > radius_min_range)
		{
			final_pts.push_back(l_pts[i]);
		}
	}

	/*Check the number of satisfactory points*/
	double ratio = (double)final_pts.size() / (double)l_pts.size();

	//printf("************************** good(%d) total(%d) | ratio(%.2f)\n", final_pts.size(), l_pts.size(), ratio);
	printf("[%d] rough radius: %.2f\n", index, sphere_radius);

	if (ratio > 0.8)
	{
		/*set the sign to 1 to quit the loop*/
		sphere_sign = 1;

		/*Further refine the radius and center position*/
		computeSpehereCenter(final_pts);
	}
}

void SaveKinectData::sphereTracking()
{
	total_num = save_index - 1;
	int final_num;

	if (total_num > NUMBER)
	{
		final_num = NUMBER;
	}
	else
	{
		final_num = total_num;
	}

	l_pts.clear();
	final_pts.clear();

	/*File used for storing sphere path*/
	char path_file[200];
	sprintf(path_file, "tracking/path_%02d.txt", CLIENTID);
	ofstream out(path_file);

	/*File used for storing estimated radius*/
	char radius_file_test[200];
	sprintf(radius_file_test, "tracking/radius_%02d.txt", CLIENTID);
	ofstream out_test(radius_file_test);

	float radius_list[NUMBER];
	float total_radius = 0.0;
	int total_num = 0;

	for (int index = 0; index < final_num; index++)
	{
		// Read color and 3D point clouds for each frame
		char filename[200];
		sprintf(filename, "captured/kinect_%d_rgb_%02d.jpg", CLIENTID, index);
		cv::imread(filename).copyTo(rgb_l_imgs);

		sprintf(filename, "3D_Data/kinect_%d_PointCloudX_%02d.dat", CLIENTID, index);
		FILE *fpX = fopen(filename, "rb");
		PointCloudX = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
		fread(PointCloudX.data, 2, HEIGHT * WIDTH, fpX);
		fclose(fpX);

		sprintf(filename, "3D_Data/kinect_%d_PointCloudY_%02d.dat", CLIENTID, index);
		FILE *fpY = fopen(filename, "rb");
		PointCloudY = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
		fread(PointCloudY.data, 2, HEIGHT * WIDTH, fpY);
		fclose(fpY);

		sprintf(filename, "3D_Data/kinect_%d_PointCloudZ_%02d.dat", CLIENTID, index);
		FILE *fpZ = fopen(filename, "rb");
		PointCloudZ = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
		fread(PointCloudZ.data, 2, HEIGHT * WIDTH, fpZ);
		fclose(fpZ);

		/*Detecting the ball region*/
		regionDetection(rgb_l_imgs, index);

		/*Compute and store the corresponding 3D points of each detected blob*/
		compute3Dpoints(PointCloudX, PointCloudY, PointCloudZ, index);

		/*Do the denoising and sphere fitting, find an approximately optimal sphere center and radius*/
		sphereFitting(index);

		/*store current center to file*/
		if (sphere_sign == 1)
		{
			out << index;
			out << " ";
			out << sphere_center.ptr<float>(0)[0];
			out << " ";
			out << sphere_center.ptr<float>(1)[0];
			out << " ";
			out << sphere_center.ptr<float>(2)[0];
			out << "\n";

			path_mat.ptr<short int>(index)[0] = (sphere_center.ptr<float>(0)[0] > 0) ? (short int)(sphere_center.ptr<float>(0)[0] + 0.5) : (short int)(sphere_center.ptr<float>(0)[0] - 0.5);
			path_mat.ptr<short int>(index)[1] = (sphere_center.ptr<float>(1)[0] > 0) ? (short int)(sphere_center.ptr<float>(1)[0] + 0.5) : (short int)(sphere_center.ptr<float>(1)[0] - 0.5);
			path_mat.ptr<short int>(index)[2] = (sphere_center.ptr<float>(2)[0] > 0) ? (short int)(sphere_center.ptr<float>(2)[0] + 0.5) : (short int)(sphere_center.ptr<float>(2)[0] - 0.5);

			//cout << "x:" << path_mat.ptr<short int>(index)[0] << endl;
			//cout << "y:" << path_mat.ptr<short int>(index)[1] << endl;
			//cout << "z:" << path_mat.ptr<short int>(index)[2] << endl;

			out_test << sphere_radius;
			out_test << "\n";

			total_radius += sphere_radius;
			radius_list[total_num] = sphere_radius;
			total_num++;
		}
		else
		{
			out << index;
			out << " ";
			out << 0;
			out << " ";
			out << 0;
			out << " ";
			out << -1000;
			out << "\n";

			path_mat.ptr<short int>(index)[0] = 0.0f;
			path_mat.ptr<short int>(index)[1] = 0.0f;
			path_mat.ptr<short int>(index)[2] = -1000.0f;

			out_test << "-------";
			out_test << "\n";
		}

		r_blob.clear();
		l_blob.clear();
		PointCloudX.release();
		PointCloudY.release();
		PointCloudZ.release();
	}

	/*close the output file*/
	out.close();
	out_test.close();
}

void saveDepth(cv::Mat depth, char *filename)
{
	cv::Mat depth_jpg = cv::Mat::zeros(424, 512, CV_8UC1);
	for (int y = 0; y < 424; y++)
	{
		for (int x = 0; x < 512; x++)
		{
			depth_jpg.ptr<uchar>(y)[x] = 255 * depth.ptr<short int>(y)[x] / 5000 /*1200*/ /*4600*/;
		}
	}

	cv::imwrite(filename, depth_jpg);
	depth_jpg.release();
}

void SaveKinectData::StartCapturingInternal()
{

	//Sleep(30);
	if (!quit)
	{
		kc->GetDepth(m_depth);
		kc->GetColor(m_img);
		kc->GetCloud(Point_Cloud_X, Point_Cloud_Y, Point_Cloud_Z);
		kc->GetRGB(m_RGB);

		if (!m_img.empty() && !m_depth.empty() && !m_RGB.empty())
		{
			m_depth.copyTo(send_data);

			/*For calibration capturing images use*/
			if (save_image == 1)
			{
				SYSTEMTIME stime;
				//structure to store system time (in usual time format)
				FILETIME ltime;
				//structure to store local time (local time in 64 bits)
				FILETIME ftTimeStamp;

				char TimeStamp[256];				   //to store TimeStamp information
				GetSystemTimeAsFileTime(&ftTimeStamp); //Gets the current system time

				FileTimeToLocalFileTime(&ftTimeStamp, &ltime); //convert in local time and store in ltime
				FileTimeToSystemTime(&ltime, &stime);		   //convert in system time and store in stime

				/*******************	Save Image to Local *************************/
				FILE *fp;
				char filename[200];

				//Save Mapping RGB image
				sprintf(filename, "captured/kinect_%d_rgb_%02d.jpg", CLIENTID, save_index);
				//printf("m_image:%d, %d\n", m_image.rows, m_image.cols);
				cv::imwrite(filename, m_RGB);

				///*Save HD color image*/
				//sprintf(filename, "captured/kinect_%d_color_%02d.jpg", CLIENTID, save_index);
				//cv::imwrite(filename, m_img);

				//Save 3D data
				FILE *fpX;
				FILE *fpY;
				FILE *fpZ;

				sprintf(filename, "3D_Data/kinect_%d_PointCloudX_%02d.dat", CLIENTID, save_index);
				fpX = fopen(filename, "wb");
				fwrite(Point_Cloud_X.data, 2, HEIGHT * WIDTH, fpX);
				fclose(fpX);

				sprintf(filename, "3D_Data/kinect_%d_PointCloudY_%02d.dat", CLIENTID, save_index);
				fpY = fopen(filename, "wb");
				fwrite(Point_Cloud_Y.data, 2, HEIGHT * WIDTH, fpY);
				fclose(fpY);

				sprintf(filename, "3D_Data/kinect_%d_PointCloudZ_%02d.dat", CLIENTID, save_index);
				fpZ = fopen(filename, "wb");
				fwrite(Point_Cloud_Z.data, 2, HEIGHT * WIDTH, fpZ);
				fclose(fpZ);

				/*Save depth image*/
				//sprintf(filename, "captured/kinect_%d_dep_%02d.dat", CLIENTID, save_index);
				//fp = fopen(filename, "wb");
				//fwrite(send_data.data, 2, HEIGHT*WIDTH, fp);
				//fclose(fp);

				sprintf(filename, "captured/kinect_%d_dep_%02d.jpg", CLIENTID, save_index);
				saveDepth(send_data, filename);

				/*Save the time stamp to .txt file*/
				/*time_t rawtime;
				struct tm * timeinfo;
				time (&rawtime);
				timeinfo = localtime (&rawtime);
				char time_stamp[100];
				sprintf(time_stamp, "%d:%d:%d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->*/

				char time_stamp[100];
				sprintf(time_stamp, "%d:%d:%d:%d", stime.wHour, stime.wMinute, stime.wSecond, stime.wMilliseconds);
				out_txt << save_index;
				out_txt << " ";
				out_txt << time_stamp;
				out_txt << "\n";

				printf("save_index: %d\n", save_index);
				save_index++;
				/********************	End		************************************/
			}

			DepthCodec pointcloudX_Codec;
			pointcloudX_Codec.encode((unsigned short *)Point_Cloud_X.data, WIDTH * HEIGHT * sizeof(XnUInt16),
									 (unsigned char *)m_pointcloudX_Stream, &m_pointcloudX_depthSize);

			DepthCodec pointcloudY_Codec;
			pointcloudX_Codec.encode((unsigned short *)Point_Cloud_Y.data, WIDTH * HEIGHT * sizeof(XnUInt16),
									 (unsigned char *)m_pointcloudY_Stream, &m_pointcloudY_depthSize);

			DepthCodec pointcloudZ_Codec;
			pointcloudX_Codec.encode((unsigned short *)Point_Cloud_Z.data, WIDTH * HEIGHT * sizeof(XnUInt16),
									 (unsigned char *)m_pointcloudZ_Stream, &m_pointcloudZ_depthSize);

			Point_Cloud_X.release();
			Point_Cloud_Y.release();
			Point_Cloud_Z.release();

#ifdef DEBUG
			fprintf(stderr, "Depth: After Encoding  = %u bytes\n", m_depthSize);
			cv::Scalar mean, std;
			unsigned long tmp;
			cv::Mat tmpM(1, m_depthSize, CV_8UC1, m_depthStream);
			cv::meanStdDev(tmpM, mean, std);
			fprintf(stderr, ">>B mean = %f, stdev = %f\n", mean[0], std[0]);
			depthCodec.decode((unsigned char *)m_depthStream, m_depthSize,
							  (unsigned short *)m_depth.data, &tmp);
			cv::meanStdDev(m_depth, mean, std);
			fprintf(stderr, "mean = %f, stdev = %f\n", mean[0], std[0]);
			fprintf(stderr, "Depth max value = %u\n", m_DepthGenerator.GetDeviceMaxDepth());
#endif

			/*Compress Virtual Image to bitstream*/
			// JPEG Quality
			vector<int> jpegfmt;
			jpegfmt.push_back(CV_IMWRITE_JPEG_QUALITY);
			jpegfmt.push_back(JPEGQUALITY);

			cv::imencode(".jpg", m_RGB, m_jpegBuf, jpegfmt);

			m_jpegSize = m_jpegBuf.size();
			char *z = m_jpegStream;
			for (vector<unsigned char>::iterator zz = m_jpegBuf.begin(); zz != m_jpegBuf.end(); zz++)
				*(z++) = *zz;

			jpegfmt.clear();
		}
		//SleepEx(25, false);
	}
}

void SaveKinectData::closeTimeFile()
{
	out_txt.close();
}

void SaveKinectData::openTimeFile()
{
	out_txt.open(path_file, std::ofstream::out | std::ofstream::app);
}

void SaveKinectData::setSaveSignal(int x)
{
	save_image = x;
}
void SaveKinectData::Start()
{
	quit = false;
	m_depthStream = (char *)malloc(WIDTH * HEIGHT * sizeof(XnUInt16));
	m_jpegStream = (char *)malloc(WIDTH * HEIGHT * 4 * sizeof(XnUInt8));
	m_pathStream = (char *)malloc(NUMBER * 3 * sizeof(XnUInt16));
	m_pointcloudX_Stream = (char *)malloc(WIDTH * HEIGHT * sizeof(XnUInt16));
	m_pointcloudY_Stream = (char *)malloc(WIDTH * HEIGHT * sizeof(XnUInt16));
	m_pointcloudZ_Stream = (char *)malloc(WIDTH * HEIGHT * sizeof(XnUInt16));

	m_depthSize = 0; // no useful data
	m_jpegSize = 0;	 // no useful data
	m_pathSize = 0;
	m_pointcloudX_depthSize = 0;
	m_pointcloudY_depthSize = 0;
	m_pointcloudZ_depthSize = 0;

	sprintf(path_file, "captured/time_%02d.txt", CLIENTID);
	out_txt.open(path_file, std::ofstream::out | std::ofstream::app);

	path_mat = cv::Mat::zeros(NUMBER, 3, CV_16UC1);
}

void SaveKinectData::Stop()
{
	delete[] m_depthStream;
	delete[] m_jpegStream;
	delete[] m_pathStream;

	delete[] m_pointcloudX_Stream;
	delete[] m_pointcloudY_Stream;
	delete[] m_pointcloudZ_Stream;

	//delete img;
	m_jpegBuf.clear();
	dp.clear();
	m_img.release();

	//delete depth;
	m_depth.release();

	//delete cloud;
	Point_Cloud_X.release();
	Point_Cloud_Y.release();
	Point_Cloud_Z.release();
	cloud.clear();
	m_RGB.release();
	m_cloud.release();

	path_mat.release();
}

unsigned long SaveKinectData::getPathStreamSize()
{
	DepthCodec pathCodec;
	pathCodec.encode((unsigned short *)path_mat.data, NUMBER * 3 * sizeof(XnUInt16),
					 (unsigned char *)m_pathStream, &m_pathSize);
	return m_pathSize;
}

void SaveKinectData::resetTracking()
{
	save_index = 0;
	path_mat = Mat::zeros(NUMBER, 3, CV_16UC1);
}

void SaveImages(string out, int temp)
{
	SaveKinectData save(out);
	save.csock = temp;
	save.Start();
	cout << "starting save" << endl;
	cin.get();
	cout << "Got end signal!" << endl;
	//save.kc->stop();
	save.Stop();
}

int main()
{
	if (!save.kinect_init)
	{
		save.kc = new KinectGrabber();
		save.kinect_init = true;
		save.kc->start();
	}

	// Initialize socket and setup connection
	int host_port = 3000;
	unsigned short wVersionRequested;
	WSADATA wsaData;
	int err;
	wVersionRequested = MAKEWORD(2, 2);
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0 || (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2))
	{
		fprintf(stderr, "Could not find usable sock dll %d\n", WSAGetLastError());
		exit(-1);
	}
	int hsock;
	int *p_int;
	hsock = socket(AF_INET, SOCK_STREAM, 0);
	if (hsock == -1)
	{
		fprintf(stderr, "Error initializing socket %d\n", WSAGetLastError());
		exit(-1);
	}
	p_int = (int *)malloc(sizeof(int));
	*p_int = 1;
	if ((setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char *)p_int, sizeof(int)) == -1) ||
		(setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char *)p_int, sizeof(int)) == -1))
	{
		fprintf(stderr, "Error setting options %d\n", WSAGetLastError());
		free(p_int);
		exit(-1);
	}
	free(p_int);
	struct sockaddr_in my_addr;
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(host_port);
	memset(&(my_addr.sin_zero), 0, 8);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	if (bind(hsock, (struct sockaddr *)&my_addr, sizeof(my_addr)) == -1)
	{
		fprintf(stderr, "Error binding to socket, make sure nothing else is listening on this port %d\n", WSAGetLastError());
		exit(-1);
	}
	if (listen(hsock, 1) == -1)
	{
		fprintf(stderr, "Error listening %d\n", WSAGetLastError());
		exit(-1);
	}

	save.Start();

	while (1)
	{

		/******************* For Remote Use ************************************************/

		fprintf(stderr, "Press ESC to kill the client ...\n");
		int csock;
		sockaddr_in sadr;
		int addr_size = sizeof(SOCKADDR);
		if ((csock = accept(hsock, (SOCKADDR *)&sadr, &addr_size)) == INVALID_SOCKET)
		{
			fprintf(stderr, "Error accepting %d\n", WSAGetLastError());
			exit(-1);
		}
		printf("Received connection from %s.\n", inet_ntoa(sadr.sin_addr));
		while (1)
		{
			int bytecount;
			unsigned long buf;
			unsigned long save_signal;

			// (1) Receive 3 short int variable, indicating the controlling process
			short int eye_rec[3]; //important signal setting

			if ((bytecount = recv(csock, (char *)eye_rec, sizeof(short int) * 3, 0)) == SOCKET_ERROR)
			{
				fprintf(stderr, "Error receiving JPEG instruction: %d\n", WSAGetLastError());
				exit(-1);
			}

			if (eye_rec[0] == 0) //stop capturing image
			{
				save.closeTimeFile();
				save.setSaveSignal(0);
			}
			else if (eye_rec[0] == 1) //start capturing image
			{
				save.openTimeFile();
				save.setSaveSignal(1);
				eye_rec[0] == 2; //when eye_rec[0] == 2 it means it is capturing the image
			}
			else if (eye_rec[0] == 9999)
			{
				printf("it is breaked\n");
				break;
			}
			else if (eye_rec[2] == 1) //do the image transfering
			{
				save.setSaveSignal(0);
			}
			else if (eye_rec[2] == 1500) //do the image transfering
			{
				save.setSaveSignal(1);
			}

			if (!save.quit)
			{
				//SaveImages(string("./captured/"), csock);
				/*	SaveKinectData save("./captured/");*/
				save.csock = csock;
				save.StartCapturingInternal();
			}

			if (eye_rec[2] == 1) //do the demo
			{
				// (1) Send 1 unsigned long on the size of the JPEG bitstream
				buf = save.getJPEGSize();
				if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error sending JPEG size : %d\n", WSAGetLastError());
					exit(-1);
				}

				// (2) Send JPEG bitstream
				char *bitstream = save.getJPEG();
				unsigned long dataLeft = buf;
				while (dataLeft > 0)
				{
					bytecount = send(csock, bitstream, dataLeft, 0);
					if (bytecount == SOCKET_ERROR)
					{
						fprintf(stderr, "Error sending JPEG bitstream : %d\n", WSAGetLastError());
						exit(-1);
					}
					bitstream += bytecount;
					dataLeft -= bytecount;
				}

				// (3.1) Receive 1 unsigned long (value = 3) to send point cloud X bitstream
				if ((bytecount = recv(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error receiving Depth instruction: %d\n", WSAGetLastError());
					exit(-1);
				}
				if (buf == 9999)
					break;
				if (buf != 3)
				{
					fprintf(stderr, "Expect 1 (Depth) but received %d\n", buf);
					exit(-1);
				}

				// (4.1) Send 1 unsigned long on the size of the point cloud X bitstream
				buf = save.getPointCLoudX_StreamSize();
				if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error sending Depth size : %d\n", WSAGetLastError());
					exit(-1);
				}

				// (5.1) Send the point cloud X stream bitstream
				bitstream = save.getPointCLoudX_Stream();
				dataLeft = buf;
				while (dataLeft > 0)
				{
					bytecount = send(csock, bitstream, dataLeft, 0);
					if (bytecount == SOCKET_ERROR)
					{
						fprintf(stderr, "Error sending Depth bitstream : %d\n", WSAGetLastError());
						exit(-1);
					}
					bitstream += bytecount;
					dataLeft -= bytecount;
				}

				// (3.2) Receive 1 unsigned long (value = 4) to send Y point cloud bitstream
				if ((bytecount = recv(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error receiving Depth instruction: %d\n", WSAGetLastError());
					exit(-1);
				}
				if (buf == 9999)
					break;
				if (buf != 4)
				{
					fprintf(stderr, "Expect 1 (Depth) but received %d\n", buf);
					exit(-1);
				}

				// (4.2) Send 1 unsigned long on the size of the point cloud Y bitstream
				buf = save.getPointCLoudY_StreamSize();
				if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error sending Depth size : %d\n", WSAGetLastError());
					exit(-1);
				}

				// (5.2) Send the point cloud Y stream bitstream
				bitstream = save.getPointCLoudY_Stream();
				dataLeft = buf;
				while (dataLeft > 0)
				{
					bytecount = send(csock, bitstream, dataLeft, 0);
					if (bytecount == SOCKET_ERROR)
					{
						fprintf(stderr, "Error sending Depth bitstream : %d\n", WSAGetLastError());
						exit(-1);
					}
					bitstream += bytecount;
					dataLeft -= bytecount;
				}

				// (3.3) Receive 1 unsigned long (value = 5) to send Z point cloud bitstream
				if ((bytecount = recv(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error receiving Depth instruction: %d\n", WSAGetLastError());
					exit(-1);
				}
				if (buf == 9999)
					break;
				if (buf != 5)
				{
					fprintf(stderr, "Expect 1 (Depth) but received %d\n", buf);
					exit(-1);
				}

				// (4.3) Send 1 unsigned long on the size of the point cloud Z bitstream
				buf = save.getPointCLoudZ_StreamSize();
				if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error sending Depth size : %d\n", WSAGetLastError());
					exit(-1);
				}

				// (5.3) Send the point cloud Z stream bitstream
				bitstream = save.getPointCLoudZ_Stream();
				dataLeft = buf;
				while (dataLeft > 0)
				{
					bytecount = send(csock, bitstream, dataLeft, 0);
					if (bytecount == SOCKET_ERROR)
					{
						fprintf(stderr, "Error sending Depth bitstream : %d\n", WSAGetLastError());
						exit(-1);
					}
					bitstream += bytecount;
					dataLeft -= bytecount;
				}
			}

			else if (eye_rec[0] == 0 && eye_rec[1] == 1) // start the tracking proceduring
			{
				printf("tracking is on ...\n");
				save.sphereTracking();
				printf("tracking is done!\n");

				//(1)a send 1 unsigned long on the size of the path bitstream
				buf = save.getPathStreamSize();
				if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
				{
					fprintf(stderr, "Error sending Depth size : %d\n", WSAGetLastError());
					exit(-1);
				}

				/*(1)b send the path mat to the server*/
				char *bitstream = save.getPathStream();
				unsigned long dataLeft = buf;
				while (dataLeft > 0)
				{
					bytecount = send(csock, bitstream, dataLeft, 0);
					if (bytecount == SOCKET_ERROR)
					{
						fprintf(stderr, "Error sending Depth bitstream : %d\n", WSAGetLastError());
						exit(-1);
					}
					bitstream += bytecount;
					dataLeft -= bytecount;
				}

				save.resetTracking();
			}

			// (2) Send 1 unsigned long on the size of the JPEG bitstream
			buf = save.getJPEGSize();
			if ((bytecount = send(csock, (char *)&buf, sizeof(unsigned long), 0)) == SOCKET_ERROR)
			{
				fprintf(stderr, "Error sending JPEG size : %d\n", WSAGetLastError());
				exit(-1);
			}

			//save.Stop();
			//if (save.count > 300)
			//{
			//	//save.sphereTracking(sphere_center_total,radius_total);
			//	break;
			//}
		}
		char c = cv::waitKey(5);
		if (c == 27)
			break;
	}

	save.Stop();

	//DisplayImages();
	// Initialize socket and setup connection

	return 0;
}