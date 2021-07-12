#include <iostream>
#include <fstream>
#include "calibrationHandler.h"
#include "sphereTracking.h"

SphereTracking::~SphereTracking()
{
	for (auto rgb_l_img : rgb_l_imgs)
	{
		rgb_l_img.release();
	}
	for (auto dep_l_img : dep_l_imgs)
	{
		dep_l_img.release();
	}

	_color_inverse_projection.release();
	_geometry_3D.release();
	_l_img.release(); _r_img.release(); _l_thr.release(); _r_thr.release();
	_sphere_center.release();
	_pts_mat.release();
	_path_mat_sphere.release();
}

void SphereTracking::LoadImage(std::shared_ptr<CalibrationHandler> sharedCalibrationHandler)
{
	char filename[200];
	for (int i = 0; i <= _final_num; i++)
	{
		// Load rgb images
		sprintf(filename, "../src/sensors/Kinect_%d/kinect_%d_rgb_%02d.jpg", _client_id, 1, i);
		rgb_l_imgs.emplace_back(cv::imread(filename));

		// Load depth images
		sprintf(filename, "../src/sensors/Kinect_%d/kinect_%d_dep_%02d.dat", _client_id, 1, i);
		FILE *fp = fopen(filename, "rb");
		dep_l_imgs.emplace_back(cv::Mat::zeros(_height, _width, CV_16UC1));
		fread(dep_l_imgs[i].data, 2, _height * _width, fp);
		fclose(fp);

		if (i % 10 == 0)
			printf("load: %02d\n", i);
	}

	sharedCalibrationHandler->ImageLoadingCompleteMessage(this->_client_id);
	SphereCenterDetection();

	// Store client information
	sharedCalibrationHandler->StoreClientInfo(std::move(this));
}

void SphereTracking::SphereCenterDetection()
{
	_geometry_3D.create(_height * _width, 4, CV_32F);
	/*File used for storing sphere path*/
	char path_file[200];
	sprintf(path_file, "../src/tracking/path_%02d.txt", _client_id);
	std::ofstream out(path_file);

	char radius_file_test[200];
	sprintf(radius_file_test, "../src/tracking/radius_%02d.txt", _client_id);
	std::ofstream out_test(radius_file_test);
	std::vector<float> radius_list(rgb_l_imgs.size(), 0.0f);

	float total_radius{0.0f};
	int total_num{0};
	_path_mat_sphere = cv::Mat::zeros(rgb_l_imgs.size(), 3, CV_16UC1);

	for (int index = 0; index < _final_num; index++)
	{
		/*Detecting the ball region*/
		RegionDetection(rgb_l_imgs[index], index);

		/*Compute the 3D points of the image pairs*/
		Project3D2D(rgb_l_imgs[index], dep_l_imgs[index], 0);

		/*Compute and store the corresponding 3D points of each detected blob*/
		Compute3Dpoints(index);

		/*Do the denoising and sphere fitting, find an approximately optimal sphere center and radius*/
		SphereFitting(index);

		/*store current center to file*/
		if (_sphere_sign == 1)
		{
			out << index;
			out << " ";
			out << _sphere_center.ptr<float>(0)[0];
			out << " ";
			out << _sphere_center.ptr<float>(1)[0];
			out << " ";
			out << _sphere_center.ptr<float>(2)[0];
			out << "\n";

			_path_mat_sphere.ptr<short int>(index)[0] = (_sphere_center.ptr<float>(0)[0] > 0) ? (short int)(_sphere_center.ptr<float>(0)[0] + 0.5) : (short int)(_sphere_center.ptr<float>(0)[0] - 0.5);
			_path_mat_sphere.ptr<short int>(index)[1] = (_sphere_center.ptr<float>(1)[0] > 0) ? (short int)(_sphere_center.ptr<float>(1)[0] + 0.5) : (short int)(_sphere_center.ptr<float>(1)[0] - 0.5);
			_path_mat_sphere.ptr<short int>(index)[2] = (_sphere_center.ptr<float>(2)[0] > 0) ? (short int)(_sphere_center.ptr<float>(2)[0] + 0.5) : (short int)(_sphere_center.ptr<float>(2)[0] - 0.5);

			out_test << _sphere_radius;
			out_test << "\n";

			total_radius += _sphere_radius;
			radius_list[total_num] = _sphere_radius;
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

			_path_mat_sphere.ptr<short int>(index)[0] = 0.0f;
			_path_mat_sphere.ptr<short int>(index)[1] = 0.0f;
			_path_mat_sphere.ptr<short int>(index)[2] = -1000.0f;

			out_test << "-------";
			out_test << "\n";
		}

		_r_blob.clear();
		_l_blob.clear();
	}

	/*close the output file*/
	out.close();
	out_test.close();
}

/*This function is used to detect the specified color region*/
void SphereTracking::RegionDetection(cv::Mat &left, int i)
{
	cv::Mat l_hsv;
	cv::cvtColor(left, l_hsv, cv::COLOR_BGR2HSV);

	/*store the result after applying threshold*/
	/*cv::inRange(l_hsv, cv::Scalar(10, 160, 150), cv::Scalar(80, 255, 255), l_thr);*/
	cv::inRange(l_hsv, cv::Scalar(10, 160, 60), cv::Scalar(80, 255, 255), _l_thr);

	//char filename[200];
	//sprintf(filename, "../src/tracking/initial_%d_%03d.jpg", CLIENTID, i);
	//cv::imwrite(filename, l_thr);

	/*Find the corresponding 2D point blob on both images*/
	FindSphereBlobs(i);

	if (i % 10 == 0)
		printf("%02d region detection is done\n", i);
}

/*Find the corresponding 2D point on the sphere of both images*/
void SphereTracking::FindSphereBlobs(int index)
{
	/*This part is to find connecting parts on the left image*/
	std::vector<std::vector<cv::Point2i>> blobs;
	FindBlobs(_l_thr, blobs);

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
		_l_blob = blobs[max_idx];
	}

	/********************* For testing purpose ***************************************/

	/*Generate slide window to go through all the pixels*/
	cv::Mat l_thr_3c, r_thr_3c; /*3-channel version of l_thr, r_thr*/
	l_thr_3c.create(_l_thr.rows, _l_thr.cols, CV_8UC3);
	rgb_l_imgs[index].copyTo(l_thr_3c);

	for (int i = 0; i < (int)_l_blob.size(); i++)
	{
		int x = _l_blob[i].x;
		int y = _l_blob[i].y;

		l_thr_3c.ptr<uchar>(y)[3 * x] = 255;
		l_thr_3c.ptr<uchar>(y)[3 * x + 1] = 0;
		l_thr_3c.ptr<uchar>(y)[3 * x + 2] = 0;
	}

	/*Save the images to file*/
	{
		char filename[200];
		sprintf(filename, "../src/tracking/Kinect_%d/img_%d_%03d.jpg", _client_id, _client_id, index);
		cv::imwrite(filename, l_thr_3c);
	}
}

void SphereTracking::FindBlobs(const cv::Mat &binary, std::vector<std::vector<cv::Point2i>> &blobs)
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
						continue;
					blob.emplace_back(cv::Point2i(j, i));
				}
			}

			blobs.emplace_back(blob);
			label_count++;
		}
	}
}

/*Generate 3D point cloud from color image and depth image*/
void SphereTracking::Project3D2D(cv::Mat color, cv::Mat depth, int cam_idx)
{
	InverseProject(color, depth);
}

void SphereTracking::InverseProject(cv::Mat color, cv::Mat depth)
{
	for (int y = 0; y < depth.rows; y++)
	{
		short int *ptrDepth = depth.ptr<short int>(y);
		for (int x = 0; x < depth.cols; x++)
		{
			int i = y * _width + x;
			if (ptrDepth[x] == 0)
			{
				float *ptr_3d = _geometry_3D.ptr<float>(i);

				ptr_3d[0] = 0.0f;
				ptr_3d[1] = 0.0f;
				ptr_3d[2] = 0.0f;
				ptr_3d[3] = 1.0f;
			}
			else
			{
				float *ptr_3d = _geometry_3D.ptr<float>(i);

				ptr_3d[0] = (float)ptrDepth[x] * (_color_inverse_projection.ptr<float>(0)[0] * x + _color_inverse_projection.ptr<float>(0)[1] * x + _color_inverse_projection.ptr<float>(0)[2]);
				ptr_3d[1] = (float)ptrDepth[x] * (_color_inverse_projection.ptr<float>(1)[0] * x + _color_inverse_projection.ptr<float>(1)[1] * y + _color_inverse_projection.ptr<float>(1)[2]);
				ptr_3d[2] = (float)ptrDepth[x];
				ptr_3d[3] = 1.0f;
			}
		}
	}
}

void SphereTracking::Compute3Dpoints(int index)
{
	/*At the begining, clear the stored points*/
	_l_pts.clear();
	_sphere_sign = 0; // in the begining set the sphere sign as 0;
	_arr_idx.clear();
	_sphere_radius = 0; //as I use the radius as condition for ransac purpose, so set it to zero for each new frame
	_final_pts.clear();

	/*compute the left image*/
	char filename[200];
	sprintf(filename, "../src/tracking/Kinect_%d/kinect_%d_%02d.txt", _client_id, _client_id, index);
	std::ofstream out(filename);
	for (int i = 0; i < (int)_l_blob.size(); i++)
	{
		int x_2d = _l_blob[i].x;
		int y_2d = _l_blob[i].y;

		int pt_index = y_2d * _width + x_2d;
		cv::Point3f pt;

		pt.x = _geometry_3D.ptr<float>(pt_index)[0] / _geometry_3D.ptr<float>(pt_index)[3];
		pt.y = _geometry_3D.ptr<float>(pt_index)[1] / _geometry_3D.ptr<float>(pt_index)[3];
		pt.z = _geometry_3D.ptr<float>(pt_index)[2] / _geometry_3D.ptr<float>(pt_index)[3];

		/*check whether the depth is measurable*/
		if (pt.z != 0)
		{
			if (pt.z < 4500) // maximum value for depth
			{
				//std::cout << pt.z << std::endl;
				_l_pts.emplace_back(pt);

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

void SphereTracking::SphereFitting(int index)
{
	/* initialize random seed: */
	int total = _l_pts.size();

	if (total > _pt_rand)
	{
		_r_pts = _l_pts;
		int random = 0;
		while (random < _ransac_max && _sphere_sign == 0)
		{
			std::random_shuffle(_r_pts.begin(), _r_pts.end());

			std::vector<cv::Point3f>::const_iterator first = _r_pts.begin();
			std::vector<cv::Point3f>::const_iterator last = _r_pts.begin() + _pt_rand;
			std::vector<cv::Point3f> p_pts(first, last);

			/*Compute the sphere centers*/
			ComputeSpehereCenter(p_pts);

			/*Check whether a possible center is found*/
			if (_sphere_radius > (_estimated_radius - 20) && _sphere_radius < (_estimated_radius + 20))
			//if(sphere_radius > (_estimated_radius - 10)  && sphere_radius < (_estimated_radius + 10))
			{
				/*further refine the sphere center and radius by encompassing more points*/
				SphereRefine(index);
			}
			random++;
		}
	}
	else
	{
		_sphere_center = cv::Mat::zeros(3, 1, CV_32FC1);
	}
}

/*Compute the sphere center based on the point cloud*/
void SphereTracking::ComputeSpehereCenter(std::vector<cv::Point3f> &cur_pts)
{
	/*change vector into matrix*/
	_pts_mat = cv::Mat::zeros(cur_pts.size(), 3, CV_32FC1);
	for (int row = 0; row < _pts_mat.rows; row++)
	{
		_pts_mat.ptr<float>(row)[0] = cur_pts[row].x;
		_pts_mat.ptr<float>(row)[1] = cur_pts[row].y;
		_pts_mat.ptr<float>(row)[2] = cur_pts[row].z;
	}
	cv::Mat col_1, col_2, col_3; //store each column of the point cloud matrix

	_pts_mat(cv::Range(0, _pts_mat.rows), cv::Range(0, 1)).copyTo(col_1);
	_pts_mat(cv::Range(0, _pts_mat.rows), cv::Range(1, 2)).copyTo(col_2);
	_pts_mat(cv::Range(0, _pts_mat.rows), cv::Range(2, 3)).copyTo(col_3);

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

	/*Copy the temporary data to global*/
	center.copyTo(_sphere_center);
	_sphere_radius = radius;

	//printf("[%f, %f, %f]\n", center.ptr<float>(0)[0], center.ptr<float>(1)[0], center.ptr<float>(2)[0]);
	//printf("radius: %.5f\n", radius);
}

/*Based on the output of sphereFitting(), further refine the center by considering other pixels */
void SphereTracking::SphereRefine(int index)
{
	/*Compute the distance between the estimated center and all the points*/
	cv::Mat all_pts_mat(_l_pts.size(), 3, CV_32FC1);

	for (int i = 0; i < all_pts_mat.rows; i++)
	{
		all_pts_mat.ptr<float>(i)[0] = _l_pts[i].x;
		all_pts_mat.ptr<float>(i)[1] = _l_pts[i].y;
		all_pts_mat.ptr<float>(i)[2] = _l_pts[i].z;
	}

	cv::Mat center_tran, center_mat;
	cv::transpose(_sphere_center, center_tran);
	cv::repeat(center_tran, all_pts_mat.rows, 1, center_mat);

	cv::Mat pts_cen = all_pts_mat - center_mat;
	cv::Mat pts_cen_sqr = pts_cen.mul(pts_cen, 1.0);

	cv::Mat dis_mat;
	cv::reduce(pts_cen_sqr, dis_mat, 1, cv::REDUCE_SUM);

	/*Get the final point set*/
	_final_pts.clear();
	for (int i = 0; i < dis_mat.rows; i++)
	{
		if (dis_mat.ptr<float>(i)[0] < _radius_max_range && dis_mat.ptr<float>(i)[0] > _radius_min_range)
		{
			_final_pts.emplace_back(_l_pts[i]);
		}
	}

	/*Check the number of satisfactory points*/
	double ratio = (double)_final_pts.size() / (double)_l_pts.size();

	//printf("************************** good(%d) total(%d) | ratio(%.2f)\n", final_pts.size(), l_pts.size(), ratio);
	// printf("[%d] rough radius: %.2f\n", index, _sphere_radius);

	if (ratio > 0.8)
	{
		/*set the sign to 1 to quit the loop*/
		_sphere_sign = 1;

		/*Further refine the radius and center position*/
		ComputeSpehereCenter(_final_pts);
	}
}