#include <iostream>
#include <fstream>
#include <future>
#include "calibrationHandler.h"

CalibrationHandler::CalibrationHandler(int number_of_cameras, int final_number)
{
	_number_of_cameras = number_of_cameras;
	_final_number = final_number;
	_color_projection = cv::Mat::zeros(3, 4, CV_32F);
	_color_inverse = cv::Mat::zeros(3, 3, CV_32F);

	/*Assign values to color_calib*/
	{
		/*Assign color calibration parameters*/
		float *ptr_calib = _color_projection.ptr<float>(0);
		ptr_calib[0] = 524.639247602080790f;
		ptr_calib[1] = 0.0f;
		ptr_calib[2] = 316.625776542517880f;
		ptr_calib[3] = 0.0f;

		ptr_calib = _color_projection.ptr<float>(1);
		ptr_calib[0] = 0.0f;
		ptr_calib[1] = 523.503043480237810f;
		ptr_calib[2] = 256.231848585108540f;
		ptr_calib[3] = 0.0f;

		ptr_calib = _color_projection.ptr<float>(2);
		ptr_calib[0] = 0.0f;
		ptr_calib[1] = 0.0f;
		ptr_calib[2] = 1.0f;
		ptr_calib[3] = 0.0f;

		/*Assign inverse Color projection parameters*/
		float *ptr_inv = _color_inverse.ptr<float>(0);
		ptr_inv[0] = 1.0f / _color_projection.ptr<float>(0)[0];
		ptr_inv[1] = 0.0f;
		ptr_inv[2] = -_color_projection.ptr<float>(0)[2] / _color_projection.ptr<float>(0)[0];

		ptr_inv = _color_inverse.ptr<float>(1);
		ptr_inv[0] = 0.0f;
		ptr_inv[1] = 1.0f / _color_projection.ptr<float>(1)[1];
		ptr_inv[2] = -_color_projection.ptr<float>(1)[2] / _color_projection.ptr<float>(1)[1];

		ptr_inv = _color_inverse.ptr<float>(2);
		ptr_inv[0] = 0.0f;
		ptr_inv[1] = 0.0f;
		ptr_inv[2] = 1.0f;
	}
	
	cam_ref_index = std::vector<int>(_number_of_cameras);
	_clients = std::vector<path>(_number_of_cameras);
	_path_mats = std::vector<cv::Mat>(_number_of_cameras);
	_rotation = std::vector<cv::Mat>(_number_of_cameras);
	_translation = std::vector<cv::Mat>(_number_of_cameras);

	// reference camera's extrinsics
	_rotation[0] = cv::Mat::eye(3, 3, CV_32F);
	_translation[0] = cv::Mat::zeros(3, 1, CV_32F);
}

CalibrationHandler::~CalibrationHandler()
{
	_color_projection.release();
	_color_inverse.release();

	for (auto _path_mat : _path_mats)
	{
		_path_mat.release();
	}
}

void CalibrationHandler::StartProcessing()
{
	if (_number_of_cameras <= 1)
	{
		printf("no camera calibration will be involved");
		return;
	}

	std::vector<std::future<void>> futures;
	for (int i = 1; i <= _number_of_cameras; i++)
	{
		std::unique_ptr<SphereTracking> cameraSphereTracking = std::make_unique<SphereTracking>(_color_inverse, std::move(i), std::move(_final_number));
		futures.emplace_back(std::async(std::launch::async, &SphereTracking::LoadImage, std::move(cameraSphereTracking), get_shared_this()));
	}

	std::for_each(futures.begin(), futures.end(), [](std::future<void> &ftr)
				  { ftr.wait(); });

	CalculateCameraExtrinsics();
}

void CalibrationHandler::ImageLoadingCompleteMessage(int clientID)
{
	std::lock_guard<std::mutex> lock(_mutex_1);
	std::cout << "Client " << clientID << " has finished image loading" << std::endl;
}

void CalibrationHandler::StoreClientInfo(SphereTracking *camera)
{
	std::lock_guard<std::mutex> lock(_mutex_2);
	cameras.emplace_back(*(std::move(camera)));
	std::cout << "Client " << camera->GetClientID() << " has finished the sphere center detection task" << std::endl;
}

void CalibrationHandler::CalculateCameraExtrinsics()
{
	int idx = 0;
	for (int i = 0; i < cameras.size(); i++)
	{
		for (int j = 1; j < (cameras.size() + 1); j++)
		{
			if (cameras[i].GetClientID() == j)
			{
				_path_mats[(j - 1)] = cameras[i].GetPathMatSphere();
				cam_ref_index[(j - 1)] = i;
				break;
			}
		}
	}
	LoadPath();

	// Compute extrinsics
	for (int i = 1; i < cameras.size(); i++)
	{
		ComputeExtrinsics(i, 0);
	}
}

void CalibrationHandler::LoadPath()
{
	int idx = 0;
	while (!(_path_mats[0].ptr<short int>(idx)[2] == 0))
	{
		for (int i = 0; i < _path_mats.size(); i++)
		{
			coordinate coord;
			coord.idx = idx;
			coord.x = (double)_path_mats[i].ptr<short int>(idx)[0];
			coord.y = (double)_path_mats[i].ptr<short int>(idx)[1];
			coord.z = (double)_path_mats[i].ptr<short int>(idx)[2];
			_clients[i].coords.emplace_back(coord);
		}
		idx++;
		if (idx == (_path_mats[0].size().height - 1))
			break;
	}

	/*test*/
	// for (int i = 0; i < (int)clients[0].coords.size(); i++)
	// {
	// 	printf("[%d] (%.f, %.f, %.f)|(%.f, %.f, %.f)|(%.f, %.f, %.f)\n", i,
	// 		   clients[0].coords[i].x, clients[0].coords[i].y, clients[0].coords[i].z,
	// 		   clients[1].coords[i].x, clients[1].coords[i].y, clients[1].coords[i].z,
	// 		   clients[2].coords[i].x, clients[2].coords[i].y, clients[2].coords[i].z);
	// }
}

/*Compute the extrinsic parameters between two cameras*/
void CalibrationHandler::ComputeExtrinsics(int idx_1, int idx_2)
{
	/*remove not-captured frames*/
	CorrespondenceBuild(idx_1, idx_2);

	/*Initial the matrix for the two path from two kinects*/
	cv::Mat mat_A = cv::Mat::zeros((int)_proxy_clients[0].coords.size(), 1, CV_32FC3);
	cv::Mat mat_B = cv::Mat::zeros((int)_proxy_clients[1].coords.size(), 1, CV_32FC3);
	cv::Mat mat_A_3 = cv::Mat::zeros((int)_proxy_clients[0].coords.size(), 3, CV_32FC1); //3 columns version of mat_A
	cv::Mat mat_B_3 = cv::Mat::zeros((int)_proxy_clients[1].coords.size(), 3, CV_32FC1); //3 columns version of mat_B

	/*Assign coordinates for the two matrices*/
	for (int i = 0; i < (int)_proxy_clients[0].coords.size(); i++) //matrix A
	{
		mat_A.ptr<float>(i)[0] = _proxy_clients[0].coords[i].x;
		mat_A.ptr<float>(i)[1] = _proxy_clients[0].coords[i].y;
		mat_A.ptr<float>(i)[2] = _proxy_clients[0].coords[i].z;

		mat_A_3.ptr<float>(i)[0] = _proxy_clients[0].coords[i].x;
		mat_A_3.ptr<float>(i)[1] = _proxy_clients[0].coords[i].y;
		mat_A_3.ptr<float>(i)[2] = _proxy_clients[0].coords[i].z;
	}

	for (int i = 0; i < (int)_proxy_clients[1].coords.size(); i++) //matrix B
	{
		mat_B.ptr<float>(i)[0] = _proxy_clients[1].coords[i].x;
		mat_B.ptr<float>(i)[1] = _proxy_clients[1].coords[i].y;
		mat_B.ptr<float>(i)[2] = _proxy_clients[1].coords[i].z;

		mat_B_3.ptr<float>(i)[0] = _proxy_clients[1].coords[i].x;
		mat_B_3.ptr<float>(i)[1] = _proxy_clients[1].coords[i].y;
		mat_B_3.ptr<float>(i)[2] = _proxy_clients[1].coords[i].z;
	}

	/*Implement the function from Matlab on computing rigid transformations*/
	cv::Mat centroid_A = cv::Mat(1, 3, CV_32FC1);
	cv::Mat centroid_B = cv::Mat(1, 3, CV_32FC1);

	cv::Scalar mean_A = cv::mean(mat_A);
	cv::Scalar mean_B = cv::mean(mat_B);

	for (int i = 0; i < 3; i++)
	{
		centroid_A.ptr<float>(0)[i] = mean_A(i);
		centroid_B.ptr<float>(0)[i] = mean_B(i);
	}

	cv::Mat centroid_A_mat, centroid_B_mat;
	cv::repeat(centroid_A, mat_A.rows, 1, centroid_A_mat);
	cv::repeat(centroid_B, mat_B.rows, 1, centroid_B_mat);

	cv::Mat H, A_cen_tran, B_cen, temp;
	temp = mat_A_3 - centroid_A_mat;
	cv::transpose(temp, A_cen_tran);

	B_cen = mat_B_3 - centroid_B_mat;

	H = A_cen_tran * B_cen;

	cv::Mat U, S, V;
	cv::SVD::compute(H, S, U, V);

	cv::Mat R, t;

	cv::Mat U_tran;
	cv::Mat V_tran;
	cv::transpose(V, V_tran);
	cv::transpose(U, U_tran);
	R = V_tran * U_tran;

	double det_r = cv::determinant(R);
	if (det_r < 0)
	{
		V_tran.ptr<float>(0)[2] = -V_tran.ptr<float>(0)[2];
		V_tran.ptr<float>(1)[2] = -V_tran.ptr<float>(1)[2];
		V_tran.ptr<float>(2)[2] = -V_tran.ptr<float>(2)[2];
		R = V_tran * U_tran;
	}

	cv::Mat cen_A_tran, cen_B_tran;
	cv::transpose(centroid_A, cen_A_tran);
	cv::transpose(centroid_B, cen_B_tran);

	t = cen_B_tran - R * cen_A_tran;

	/*******************	 Compute the Mean-squared-error		*******************/
	cv::Mat mat_A_transformed_proxy;

	cv::Mat mat_A_tran_proxy, mat_A_transformed_tran_proxy, t_tran_proxy, t_mat_proxy;
	cv::transpose(mat_A_3, mat_A_tran_proxy);
	cv::transpose(t, t_tran_proxy);
	cv::repeat(t, 1, mat_A.rows, t_mat_proxy);
	mat_A_transformed_proxy = R * mat_A_tran_proxy + t_mat_proxy;
	cv::transpose(mat_A_transformed_proxy, mat_A_transformed_tran_proxy);

	double sum_error = 0.0;
	for (int y = 0; y < mat_A_transformed_tran_proxy.rows; y++)
	{

		float x_diff = abs(mat_A_transformed_tran_proxy.ptr<float>(y)[0] - mat_B_3.ptr<float>(y)[0]);
		float y_diff = abs(mat_A_transformed_tran_proxy.ptr<float>(y)[1] - mat_B_3.ptr<float>(y)[1]);
		float z_diff = abs(mat_A_transformed_tran_proxy.ptr<float>(y)[2] - mat_B_3.ptr<float>(y)[2]);

		sum_error += x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;
	}

	double avg_sqr_error = sum_error / (double)mat_A_transformed_tran_proxy.rows;
	double avg_error = cv::sqrt(avg_sqr_error);
	printf("squared error: %.5f | average error: %.5f\n", avg_sqr_error, avg_error);
	/*************************** End of Mean-squared-error  ***********************/

	/***********************************************   Output the extrinsics	*****************************************/
	char filename_rotation[200];
	char filename_translate[200];

	sprintf(filename_rotation, "../src/rotation_%d_%d.txt", idx_1 + 1, idx_2 + 1);
	sprintf(filename_translate, "../src/translation_%d_%d.txt", idx_1 + 1, idx_2 + 1);

	std::ofstream out_r(filename_rotation);
	std::ofstream out_t(filename_translate);

	for (int i = 0; i < R.rows; i++)
	{
		out_r << R.ptr<float>(i)[0];
		out_r << " ";
		out_r << R.ptr<float>(i)[1];
		out_r << " ";
		out_r << R.ptr<float>(i)[2];
		out_r << "\n";
	}

	for (int i = 0; i < t.rows; i++)
	{
		out_t << t.ptr<float>(i)[0];
		out_t << "\n";
	}
	out_r.close();
	out_t.close();

	R.copyTo(_rotation[idx_1]);
	t.copyTo(_translation[idx_1]);
}

/*Re-organize the two clients to make sure they have the same number and indices of points*/
void CalibrationHandler::CorrespondenceBuild(int idx_1, int idx_2)
{
	/*initially clear the proxy clients*/
	_proxy_clients[0].coords.clear();
	_proxy_clients[1].coords.clear();
	_valid_clients[0].coords.clear();
	_valid_clients[1].coords.clear();

	/*Traverse the first client points, it is added to proxy_client only when its index exists in the other client*/
	for (int i = 0; i < (int)_clients[idx_1].coords.size(); i++)
	{
		if (_clients[idx_1].coords[i].z > 0 && _clients[idx_2].coords[i].z > 0)
		{
			_proxy_clients[0].coords.emplace_back(_clients[idx_1].coords[i]);
			_proxy_clients[1].coords.emplace_back(_clients[idx_2].coords[i]);
		}
	}

	/*Get valid point clients*/
	for (int i = 0; i < (int)_clients[idx_1].coords.size(); i++)
	{
		if (_clients[idx_1].coords[i].z > 0)
		{
			_valid_clients[0].coords.emplace_back(_clients[idx_1].coords[i]);
		}

		if (_clients[idx_2].coords[i].z > 0)
		{
			_valid_clients[1].coords.emplace_back(_clients[idx_2].coords[i]);
		}
	}

	printf("clients[0] %d | clients[1] %d\n", (int)_clients[0].coords.size(), (int)_clients[1].coords.size());
	printf("proxy_clients[0] %d | proxy_clients[1] %d\n", (int)_proxy_clients[0].coords.size(), (int)_proxy_clients[1].coords.size());
	printf("valid_clients[0] %d | valid_clients[1] %d\n", (int)_valid_clients[0].coords.size(), (int)_valid_clients[1].coords.size());
}