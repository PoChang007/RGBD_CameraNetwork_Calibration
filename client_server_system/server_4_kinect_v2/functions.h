// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

void setX_axis(float angle);
void setY_axis(float angle);

std::string to_string(int a)
{
	std::ostringstream os;
	os << a;
	return os.str();
}

char *to_char(string s)
{
	char *a = new char[s.size() + 1];
	a[s.size()] = 0;
	memcpy(a, s.c_str(), s.size());
	return a;
}

double to_double(std::string const &str)
{
	std::istringstream ss(str);

	double d;
	ss >> d;

	/* eat up trailing whitespace if there was a double read, and ensure
	* there is no character left. the eof bit is set in the case that
	* `std::ws` tried to read beyond the stream. */
	if (!(ss && (ss >> std::ws).eof()))
		exit(1);

	return d;
}

void printMatrix(cv::Mat mat)
{
	/* If the matrix is CV_32F (float) type */
	if (mat.depth() == CV_32F)
	{
		for (int i = 0; i < mat.rows; i++)
		{
			float *ptr = mat.ptr<float>(i);
			for (int j = 0; j < mat.cols; j++)
			{
				printf("%f, ", ptr[j]);
			}
			printf("\n");
		}
	}

	/*If the matrix is CV_16U (double) type*/
	if (mat.depth() == CV_16U)
	{
		for (int i = 0; i < mat.rows; i++)
		{
			short int *ptr = mat.ptr<short int>(i);
			for (int j = 0; j < mat.cols; j++)
			{
				printf("%d, ", ptr[j]);
			}
			printf("\n");
		}
	}

	/*If the matrix is CV_64F (double) type*/
	if (mat.depth() == CV_8U)
	{
		for (int i = 0; i < mat.rows; i++)
		{
			char *ptr = mat.ptr<char>(i);
			for (int j = 0; j < mat.cols; j++)
			{
				printf("%d, ", ptr[j]);
			}
			printf("\n");
		}
	}
}

/*Load extrincsics from files*/
void loadExtrinsics()
{
	char filename_rotation_21[200];
	char filename_translate_21[200];
	char filename_rotation_31[200];
	char filename_translate_31[200];
	char filename_rotation_41[200];
	char filename_translate_41[200];

	sprintf(filename_rotation_21, "rotation_2_1.txt");
	sprintf(filename_translate_21, "translation_2_1.txt");
	sprintf(filename_rotation_31, "rotation_3_1.txt");
	sprintf(filename_translate_31, "translation_3_1.txt");
	sprintf(filename_rotation_41, "rotation_4_1.txt");
	sprintf(filename_translate_41, "translation_4_1.txt");

	ifstream in_21_r(filename_rotation_21);
	ifstream in_21_t(filename_translate_21);
	ifstream in_31_r(filename_rotation_31);
	ifstream in_31_t(filename_translate_31);
	ifstream in_41_r(filename_rotation_41);
	ifstream in_41_t(filename_translate_41);

	string word;

	/*Load rotation and translation between kinect 2 and 1*/
	int i = 0;
	while (in_21_r >> word)
	{
		R21[i][0] = to_double(word);

		in_21_r >> word;
		R21[i][1] = to_double(word);

		in_21_r >> word;
		R21[i][2] = to_double(word);

		i++;
	}

	i = 0;
	while (in_21_t >> word)
	{
		T21[i] = to_double(word);
		i++;
	}

	/*Load rotation and translation between kinect 3 and 1*/
	i = 0;
	while (in_31_r >> word)
	{
		R31[i][0] = to_double(word);

		in_31_r >> word;
		R31[i][1] = to_double(word);

		in_31_r >> word;
		R31[i][2] = to_double(word);

		i++;
	}

	i = 0;
	while (in_31_t >> word)
	{
		T31[i] = to_double(word);
		i++;
	}

	/*Load rotation and translation between kinect 4 and 1*/
	i = 0;
	while (in_41_r >> word)
	{
		R41[i][0] = to_double(word);

		in_41_r >> word;
		R41[i][1] = to_double(word);

		in_41_r >> word;
		R41[i][2] = to_double(word);

		i++;
	}

	i = 0;
	while (in_41_t >> word)
	{
		T41[i] = to_double(word);
		i++;
	}

	printf("R: (%.5f, %.5f, %.5f), (%.5f, %.5f, %.5f), (%.5f, %.5f, %.5f)\n", R21[0][0], R21[0][1], R21[0][2],
		   R21[1][0], R21[1][1], R21[1][2], R21[2][0], R21[2][1], R21[2][2]);

	printf("T: (%.5f, %.5f, %.5f)\n", T21[0], T21[1], T21[2]);
}

/*Initialize function*/
void initialize()
{
	/*Check whether to load extrinsics*/
	if (kinect_1.calib_conn[2] == 1)
	{
		loadExtrinsics();
	}
}

/*for each round, renew the image*/
void renewImage()
{
	//virimg = cv::Mat::zeros(HEIGHT1, WIDTH1, CV_8UC3);
	//z_buffer = cv::Mat::zeros(HEIGHT1, WIDTH1, CV_32F);

	//for (int i = 0; i < Client_Total; i++){
	//	RGBImage[i] = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
	//	PointCloudX[i] = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
	//	PointCloudY[i] = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
	//	PointCloudZ[i] = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
	//}
}

void inverseProject(cv::Mat color, cv::Mat depth, cv::Mat &geometry_3D)
{
	for (int y = 0; y < depth.rows; y++)
	{
		short int *ptrDepth = depth.ptr<short int>(y);
		for (int x = 0; x < depth.cols; x++)
		{
			int i = y * WIDTH + x;
			if (ptrDepth[x] == 0)
			{
				float *ptrScalar = depth_pixels_scalar_tran.ptr<float>(i);
				float *ptr_3d = geometry_3D.ptr<float>(i);

				ptr_3d[0] = 0.0f;
				ptr_3d[1] = 0.0f;
				ptr_3d[2] = 0.0f;
				ptr_3d[3] = 1.0f;
			}
			else
			{
				float *ptrScalar = depth_pixels_scalar_tran.ptr<float>(i);
				float *ptr_3d = geometry_3D.ptr<float>(i);

				ptr_3d[0] = (float)ptrScalar[0] * (float)ptrDepth[x];
				ptr_3d[1] = (float)ptrScalar[1] * (float)ptrDepth[x];
				ptr_3d[2] = (float)ptrDepth[x];
				ptr_3d[3] = 1.0f;
			}
		}
	}
}

/*get the center trajectories from 4 kinects*/
void loadPath()
{
	int idx = 0;
	while (!(path_mat[0].ptr<short int>(idx)[2] == 0))
	{

		for (int i = 0; i < Client_Total; i++)
		{
			coordinate coord;
			coord.idx = idx;
			coord.x = (double)path_mat[i].ptr<short int>(idx)[0];
			coord.y = (double)path_mat[i].ptr<short int>(idx)[1];
			coord.z = (double)path_mat[i].ptr<short int>(idx)[2];

			clients[i].coords.push_back(coord);
		}

		idx++;
	}

	/*test*/
	/*for(int i = 0; i < (int)clients[0].coords.size(); i++)
	{
	printf("[%d] (%.f, %.f, %.f)|(%.f, %.f, %.f)|(%.f, %.f, %.f)\n", i,
	clients[0].coords[i].x, clients[0].coords[i].y, clients[0].coords[i].z,
	clients[1].coords[i].x, clients[1].coords[i].y, clients[1].coords[i].z,
	clients[2].coords[i].x, clients[2].coords[i].y, clients[2].coords[i].z);
	}*/

	// File used for storing sphere path
	for (int i = 0; i < Client_Total; i++)
	{
		char path_file[200];
		sprintf(path_file, "path_%02d.txt", i + 1);
		ofstream out(path_file);
		int Total_Points = path_mat[0].rows;

		for (int j = 0; j < Total_Points; j++)
		{
			if (!(path_mat[0].ptr<short int>(j)[2] == 0))
			{
				out << j;
				out << " ";
				out << (double)path_mat[i].ptr<short int>(j)[0];
				out << " ";
				out << (double)path_mat[i].ptr<short int>(j)[1];
				out << " ";
				out << (double)path_mat[i].ptr<short int>(j)[2];
				out << "\n";
			}
		}

		// close the output file
		out.close();
	}
}

/*Re-organize the two clients to make sure they have the same number and indices of points*/
void correspondenceBuild(int idx_1, int idx_2)
{
	/*initially clear the proxy clients*/
	proxy_clients[0].coords.clear();
	proxy_clients[1].coords.clear();
	valid_clients[0].coords.clear();
	valid_clients[1].coords.clear();

	/*Traverse the first client points, it is added to proxy_client only when its index exists in the other client*/
	for (int i = 0; i < (int)clients[idx_1].coords.size(); i++)
	{
		if (clients[idx_1].coords[i].z > 0 && clients[idx_2].coords[i].z > 0)
		{
			proxy_clients[0].coords.push_back(clients[idx_1].coords[i]);
			proxy_clients[1].coords.push_back(clients[idx_2].coords[i]);
		}
	}

	/*Get valid point clients*/
	for (int i = 0; i < (int)clients[idx_1].coords.size(); i++)
	{
		if (clients[idx_1].coords[i].z > 0)
		{
			valid_clients[0].coords.push_back(clients[idx_1].coords[i]);
		}

		if (clients[idx_2].coords[i].z > 0)
		{
			valid_clients[1].coords.push_back(clients[idx_2].coords[i]);
		}
	}

	printf("clients[0] %d | clients[1] %d\n", clients[0].coords.size(), clients[1].coords.size());
	printf("proxy_clients[0] %d | proxy_clients[1] %d\n", proxy_clients[0].coords.size(), proxy_clients[1].coords.size());
	printf("valid_clients[0] %d | valid_clients[1] %d\n", valid_clients[0].coords.size(), valid_clients[1].coords.size());
}

/*Compute the extrinsic parameters between two kinects*/
void computeExtrinsics(int idx_1, int idx_2)
{
	/*remove not-captured frames*/
	correspondenceBuild(idx_1, idx_2);

	/*Initial the matrix for the two path from two kinects*/
	cv::Mat mat_A = cv::Mat::zeros((int)proxy_clients[0].coords.size(), 1, CV_32FC3);
	cv::Mat mat_B = cv::Mat::zeros((int)proxy_clients[1].coords.size(), 1, CV_32FC3);
	cv::Mat mat_A_3 = cv::Mat::zeros((int)proxy_clients[0].coords.size(), 3, CV_32FC1); //3 columns version of mat_A
	cv::Mat mat_B_3 = cv::Mat::zeros((int)proxy_clients[1].coords.size(), 3, CV_32FC1); //3 columns version of mat_B

	/*Assign coordinates for the two matrices*/
	for (int i = 0; i < (int)proxy_clients[0].coords.size(); i++) //matrix A
	{
		mat_A.ptr<float>(i)[0] = proxy_clients[0].coords[i].x;
		mat_A.ptr<float>(i)[1] = proxy_clients[0].coords[i].y;
		mat_A.ptr<float>(i)[2] = proxy_clients[0].coords[i].z;

		mat_A_3.ptr<float>(i)[0] = proxy_clients[0].coords[i].x;
		mat_A_3.ptr<float>(i)[1] = proxy_clients[0].coords[i].y;
		mat_A_3.ptr<float>(i)[2] = proxy_clients[0].coords[i].z;
	}

	for (int i = 0; i < (int)proxy_clients[1].coords.size(); i++) //matrix B
	{
		mat_B.ptr<float>(i)[0] = proxy_clients[1].coords[i].x;
		mat_B.ptr<float>(i)[1] = proxy_clients[1].coords[i].y;
		mat_B.ptr<float>(i)[2] = proxy_clients[1].coords[i].z;

		mat_B_3.ptr<float>(i)[0] = proxy_clients[1].coords[i].x;
		mat_B_3.ptr<float>(i)[1] = proxy_clients[1].coords[i].y;
		mat_B_3.ptr<float>(i)[2] = proxy_clients[1].coords[i].z;
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

	sprintf(filename_rotation, "rotation_%d_%d.txt", idx_1 + 1, idx_2 + 1);
	sprintf(filename_translate, "translation_%d_%d.txt", idx_1 + 1, idx_2 + 1);

	ofstream out_r(filename_rotation);
	ofstream out_t(filename_translate);

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
}