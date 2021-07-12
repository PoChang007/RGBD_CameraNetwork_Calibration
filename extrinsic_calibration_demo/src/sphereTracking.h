#ifndef SPHERETRACKING_H_
#define SPHERETRACKING_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CalibrationHandler;

class SphereTracking
{
public:
	SphereTracking(cv::Mat _color_inverse, int &&client_id, int &&final_number) : 
		_color_inverse_projection(_color_inverse), _client_id(client_id), _final_num(final_number){};
	~SphereTracking();

	void LoadImage(std::shared_ptr<CalibrationHandler> sharedCalibrationHandler);

	cv::Mat GetPathMatSphere() const
	{
		return _path_mat_sphere;
	}

	int GetClientID() const
	{
		return _client_id;
	}

	int GetImageHeight() const
	{
		return _height;
	}

	int GetImageWidth() const
	{
		return _width;
	}

	std::vector<cv::Mat> rgb_l_imgs;
	std::vector<cv::Mat> dep_l_imgs;

private:
	void SphereCenterDetection();
	void RegionDetection(cv::Mat &left, int i);
	void FindSphereBlobs(int index);
	void FindBlobs(const cv::Mat &binary, std::vector<std::vector<cv::Point2i>> &blobs);

	void Project3D2D(cv::Mat color, cv::Mat depth, int cam_idx);
	void InverseProject(cv::Mat color, cv::Mat depth);

	void Compute3Dpoints(int index);

	void ComputeSpehereCenter(std::vector<cv::Point3f> &cur_pts);
	void SphereFitting(int index);
	void SphereRefine(int index);

	cv::Mat _color_inverse_projection;
	cv::Mat _geometry_3D; // stores 3D points that after being transformed to world coordinate.

	cv::Mat _l_img, _r_img, _l_thr, _r_thr;

	/*store the conecting 2D point blobs */
	std::vector<cv::Point2i> _l_blob, _r_blob;

	/*store the 3D points of sphere, the noisy one*/
	std::vector<cv::Point3f> _l_pts, _r_pts; // r_pts - random re-shuffled points;

	int _sphere_sign{0};
	std::vector<int> _arr_idx; // the vector stores all the index values of point cloud

	/*Sphere fitting used*/
	cv::Mat _sphere_center;
	float _sphere_radius{0};

	/*store the de-noised 3D points from l_pts*/
	std::vector<cv::Point3f> _final_pts;

	int _pt_rand{50}; //the number used for RANSAC, which is a small proportion of pt_sum
	int _ransac_max{5000};
	float _estimated_radius{203.0f};
	float _radius_min_range = (_estimated_radius - 40) * (_estimated_radius - 40);
	float _radius_max_range = (_estimated_radius + 40) * (_estimated_radius + 40);

	/*store the 3D point cloud in matrix form (the same data as l_pts*/
	cv::Mat _pts_mat;

	/*The data store path file*/
	cv::Mat _path_mat_sphere;

	int _client_id;
	int _final_num;

	int _width{640};
	int _height{480};
};

#endif