#ifndef CALIBRATIONHANDLER_H_
#define CALIBRATIONHANDLER_H_

#include <vector>
#include "sphereTracking.h"

class CalibrationHandler : public std::enable_shared_from_this<CalibrationHandler>
{
public:
	CalibrationHandler(int number_of_cameras, int final_number);
	~CalibrationHandler();

	struct coordinate
	{
		int idx;
		float x;
		float y;
		float z;
	};

	struct path
	{
		std::vector<coordinate> coords;
	};

	void StartProcessing();
	void ImageLoadingCompleteMessage(int clientID);
	void StoreClientInfo(SphereTracking *camera);

	cv::Mat GetColorProjectMatrix() const
	{
		return _color_projection;
	}

	cv::Mat GetInverseColorProjectMatrix() const
	{
		return _color_inverse;
	}

	std::vector<cv::Mat> GetRotationMatrix() const
	{
		return _rotation;
	}

	std::vector<cv::Mat> GetTranslationMatrix() const
	{
		return _translation;
	}

	/*For extrinsic calibration use*/
	std::vector<SphereTracking> cameras;
	std::shared_ptr<CalibrationHandler> get_shared_this() { return shared_from_this(); }
	std::vector<int> cam_ref_index;

private:
	void CalculateCameraExtrinsics();
	void LoadPath();
	void ComputeExtrinsics(int idx_1, int idx_2);
	void CorrespondenceBuild(int idx_1, int idx_2);

	int _number_of_cameras{0};
	int _final_number{0};
	std::mutex _mutex_1;
	std::mutex _mutex_2;

	cv::Mat _color_projection;
	cv::Mat _color_inverse;

	/*For extrinsic calibration use*/
	std::vector<cv::Mat> _path_mats; //the path list received from each client
	std::vector<path> _clients; //store the noisy path including those z = -1000
	path _proxy_clients[2]; // this is for pair-wise use that make sure the each point in the client cooresponds to the same point in the other proxy_client
	path _valid_clients[2];	// the difference between this one and clients is that it only keep those points whose z values are positive
	std::vector<cv::Mat> _rotation;
	std::vector<cv::Mat> _translation;
};

#endif
