#include "threeDViewer.h"

ThreeDViewer::ThreeDViewer(std::shared_ptr<CalibrationHandler> handler)
{
	_colorProjection = handler->GetColorProjectMatrix();
	_inverseColorProjection = handler->GetInverseColorProjectMatrix();
	_rows = handler->cameras[0].GetImageHeight();
	_cols = handler->cameras[0].GetImageWidth();
	_rotation = handler->GetRotationMatrix();
	_translation = handler->GetTranslationMatrix();

	// initialize point clouds
	for (int i = 0; i < handler->cameras.size(); i++)
	{
		clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZRGB>);
		clouds[i]->height = (uint32_t)_rows;
		clouds[i]->width = (uint32_t)_cols;
		clouds[i]->is_dense = false;
		clouds[i]->points.resize(clouds[i]->width * clouds[i]->height);
		_path_mat_sphere.emplace_back(handler->cameras[handler->cam_ref_index[i]].GetPathMatSphere());
	}
}

ThreeDViewer::~ThreeDViewer()
{
	_inverseColorProjection.release();
	for (auto rotation : _rotation)
	{
		rotation.release();
	}
	for (auto translation : _translation)
	{
		translation.release();
	}
}

void ThreeDViewer::SetUpPointClouds(std::shared_ptr<CalibrationHandler> &handler, const int &counter)
{
	for (int i = 0; i < clouds.size(); i++)
	{
		for (int u = 0; u < _rows; u++)
		{
			for (int v = 0; v < _cols; v++)
			{
				int index = u * _cols + v;
				if ((float)handler->cameras[handler->cam_ref_index[i]].dep_l_imgs[counter].ptr<short int>(u)[v] != 0)
				{
					float dx = (float)handler->cameras[handler->cam_ref_index[i]].dep_l_imgs[counter].ptr<short int>(u)[v] *
							   (_inverseColorProjection.ptr<float>(0)[0] * v + _inverseColorProjection.ptr<float>(0)[1] * u + _inverseColorProjection.ptr<float>(0)[2]);
					float dy = (float)handler->cameras[handler->cam_ref_index[i]].dep_l_imgs[counter].ptr<short int>(u)[v] *
							   (_inverseColorProjection.ptr<float>(1)[0] * v + _inverseColorProjection.ptr<float>(1)[1] * u + _inverseColorProjection.ptr<float>(1)[2]);
					float dz = (float)handler->cameras[handler->cam_ref_index[i]].dep_l_imgs[counter].ptr<short int>(u)[v];
					clouds[i]->points[index].x = (_rotation[i].ptr<float>(0)[0] * dx + _rotation[i].ptr<float>(0)[1] * dy + _rotation[i].ptr<float>(0)[2] * dz +
												  _translation[i].ptr<float>(0)[0]) *
												 _scale;
					clouds[i]->points[index].y = (_rotation[i].ptr<float>(1)[0] * dx + _rotation[i].ptr<float>(1)[1] * dy + _rotation[i].ptr<float>(1)[2] * dz +
												  _translation[i].ptr<float>(1)[0]) *
												 _scale;
					clouds[i]->points[index].z = (_rotation[i].ptr<float>(2)[0] * dx + _rotation[i].ptr<float>(2)[1] * dy + _rotation[i].ptr<float>(2)[2] * dz +
												  _translation[i].ptr<float>(2)[0]) *
												 _scale;

					std::uint8_t r((int)handler->cameras[handler->cam_ref_index[i]].rgb_l_imgs[counter].ptr<uchar>(u)[3 * v + 2]);
					std::uint8_t g((int)handler->cameras[handler->cam_ref_index[i]].rgb_l_imgs[counter].ptr<uchar>(u)[3 * v + 1]);
					std::uint8_t b((int)handler->cameras[handler->cam_ref_index[i]].rgb_l_imgs[counter].ptr<uchar>(u)[3 * v]);
					std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
										 static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
					clouds[i]->points[index].rgb = *reinterpret_cast<float *>(&rgb);
				}
				else
				{
					clouds[i]->points[index].x = 0;
					clouds[i]->points[index].y = 0;
					clouds[i]->points[index].z = 0;

					std::uint8_t r(0), g(0), b(0);
					std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
										 static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
					clouds[i]->points[index].rgb = *reinterpret_cast<float *>(&rgb);
				}
			}
		}
	}
}

void ThreeDViewer::DrawCameraFrustum(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
	int counter = 0;
	for (int i = 0; i < clouds.size(); i++)
	{
		auto *tran = _translation[i].ptr<float>(0);
		auto *rot_1 = _rotation[i].ptr<float>(0);
		auto *rot_2 = _rotation[i].ptr<float>(1);
		auto *rot_3 = _rotation[i].ptr<float>(2);
		auto cam_origin_point = pcl::PointXYZ(tran[0] * _scale, tran[1] * _scale, tran[2] * _scale);

		// initialize two points for line drawing
		auto point_1 = pcl::PointXYZ(cam_origin_point.x, cam_origin_point.y, cam_origin_point.z);
		auto point_2 = pcl::PointXYZ(0.0f, 0.0f, 0.0f);

		// draw coordinate axis
		// x axis
		auto axis_color = std::vector<float>{1.0f, 0.0f, 0.0f};
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[0] * 0.25f,
								cam_origin_point.y + rot_2[0] * 0.25f,
								cam_origin_point.z + rot_3[0] * 0.25f);
		viewer->addLine(point_1, point_2, axis_color[0], axis_color[1], axis_color[2], std::to_string(counter));
		counter++;

		// y axis
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[1] * 0.25f,
								cam_origin_point.y + rot_2[1] * 0.25f,
								cam_origin_point.z + rot_3[1] * 0.25f);
		viewer->addLine(point_1, point_2, axis_color[0], axis_color[1], axis_color[2], std::to_string(counter));
		counter++;

		// z axis
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.25f,
								cam_origin_point.y + rot_2[2] * 0.25f,
								cam_origin_point.z + rot_3[2] * 0.25f);
		viewer->addLine(point_1, point_2, axis_color[0], axis_color[1], axis_color[2], std::to_string(counter));
		counter++;

		// camera frustum
		auto frustum_color = std::vector<float>{1.0f, 1.0f, 0.0f};
		// upper right
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// upper left
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// lower left
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// lower right
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// upper right, upper left
		point_1 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// upper right, lower right
		point_1 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// upper left, lower left
		point_1 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f - rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f - rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f - rot_3[1] * 0.1f);
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;

		// lower left, lower right
		point_1 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f - rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f - rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f - rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		point_2 = pcl::PointXYZ(cam_origin_point.x + rot_1[2] * 0.1f + rot_1[0] * 0.1f + rot_1[1] * 0.1f,
								cam_origin_point.y + rot_2[2] * 0.1f + rot_2[0] * 0.1f + rot_2[1] * 0.1f,
								cam_origin_point.z + rot_3[2] * 0.2f + rot_3[0] * 0.1f + rot_3[1] * 0.1f);
		viewer->addLine(point_1, point_2, frustum_color[0], frustum_color[1], frustum_color[2], std::to_string(counter));
		counter++;
	}
}

void ThreeDViewer::DrawSphereCenterIn2DImage(std::shared_ptr<CalibrationHandler> &handler, const int &cam_index, const int &frameCounter)
{
	if (_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2] != -1000)
	{
		float u_coord = (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[0] * _colorProjection.ptr<float>(0)[0] +
						(float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[1] * _colorProjection.ptr<float>(0)[1] +
						(float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2] * _colorProjection.ptr<float>(0)[2];
		float v_coord = (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[0] * _colorProjection.ptr<float>(1)[0] +
						(float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[1] * _colorProjection.ptr<float>(1)[1] +
						(float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2] * _colorProjection.ptr<float>(1)[2];
		int u_normalized = (int)std::roundf(u_coord / (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2]);
		int v_normalized = (int)std::roundf(v_coord / (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2]);
		cv::circle(handler->cameras[handler->cam_ref_index[cam_index]].rgb_l_imgs[frameCounter], cv::Point(u_normalized, v_normalized), 5, cv::Scalar(200, 0, 0), cv::FILLED);
	}
}

void ThreeDViewer::DrawSphereIn3DSpace(pcl::visualization::PCLVisualizer::Ptr &viewer, const int &cam_index, const int &frameCounter, std::string &sphereName)
{
	if (frameCounter == 0)
	{
		if (cam_index == 0)
			viewer->addSphere(pcl::PointXYZ(_translation[cam_index].ptr<float>(0)[0] * _scale, _translation[cam_index].ptr<float>(1)[0] * _scale, _translation[cam_index].ptr<float>(2)[0] * _scale),
							  10 * _scale, cam_color_0[0], cam_color_0[1], cam_color_0[2], "origin_1");
		else if (cam_index == 1)
			viewer->addSphere(pcl::PointXYZ(_translation[cam_index].ptr<float>(0)[0] * _scale, _translation[cam_index].ptr<float>(1)[0] * _scale, _translation[cam_index].ptr<float>(2)[0] * _scale),
							  10 * _scale, cam_color_1[0], cam_color_1[1], cam_color_1[2], "origin_2");
		else if (cam_index == 2)
			viewer->addSphere(pcl::PointXYZ(_translation[cam_index].ptr<float>(0)[0] * _scale, _translation[cam_index].ptr<float>(1)[0] * _scale, _translation[cam_index].ptr<float>(2)[0] * _scale),
							  10 * _scale, cam_color_2[0], cam_color_2[1], cam_color_2[2], "origin_3");
	}
	if (_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2] != -1000)
	{
		float dx = (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[0];
		float dy = (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[1];
		float dz = (float)_path_mat_sphere[cam_index].ptr<short int>(frameCounter)[2];
		float global_x = (_rotation[cam_index].ptr<float>(0)[0] * dx + _rotation[cam_index].ptr<float>(0)[1] * dy + _rotation[cam_index].ptr<float>(0)[2] * dz +
						  _translation[cam_index].ptr<float>(0)[0]) *
						 _scale;
		float global_y = (_rotation[cam_index].ptr<float>(1)[0] * dx + _rotation[cam_index].ptr<float>(1)[1] * dy + _rotation[cam_index].ptr<float>(1)[2] * dz +
						  _translation[cam_index].ptr<float>(1)[0]) *
						 _scale;
		float global_z = (_rotation[cam_index].ptr<float>(2)[0] * dx + _rotation[cam_index].ptr<float>(2)[1] * dy + _rotation[cam_index].ptr<float>(2)[2] * dz +
						  _translation[cam_index].ptr<float>(2)[0]) *
						 _scale;

		if (cam_index == 0)
			viewer->addSphere(pcl::PointXYZ(global_x, global_y, global_z), 10 * _scale, cam_color_0[0], cam_color_0[1], cam_color_0[2], sphereName);
		else if (cam_index == 1)
			viewer->addSphere(pcl::PointXYZ(global_x, global_y, global_z), 10 * _scale, cam_color_1[0], cam_color_1[1], cam_color_1[2], sphereName);
		else if (cam_index == 2)
			viewer->addSphere(pcl::PointXYZ(global_x, global_y, global_z), 10 * _scale, cam_color_2[0], cam_color_2[1], cam_color_2[2], sphereName);
	}
}
