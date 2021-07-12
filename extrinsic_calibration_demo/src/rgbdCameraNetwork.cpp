#include <iostream>
#include <fstream>
#include <thread>
#include "calibrationHandler.h"
#include "threeDViewer.h"

std::vector<bool> RenderCam{true, true, true};
bool SingleColorMode = false;
bool PointCloudRendering = true;

void KeyboardEvent(const pcl::visualization::KeyboardEvent &event)
{
	if (event.getKeyCode() == '1')
	{
		RenderCam[0] = !RenderCam[0];
	}
	else if (event.getKeyCode() == '2')
	{
		RenderCam[1] = !RenderCam[1];
	}
	else if (event.getKeyCode() == '3')
	{
		RenderCam[2] = !RenderCam[2];
	}
	else if (event.getKeySym() == "z" && event.keyDown())
	{
		SingleColorMode = !SingleColorMode;
	}
	else if (event.getKeySym() == "a" && event.keyDown())
	{
		PointCloudRendering = !PointCloudRendering;
	}
}

int main()
{
	// initialization
	int number_of_cameras = 3;
	int final_number = 100;
	int frameCounter = 0;

	std::shared_ptr<CalibrationHandler> handler = std::make_shared<CalibrationHandler>(number_of_cameras, final_number);
	handler->StartProcessing();

	std::unique_ptr<ThreeDViewer> pclRender = std::make_unique<ThreeDViewer>(handler->get_shared_this());

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(KeyboardEvent);
	viewer->setCameraPosition(0, 0, -8, 0, 0, 0.1, 0, -1, 0);

	std::vector<std::string> windowName;
	for (int i = 0; i < number_of_cameras; i++)
	{
		windowName.emplace_back("Camera " + std::to_string(i));
		cv::namedWindow(windowName[i]);
	}

	std::string textID = "Image_number_text";
	bool switchFlag = PointCloudRendering;
	int sphere_counter = 0;
	while (frameCounter <= final_number)
	{
		if (frameCounter == 0)
		{
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			pclRender->DrawCameraFrustum(viewer);
			viewer->addText("Text initialization", 0, 20, textID);
		}

		if (switchFlag != PointCloudRendering)
		{
			viewer->removeAllShapes();
			pclRender->DrawCameraFrustum(viewer);
			viewer->addText("Text initialization", 0, 20, textID);
			switchFlag = PointCloudRendering;
		}

		viewer->updateText("Image# " + std::to_string(frameCounter), 0, 20, textID);
		for (int i = 0; i < number_of_cameras; i++)
		{
			pclRender->DrawSphereCenterIn2DImage(handler, i, frameCounter);
			cv::imshow(windowName[i], handler->cameras[handler->cam_ref_index[i]].rgb_l_imgs[frameCounter]);
		}
		cv::waitKey(30);

		if (PointCloudRendering)
		{
			viewer->removeAllPointClouds();
			pclRender->SetUpPointClouds(handler, frameCounter);
			for (int i = 0; i < number_of_cameras; i++)
			{
				std::string cloudName = "point_cloud_" + std::to_string(i);
				if (RenderCam[i])
				{
					pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pclRender->clouds[i]);
					viewer->addPointCloud<pcl::PointXYZRGB>(pclRender->clouds[i], rgb, cloudName);
				}
				if (SingleColorMode)
				{
					if (i == 0 && RenderCam[i])
					{
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, pclRender->cam_color_0[0],
																 pclRender->cam_color_0[1], pclRender->cam_color_0[2], cloudName);
					}
					else if (i == 1 && RenderCam[i])
					{
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, pclRender->cam_color_1[0],
																 pclRender->cam_color_1[1], pclRender->cam_color_1[2], cloudName);
					}
					else if (i == 2 && RenderCam[i])
					{
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, pclRender->cam_color_2[0],
																 pclRender->cam_color_2[1], pclRender->cam_color_2[2], cloudName);
					}
				}
			}
		}
		else
		{
			viewer->removeAllPointClouds();
			for (int i = 0; i < number_of_cameras; i++)
			{
				std::string sphereName = "sphere_center_" + std::to_string(sphere_counter);
				pclRender->DrawSphereIn3DSpace(viewer, i, frameCounter, sphereName);
				sphere_counter++;
			}
		}

		viewer->spinOnce(100);
		if (frameCounter == final_number)
			frameCounter = -1;

		frameCounter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	cv::destroyAllWindows();

	return 0;
}
