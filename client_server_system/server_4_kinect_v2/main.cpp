// Copyright 2017 University of Kentucky 
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include <iostream>
#include <cv.h>
#include <highgui.h>  
#include <cxcore.h>
#include <gl/glut.h>
#include <fstream>
#include <sstream>


#include "RemoteKinect.h"
#include "Data.h"
#include "Functions.h"
#include "Quaternion.h"
#include "UnityIPCManager.h"
const GLfloat origin_to_center = 2.5f;
int mousex, mousey, m_zoom, m_mode;
float camDist = 5.f, camDistd;
Quaternion qDown, qNow;
bool cam_flag[5] = { 0, 1, 1, 1, 1}, file_pause = 0;
unsigned int index = 0;
cv::Mat test_img;
int save_img = 0;
std::vector<cv::Point3f> dpc;

std::string name1 = "kinect1";
std::string name2 = "kinect2";
std::string name3 = "kinect3";
std::string name4 = "kinect4";

UnityIPCManager unitySender1(name1);
UnityIPCManager unitySender2(name2);
UnityIPCManager unitySender3(name3);
UnityIPCManager unitySender4(name4);


struct point {
	short x;
	short y;
	short z;
	short r;
	short g;
	short b;
};

int sendDataEveryXFrame = 30;
int frameCounter = 0;

#define MAXPOINTS 518400
point points1[MAXPOINTS];
point points2[MAXPOINTS];
point points3[MAXPOINTS];
point points4[MAXPOINTS];


int pointsCounter1 = 0;
int pointsCounter2 = 0;
int pointsCounter3 = 0;
int pointsCounter4 = 0;


void handleSend() {
	unitySender1.sendData(&points1, sizeof(points1));
	unitySender2.sendData(&points2, sizeof(points2));
	unitySender3.sendData(&points3, sizeof(points3));
	unitySender4.sendData(&points4, sizeof(points4));
	pointsCounter1 = 0;
	pointsCounter2 = 0;
	pointsCounter3 = 0;
	pointsCounter4 = 0;
}	



void OpenGL_Idle()
{
	//	OpenGL_render();	// À¢–¬œ‘ æ
	//	glClear(GL_COLOR_BUFFER_BIT);

	glutPostRedisplay();
}

void OpenGL_keyStroke(unsigned char key, int x, int y) {
	switch (key) {
	case '1':
		cam_flag[1] = !cam_flag[1];
		break;
	case '2':
		cam_flag[2] = !cam_flag[2];
		break;
	case '3':
		cam_flag[3] = !cam_flag[3];
		break;
	case '4':
		cam_flag[4] = !cam_flag[4];
		break;
	case ' ':
		file_pause = !file_pause;
		break;
	case 27:
		exit(1);
		break;
	case 'y':
		save_img = !save_img;
		break;
	case 'o':
		/*Let clients save Image*/
		kinect_1.setSaveSignal();
		printf("sent save singal to Client1\n");
#ifdef CLIENT2
		{
			kinect_2.setSaveSignal();
			printf("sent save singal to Client2\n");
		}
#endif

#ifdef CLIENT3
		{
			kinect_3.setSaveSignal();
			printf("sent save singal to Client3\n");
		}
#endif

#ifdef CLIENT4
		{
			kinect_4.setSaveSignal();
			printf("sent save singal to Client4\n");
		}
#endif
		/*Save final image locally*/
		/*temp_file = server_file_name;
		temp_file.append(to_string(img_index)).append(".jpg");
		cv::imwrite(to_char(temp_file), virimg);
		img_index++;*/
		break;
	case 'c':
		/*Let clients stop/start calibration*/
		kinect_1.setCalibConnectionSignal();
#ifdef CLIENT2
		kinect_2.setCalibConnectionSignal();
#endif
#ifdef CLIENT3
		kinect_3.setCalibConnectionSignal();
#endif

#ifdef CLIENT4
		kinect_4.setCalibConnectionSignal();
#endif
		break;
	case 't':
		/*Let clients do tracking on the calibrated images*/
		kinect_1.setPathTrackingSignal(1);
#ifdef CLIENT2
		kinect_2.setPathTrackingSignal(1);
#endif
#ifdef CLIENT3
		kinect_3.setPathTrackingSignal(1);
#endif

#ifdef CLIENT4
		kinect_4.setPathTrackingSignal(1);
#endif
		break;
	default:
		break;
	}
}

void OpenGL_mouseMotion(int x, int y) {
	pt3D xAxis(1.0f, 0.0f, 0.0f), yAxis(0.0f, 1.0f, 0.0f), zAxis(0.0f, 0.0f, 1.0f);
	float rotx, roty, rotz;
	Quaternion qRot;
	switch (m_mode) {
	case 1:  //left button, drag rotation x_y
		rotx = (float)(y - mousey)*0.1f*PI / 180.0f;   //Rotate around x axis
		qRot.FromAxis(xAxis, rotx);
		qNow = qRot*qDown;
		roty = -(float)(x - mousex)*0.1f*PI / 180.0f;   //Rotate around y axis.
		qRot.FromAxis(yAxis, roty);
		qNow = qRot*qNow;
		break;
	case 2: //right button, drag rotation z
		rotz = (float)(x + y - mousex - mousey)*0.1f*PI / 180.0f; //Rotate around z axis.
		qRot.FromAxis(zAxis, rotz);
		qNow = qRot*qDown;
		break;
	case 3: //zoom
		camDist = camDistd + (float)(y - m_zoom)*0.05;
		if (camDist < 0.0001)
			camDist = 0.0001;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

void OpenGL_mouseMove(int button, int state, int x, int y)
{
	static int old_x, old_y, old_zoom;
	Quaternion qRot;
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN){
			if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
				m_mode = 3;
				m_zoom = y;
				qDown = qNow;
				camDistd = camDist;
			}
			else if (glutGetModifiers() == 0) {
				m_mode = 1;	// rotation
				mousex = x; mousey = y;
				qDown = qNow;
			}
		}
		if (state == GLUT_UP){
			m_mode = 0;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN){
			m_mode = 2;
			mousex = x; mousey = y;
			qDown = qNow;
		}
		if (state == GLUT_UP){
			m_mode = 0;
		}
		break;
	case GLUT_MIDDLE_BUTTON:
		qNow = qRot;
		camDist = 5.0f;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void OpenGL_init(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);	// background color: white 
	glShadeModel(GL_FLAT);
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);


	fprintf(stderr, "Connect to remote client ...");

	//cv::namedWindow("Final Virtual");
	test_img = cv::imread("img062.jpg");
	initialize();
}

void OpenGL_changeSize(int w, int h)
{
	glViewport(0, 0, GLsizei(w), GLsizei(h));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.1f, 50.0f);
	glMatrixMode(GL_MODELVIEW);
}

void calc_pointcloud(int camNum) {
	inverseProject(RGBImage[camNum], depthImage[camNum], xyzImage[camNum]);
}

void OpenGL_render(void)
{
	/*Each round renew final image*/
	renewImage();

	/*Step 1:   Starting first kinect to send data*/
	int status_1 = kinect_1.startTransfer();
	if (index > 0) {
		/*kinect_1.passEyeCenter(recieved_eye_center[0]);
		printMatrix(recieved_eye_center[0]);*/
	}
	else {
		if (status_1 == 0)
			fprintf(stderr, "okay.\n Press ESC to quit\n");
		else {
			fprintf(stderr, "Problem with connection. Abort\n");
			exit(-1);
		}
	}
	/*Starting second kinect to send data*/
#ifdef CLIENT2
	{
		int status_2 = kinect_2.startTransfer();
		if (index > 0) {

			//test->imageData = (char*)(depthImage_2.operator IplImage);

			/*Do processing on images recieved from second kinect*/

		}
		else {
			if (status_2 == 0)
				fprintf(stderr, "okay.\n Press ESC to quit\n");
			else {
				fprintf(stderr, "Problem with connection. Abort\n");
				exit(-1);
			}
		}
	}
#endif

	/*Starting third kinect to send data*/
#ifdef CLIENT3
	{
		int status_3 = kinect_3.startTransfer();
		if (index > 0) {

			//test->imageData = (char*)(depthImage_2.operator IplImage);

			/*Do processing on images recieved from second kinect*/

		}
		else {
			if (status_3 == 0)
				fprintf(stderr, "okay.\n Press ESC to quit\n");
			else {
				fprintf(stderr, "Problem with connection. Abort\n");
				exit(-1);
			}
		}
	}
#endif

#ifdef CLIENT4
	{
		int status_4 = kinect_4.startTransfer();
		if (index > 0) {

			//test->imageData = (char*)(depthImage_2.operator IplImage);

			/*Do processing on images recieved from second kinect*/

		}
		else {
			if (status_4 == 0)
				fprintf(stderr, "okay.\n Press ESC to quit\n");
			else {
				fprintf(stderr, "Problem with connection. Abort\n");
				exit(-1);
			}
		}
	}
#endif

	/*Step 2:    Generate and dispaly virtual image from all clients*/
	if (index > 0)
	{
		/*processImage();
		cv::imshow("Final Virtual", virimg);*/
	}

	/*Step 3:   Copy image data and used for next frame*/
	kinect_1.getData(RGBImage[0], /*depthImage[0],*/ PointCloudX[0], PointCloudY[0], PointCloudZ[0]/*, kinect_1_cloud*/);

#ifdef CLIENT2
	{
		kinect_2.getData(RGBImage[1], /*depthImage[1],*/ PointCloudX[1], PointCloudY[1], PointCloudZ[1]/*, kinect_2_cloud*/);
	}
#endif

#ifdef CLIENT3
	{
		kinect_3.getData(RGBImage[2], /*depthImage[1],*/ PointCloudX[2], PointCloudY[2], PointCloudZ[2]/*, kinect_3_cloud*/);
	}
#endif

#ifdef CLIENT4
	{
		kinect_4.getData(RGBImage[3], /*depthImage[3],*/ PointCloudX[3], PointCloudY[3], PointCloudZ[3]/*, kinect_4_cloud*/);
	}
#endif




	if (kinect_1.calib_conn[2] == 1)
	{
		cv::Mat OutputRGB0;
		cv::resize(RGBImage[0], OutputRGB0, cv::Size(), 0.75, 0.75);
		cv::imshow("kinect 1 rgb", OutputRGB0);
#ifdef	CLIENT2
		cv::Mat OutputRGB1;
		cv::resize(RGBImage[1], OutputRGB1, cv::Size(), 0.75, 0.75);
		cv::imshow("kinect 2 rgb", OutputRGB1);
#endif
#ifdef	CLIENT3
		cv::Mat OutputRGB2;
		cv::resize(RGBImage[2], OutputRGB2, cv::Size(), 0.75, 0.75);
		cv::imshow("kinect 3 rgb", OutputRGB2);
#endif
#ifdef	CLIENT4
		cv::Mat OutputRGB3;
		cv::resize(RGBImage[3], OutputRGB3, cv::Size(), 0.75, 0.75);
		cv::imshow("kinect 4 rgb", OutputRGB3);
#endif
	}


	/*plot 3D point*/
	if (!file_pause) {
		RGBImage[0].copyTo(rgbImage[0]);
	}
#ifdef	CLIENT2
	if (!file_pause) {
		RGBImage[1].copyTo(rgbImage[1]);
	}
#endif
#ifdef	CLIENT3
	if (!file_pause) {
		RGBImage[2].copyTo(rgbImage[2]);
	}
#endif
#ifdef	CLIENT4
	if (!file_pause) {
		RGBImage[3].copyTo(rgbImage[3]);
	}
#endif

	if (kinect_1.getPathTrackingSignal() == 1 && kinect_2.getPathTrackingSignal() == 1 && kinect_3.getPathTrackingSignal() == 1 && kinect_4.getPathTrackingSignal() == 1) //make sure the path tracking procedure is only excuted once
	{
		path_obtain_signal = 1;
		kinect_1.setPathTrackingSignal(0);
		kinect_2.setPathTrackingSignal(0);
		kinect_3.setPathTrackingSignal(0);
		kinect_4.setPathTrackingSignal(0);

		kinect_1._pathMat.copyTo(path_mat[0]);
		kinect_2._pathMat.copyTo(path_mat[1]);
		kinect_3._pathMat.copyTo(path_mat[2]);
		kinect_4._pathMat.copyTo(path_mat[3]);



		loadPath();


		computeExtrinsics(1, 0);
		computeExtrinsics(2, 0);
		computeExtrinsics(3, 0);

		loadExtrinsics();
	}




	index++;




	/************************************** Po, Wanxin Start from Here  *****************************************/
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (1) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
	gluLookAt(0.0f, 0.0f, 0.0f - camDist, 0.0f, 0.0f, origin_to_center, 0.0f, 1.0f, 0.0f);
	glPushMatrix();
	if (1) {
		glTranslatef(0.0f, 0.0f, origin_to_center);
		pt3D rotAxis;
		float rotAngle;
		float rotMat[16];
		qNow.getAxisAngle(rotAxis, rotAngle);
		qNow.getMatrix4(rotMat);
		glMultMatrixf(rotMat);
		glTranslatef(-0.0f, -0.0f, -origin_to_center);
	}

	glColor3f(1.0f, 1.0f, 1.0f);
	glPointSize(0.1f);
	glBegin(GL_POINTS);

	if (cam_flag[1]) {
		for (int y = 0; y < RGBImage[0].rows; y = y + offset) {
			for (int x = 0; x < RGBImage[0].cols; x = x + offset) {
				float b = rgbImage[0].ptr<uchar>(y)[3 * x] / 255.f;
				float g = rgbImage[0].ptr<uchar>(y)[3 * x + 1] / 255.f;
				float r = rgbImage[0].ptr<uchar>(y)[3 * x + 2] / 255.f;

				float dx = (float)PointCloudX[0].ptr<short int>(y)[x];
				float dy = (float)PointCloudY[0].ptr<short int>(y)[x];
				float dz = (float)PointCloudZ[0].ptr<short int>(y)[x];

				if (dz > 1.f) {
					glColor3f(r, g, b);
					glVertex3f(dx*0.001, dy*0.001, dz*0.001);
					point newPoint;
					newPoint.x = dx;
					newPoint.y = dy;
					newPoint.z = dz;
					newPoint.r = ((float)r) * 255;
					newPoint.g = ((float)g) * 255;
					newPoint.b = ((float)b) * 255;
					if (pointsCounter1 > MAXPOINTS) {
						break;
					}
					else {
						points1[pointsCounter1] = newPoint;
						pointsCounter1 = pointsCounter1 + 1;
					}
				}
			}
		}

	}

#ifdef	CLIENT2
	// second camera
	if (cam_flag[2]) {
		for (int y = 0; y < RGBImage[1].rows; y = y + offset) {
			for (int x = 0; x < RGBImage[1].cols; x = x + offset) {
				float b = rgbImage[1].ptr<uchar>(y)[3 * x] / 255.f;
				float g = rgbImage[1].ptr<uchar>(y)[3 * x + 1] / 255.f;
				float r = rgbImage[1].ptr<uchar>(y)[3 * x + 2] / 255.f;

				float dx = (float)PointCloudX[1].ptr<short int>(y)[x];
				float dy = (float)PointCloudY[1].ptr<short int>(y)[x];
				float dz = (float)PointCloudZ[1].ptr<short int>(y)[x];

				if (dz > 1.f) {
					float x2 = R21[0][0] * dx + R21[0][1] * dy + R21[0][2] * dz + T21[0];
					float y2 = R21[1][0] * dx + R21[1][1] * dy + R21[1][2] * dz + T21[1];
					float z2 = R21[2][0] * dx + R21[2][1] * dy + R21[2][2] * dz + T21[2];
					glColor3f(r, g, b);
					glVertex3f(x2*0.001f, y2*0.001f, z2*0.001f);

					point newPoint;
					newPoint.x = x2;
					newPoint.y = y2;
					newPoint.z = z2;
					newPoint.r = ((float)r) * 255;
					newPoint.g = ((float)g) * 255;
					newPoint.b = ((float)b) * 255;
					if (pointsCounter2 > MAXPOINTS) {
						break;
					}
					else {
						points2[pointsCounter2] = newPoint;
						pointsCounter2 = pointsCounter2 + 1;
					}
				}
			}
		}
	}
#endif

#ifdef	CLIENT3
	// third camera
	if (cam_flag[3]) {
		for (int y = 0; y < RGBImage[2].rows; y = y + offset) {
			for (int x = 0; x < RGBImage[2].cols; x = x + offset) {
				float b = rgbImage[2].ptr<uchar>(y)[3 * x] / 255.f;
				float g = rgbImage[2].ptr<uchar>(y)[3 * x + 1] / 255.f;
				float r = rgbImage[2].ptr<uchar>(y)[3 * x + 2] / 255.f;

				float dx = (float)PointCloudX[2].ptr<short int>(y)[x];
				float dy = (float)PointCloudY[2].ptr<short int>(y)[x];
				float dz = (float)PointCloudZ[2].ptr<short int>(y)[x];

				if (dz>1.f) {
					float x3 = R31[0][0] * dx + R31[0][1] * dy + R31[0][2] * dz + T31[0];
					float y3 = R31[1][0] * dx + R31[1][1] * dy + R31[1][2] * dz + T31[1];
					float z3 = R31[2][0] * dx + R31[2][1] * dy + R31[2][2] * dz + T31[2];
					glColor3f(r, g, b);
					glVertex3f(x3*0.001f, y3*0.001f, z3*0.001f);

					point newPoint;
					newPoint.x = x3;
					newPoint.y = y3;
					newPoint.z = z3;
					newPoint.r = ((float)r) * 255;
					newPoint.g = ((float)g) * 255;
					newPoint.b = ((float)b) * 255;
					if (pointsCounter3 > MAXPOINTS) {
						break;
					}
					else {
						points3[pointsCounter3] = newPoint;
						pointsCounter3 = pointsCounter3 + 1;
					}
				}
			}
		}
	}
#endif

#ifdef	CLIENT4
	 //third camera
	if (cam_flag[4]) {
		for (int y = 0; y < RGBImage[3].rows; y = y + offset) {
			for (int x = 0; x < RGBImage[3].cols; x = x + offset) {
				float b = rgbImage[3].ptr<uchar>(y)[3 * x] / 255.f;
				float g = rgbImage[3].ptr<uchar>(y)[3 * x + 1] / 255.f;
				float r = rgbImage[3].ptr<uchar>(y)[3 * x + 2] / 255.f;

				float dx = (float)PointCloudX[3].ptr<short int>(y)[x];
				float dy = (float)PointCloudY[3].ptr<short int>(y)[x];
				float dz = (float)PointCloudZ[3].ptr<short int>(y)[x];

				if (dz>1.f) {
					float x4 = R41[0][0] * dx + R41[0][1] * dy + R41[0][2] * dz + T41[0];
					float y4 = R41[1][0] * dx + R41[1][1] * dy + R41[1][2] * dz + T41[1];
					float z4 = R41[2][0] * dx + R41[2][1] * dy + R41[2][2] * dz + T41[2];
					glColor3f(r, g, b);
					glVertex3f(x4*0.001f, y4*0.001f, z4*0.001f);

					point newPoint;
					newPoint.x = x4;
					newPoint.y = y4;
					newPoint.z = z4;
					newPoint.r = ((float)r) * 255;
					newPoint.g = ((float)g) * 255;
					newPoint.b = ((float)b) * 255;
					if (pointsCounter4 > MAXPOINTS) {
						break;
					}
					else {
						points4[pointsCounter4] = newPoint;
						pointsCounter4 = pointsCounter4 + 1;
					}
				}
			}
		}
	}
	handleSend();
#endif
	glEnd();

	// Saved Output
	if (save_img == 1){
		vector<unsigned char>pixels(GLWidth*GLHeight * 4);
		glReadPixels
			(
			0, /*glutGet( GLUT_WINDOW_HEIGHT ) - */0,
			GLWidth, GLHeight,
			GL_RGBA, GL_UNSIGNED_BYTE, &pixels[0]
			);


		for (int y = 0; y < GLHeight; y++){
			for (int x = 0; x < GLWidth; x++){
				GLResults.at<cv::Vec4b>((GLHeight - 1) - y, x)[0] = pixels[y*GLWidth * 4 + 4 * x + 2];
				GLResults.at<cv::Vec4b>((GLHeight - 1) - y, x)[1] = pixels[y*GLWidth * 4 + 4 * x + 1];
				GLResults.at<cv::Vec4b>((GLHeight - 1) - y, x)[2] = pixels[y*GLWidth * 4 + 4 * x + 0];
				GLResults.at<cv::Vec4b>((GLHeight - 1) - y, x)[3] = pixels[y*GLWidth * 4 + 4 * x + 3];
			}
		}


		char filename2[200];
		sprintf_s(filename2, "Output\\%s%04d.jpg", GLResultPrefix.c_str(), idxcounter);
		imwrite(filename2, GLResults);

		idxcounter++;
		pixels.clear();
	}


	/**********************************   Po, Wanxin  ***********************************************/

	//cloud.updataPointCloud();	// update next frame cloudpoint data
	glPopMatrix();
	glFinish();
	//glFlush();
	glutSwapBuffers();

}

int main(int argc, char **argv) {

	/*Register Mouse operation*/
	//cv::setMouseCallback("Final Virtual", onRGB1Mouse, 0);
	//cv::setMouseCallback("virtual", onVirMouse, 0);

	//initialize();	

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA/*GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA*/);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(200, 100);
	glutCreateWindow("Kinect Point Cloud");
	OpenGL_init();

	glutDisplayFunc(OpenGL_render);
	glutReshapeFunc(OpenGL_changeSize);
	glutKeyboardFunc(OpenGL_keyStroke);
	glutMouseFunc(OpenGL_mouseMove);
	glutMotionFunc(OpenGL_mouseMotion);
	glutIdleFunc(OpenGL_Idle);
	glutMainLoop();

	return 0;
}
