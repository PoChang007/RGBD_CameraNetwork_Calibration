// Copyright 2017 University of Kentucky 
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include "RemoteKinect.h"

#define XN_MAX_UINT16  65535

#define WIDTH 960
#define HEIGHT 540

#define WIDTH1 1920
#define HEIGHT1 1080

// Constructor: connect to the remote serve
// If _status becomes -1, problem with connection and all subsequent
// calls will return this error code.
RemoteKinect::RemoteKinect(char *ipAddr, int port): 
_status(0), _hsock(0), _threadRunning(false) {

	/*For Samson Use*/
	local_eye_center.create(3, 1, CV_16UC1);
	recieved_eye_center.create(3, 1, CV_16UC1);


	/*Initialize Signal Data*/
	calib_conn[0] = 2;
	calib_conn[1] = 0;
	//calib_conn[2] = 1;
	calib_conn[2] = 1500;



    //Initialize socket support WINDOWS ONLY!
    unsigned short wVersionRequested;
    WSADATA wsaData;
    int err;
    wVersionRequested = MAKEWORD( 2, 2 );
    err = WSAStartup( wVersionRequested, &wsaData );
    if ( err != 0 || ( LOBYTE( wsaData.wVersion ) != 2 || HIBYTE( wsaData.wVersion ) != 2 )) {
		fprintf(stderr, "RemoteKinect::RemoteKinect : Could not find useable sock dll %d\n",WSAGetLastError());
        _status = -1; 
		return;
	}

    //Initialize sockets and set any options
    int * p_int ;
    _hsock = socket(AF_INET, SOCK_STREAM, 0);
    if(_hsock == -1){
		fprintf(stderr,"Error initializing socket %d\n",WSAGetLastError());
	    _status = -1;
		return;
    }
     
    p_int = (int*)malloc(sizeof(int));
    *p_int = 1;
    if( (setsockopt(_hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 )||
        (setsockopt(_hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 ) ){
         fprintf(stderr,"Error setting options %d\n", WSAGetLastError());
         free(p_int);
         _status = -1;
		 return;
    }
    free(p_int);
 
    //Connect to the server
    struct sockaddr_in my_addr;
    my_addr.sin_family = AF_INET ;
    my_addr.sin_port = htons(port);
    memset(&(my_addr.sin_zero), 0, 8);
    my_addr.sin_addr.s_addr = inet_addr(ipAddr);
    if( connect(_hsock, (struct sockaddr*)&my_addr, sizeof(my_addr)) == SOCKET_ERROR ){
        fprintf(stderr, "Error connecting socket %d\n", WSAGetLastError());
        _status = -1;
		return;
	}



	//Synchronization instructions
	save_img_signal = 0;


	//Initialize eye_center stream
	centerStream = (char *)malloc(1*3*sizeof(short int));
	eye_center_buf = (char *)malloc(1*3*sizeof(short int));


	//Initialize local_eye_center values
	local_eye_center.ptr<short int>(0)[0] = 500;
	local_eye_center.ptr<short int>(1)[0] = 200;
	local_eye_center.ptr<short int>(2)[0] = 600;

}

// destructor
// Cleanup
RemoteKinect::~RemoteKinect() {
	if (_threadRunning) {
		if (WaitForSingleObject(_thread,INFINITE)  != WAIT_OBJECT_0)  {
			_status = -2;
			_threadRunning = false;
		}
	}
	if ((_status == 0) && (_hsock != 0)) {
		unsigned long intbuf = 9999;
		send(_hsock, (char *)(&intbuf), sizeof(unsigned long), 0);
		closesocket(_hsock);
	}
	_RGBFrame.release();
	_depthFrame.release();
	_pointcloudX_Frame.release();
	_pointcloudY_Frame.release();
	_pointcloudZ_Frame.release();
	_dataBuffer.release();
	_pointcloudX_dataBuffer.release();
	_pointcloudY_dataBuffer.release();
	_pointcloudZ_dataBuffer.release();
	_cloudBuffer.clear();
	delete[]centerStream;
} 

/*Send the data out to Data file*/
void RemoteKinect::passEyeCenter(cv::Mat toServer)
{
	toServer.copyTo(local_eye_center);
}


/*Receive detected eye center, make sure only 1 recieved eye center is used even there are multiple clients*/
void RemoteKinect::recieveEyeCenter(char* data)
{
	memcpy(recieved_eye_center.data, data, 3 * sizeof(short int)); //save the detected eye center to local
	//recieved_eye_center.copyTo(local_eye_center);	
}

cv::Mat RemoteKinect::getRecievedEyeCenter()
{
	return recieved_eye_center;
}

// startTransfer
int RemoteKinect::startTransfer() {
	if (_status < 0) return _status; 
	if (_threadRunning) {
		fprintf(stderr,"RemoteKinect::startTransfer : can't initiate another startTransfer.\n");
		return 0;
	}
	_threadRunning = true;
	_thread = CreateThread(0,0,(LPTHREAD_START_ROUTINE)&RemoteKinect::getRemoteData,
		(LPVOID)this,0,0);
	return 0;
}

// getData
int RemoteKinect::getData(cv::Mat &RGBframe, /*cv::Mat &depthframe,*/ cv::Mat &pointcloudX_frame, cv::Mat &pointcloudY_frame, cv::Mat &pointcloudZ_frame/*, std::vector<int64_t> &cloudFrame*/) {
	if (_status < 0) return _status;
	if (_threadRunning) {
		if (WaitForSingleObject(_thread,INFINITE)  != WAIT_OBJECT_0)  {
			_status = -2;
			_threadRunning = false;
			return _status;
		}
	}
	_threadRunning = false;

	//RGBframe = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);

	_RGBFrame.copyTo(RGBframe);
	//_depthFrame.copyTo(depthframe);
	_pointcloudX_Frame.copyTo(pointcloudX_frame);
	_pointcloudY_Frame.copyTo(pointcloudY_frame);
	_pointcloudZ_Frame.copyTo(pointcloudZ_frame);

	//copy(_cloudBuffer.begin(), _cloudBuffer.end(), cloudFrame.begin());
	
	return 0;
}

// getRemoteData
DWORD WINAPI RemoteKinect::getRemoteData(void *basept) {

	RemoteKinect *bp = (RemoteKinect *)basept;
	int bytecount;

	
	short int test_data = 25;
	// (0) Send 1 unsigned long (value = 1) to get JPEG bitstream
	/*unsigned long intbuf = 0;
	if ((bytecount=send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
		bp->_status = -1;
		return 0;
	}*/

	/*
	char *eye_center = 0;
	if ((bytecount=recv(bp->_hsock, (eye_center), 3 * sizeof(short int), 0)) == SOCKET_ERROR) {
		bp->_status = -1;
		return 0;
	}
	{
		memcpy(eye_center, bp->recieved_eye_center.data, 3*sizeof(short int));
	}*/

	
	

	// (1) Send 1 unsigned long (value = 1) to get JPEG bitstream
	short int eye_pos[3];
	eye_pos[0] = bp->calib_conn[0];
	eye_pos[1] = bp->calib_conn[1];
	eye_pos[2] = bp->calib_conn[2];

	if ((bytecount=send(bp->_hsock, (char *)(eye_pos), sizeof(short int) * 3, 0)) == SOCKET_ERROR) {
		bp->_status = -1;
		return 0;
	}
	

	//for demo use
	if(eye_pos[2] == 1)
	{
		// (1) Receive 1 unsigned long on the size of the JPEG bitstream
		unsigned long dataSize = 0;
		if ((bytecount=recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// Create a large enough buffer 
		bp->_dataBuffer.create(1,dataSize,CV_8UC1);

		// (2) Download the jpeg bitstream
		char *bufindex = (char *) bp->_dataBuffer.data;
		unsigned long  dataLeft = dataSize;
		while (dataLeft > 0) {
			bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);

			if (bytecount == SOCKET_ERROR) {
				bp->_status = -1;
				return 0;
			}
			bufindex += bytecount;
			dataLeft -= bytecount;
		}

		// Uncompress JPEG bitstream to create image data
		bp->_RGBFrame = cv::imdecode(bp->_dataBuffer,1);

		//  Use dimension to initialize depth data
		//if ((bp->_depthFrame.size().height != bp->_RGBFrame.size().height) ||
		//	(bp->_depthFrame.size().width  != bp->_RGBFrame.size().width)) 
		//	bp->_depthFrame.create(bp->_RGBFrame.size().height,bp->_RGBFrame.size().width,CV_16UC1);
		
		//bp->_depthFrame.create(HEIGHT, WIDTH, CV_16UC1);
		//// (3) Send 1 unsigned long (value = 2) to get depth bitstream
		//long long intbuf = 2;
		//if ((bytecount=send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
		//	bp->_status = -1;
		//	return 0;
		//}
		//
		//// (4) Receive 1 unsigned long on the size of the depth bitstream
		//if ((bytecount=recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
		//	bp->_status = -1;
		//	return 0;
		//}

		//// Create a large enough buffer 
		//bp->_dataBuffer.create(1,dataSize,CV_8UC1);

		//// (5) Download the depth bitstream
		//bufindex = (char *)bp->_dataBuffer.data;
		//dataLeft = dataSize;
		//while (dataLeft > 0) {
		//	bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
		//	if (bytecount == SOCKET_ERROR) {
		//		bp->_status = -1;
		//		return 0;
		//	}
		//	bufindex += bytecount;
		//	dataLeft -= bytecount;
		//}

		//// (6) Uncompress depth data
		//dataLeft = bp->_depthFrame.size().height * bp->_depthFrame.size().width * 
		//	sizeof(unsigned short);
		//bp->_depthCodec.decode((unsigned char *)bp->_dataBuffer.data, dataSize, 
		//	(unsigned short *)bp->_depthFrame.data, &dataLeft);


		bp->_pointcloudX_Frame.create(HEIGHT, WIDTH, CV_16UC1);
		// (3.1) Send 1 unsigned long (value = 3) to get Point cloud X bitstream
		long long intbuf = 3;
		if ((bytecount = send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// (4.1) Receive 1 unsigned long on the size of the Point cloud X bitstream
		if ((bytecount = recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// Create a large enough buffer 
		bp->_pointcloudX_dataBuffer.create(1, dataSize, CV_8UC1);

		// (5.1) Download the Point Cloud X bitstream
		bufindex = (char *)bp->_pointcloudX_dataBuffer.data;
		dataLeft = dataSize;
		while (dataLeft > 0) {
			bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
			if (bytecount == SOCKET_ERROR) {
				bp->_status = -1;
				return 0;
			}
			bufindex += bytecount;
			dataLeft -= bytecount;
		}

		// (6.1) Uncompress Point Cloud X data
		dataLeft = bp->_pointcloudX_Frame.size().height * bp->_pointcloudX_Frame.size().width *
			sizeof(unsigned short);
		bp->_pointcloudX_Codec.decode((unsigned char *)bp->_pointcloudX_dataBuffer.data, dataSize,
			(unsigned short *)bp->_pointcloudX_Frame.data, &dataLeft);


		bp->_pointcloudY_Frame.create(HEIGHT, WIDTH, CV_16UC1);
		// (3.2) Send 1 unsigned long (value = 4) to get Point cloud Y bitstream
		intbuf = 4;
		if ((bytecount = send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// (4.2) Receive 1 unsigned long on the size of the Point cloud Y bitstream
		if ((bytecount = recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// Create a large enough buffer 
		bp->_pointcloudY_dataBuffer.create(1, dataSize, CV_8UC1);

		// (5.2) Download the Point Cloud Y bitstream
		bufindex = (char *)bp->_pointcloudY_dataBuffer.data;
		dataLeft = dataSize;
		while (dataLeft > 0) {
			bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
			if (bytecount == SOCKET_ERROR) {
				bp->_status = -1;
				return 0;
			}
			bufindex += bytecount;
			dataLeft -= bytecount;
		}

		// (6.2) Uncompress Point Cloud Y data
		dataLeft = bp->_pointcloudY_Frame.size().height * bp->_pointcloudY_Frame.size().width *
			sizeof(unsigned short);
		bp->_pointcloudY_Codec.decode((unsigned char *)bp->_pointcloudY_dataBuffer.data, dataSize,
			(unsigned short *)bp->_pointcloudY_Frame.data, &dataLeft);


		bp->_pointcloudZ_Frame.create(HEIGHT, WIDTH, CV_16UC1);
		// (3.3) Send 1 unsigned long (value = 5) to get Point cloud Z bitstream
		intbuf = 5;
		if ((bytecount = send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// (4.3) Receive 1 unsigned long on the size of the Point cloud Z bitstream
		if ((bytecount = recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}

		// Create a large enough buffer 
		bp->_pointcloudZ_dataBuffer.create(1, dataSize, CV_8UC1);

		// (5.3) Download the Point Cloud Z bitstream
		bufindex = (char *)bp->_pointcloudZ_dataBuffer.data;
		dataLeft = dataSize;
		while (dataLeft > 0) {
			bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
			if (bytecount == SOCKET_ERROR) {
				bp->_status = -1;
				return 0;
			}
			bufindex += bytecount;
			dataLeft -= bytecount;
		}

		// (6.3) Uncompress Point Cloud Z data
		dataLeft = bp->_pointcloudZ_Frame.size().height * bp->_pointcloudZ_Frame.size().width *
			sizeof(unsigned short);
		bp->_pointcloudZ_Codec.decode((unsigned char *)bp->_pointcloudZ_dataBuffer.data, dataSize,
			(unsigned short *)bp->_pointcloudZ_Frame.data, &dataLeft);


//		// (8) Receive 1 unsigned long long on the size of the cloud bitstream
//		uint64_t  dataSizec = 0;
//		if ((bytecount = recv(bp->_hsock, (char *)(&dataSizec), sizeof(uint64_t), 0)) == SOCKET_ERROR) {
//			bp->_status = -1;
//			return 0;
//
//		}
//#ifdef DEBUG
//		fprintf(stderr, "Expect Depth Stream size : %u\n", dataSize);
//		fflush(stderr);
//#endif
//		bp->_cloudBuffer.resize(dataSizec);
//		// (9) Download the cloud bitstream
//		char *bufindexc = (char *)&bp->_cloudBuffer.front();
//		uint64_t dataLeftc = dataSizec*sizeof(int64_t);
//		while (dataLeftc > 0) {
//			int bytecountc = recv(bp->_hsock, bufindexc, dataLeftc/*3 * sizeof(bufindexc)*/, 0);
//			if (bytecountc == SOCKET_ERROR) {
//				bp->_status = -1;
//				return 0;
//			}
//			bufindexc += bytecountc;
//			dataLeftc -= bytecountc;
//		}


	}
	// Used right after the tracking is done on the client side and only be used once
	else if(bp->getPathTrackingSignal() == 1)
	{
		//(1)a This is for tracking and recieving path use, this step is executed only when calib_conn[1] = 1 (8) Receive 1 unsigned long on the size of the depth bitstream
		unsigned long pathSize = 0;
		if ((bytecount=recv(bp->_hsock, (char *)(&pathSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
			bp->_status = -1;
			return 0;
		}		

		// creat a data buffer for the depth
		bp->_dataBuffer.create(1,pathSize,CV_8UC1);
		
		// (1)b get the path stream
		char *bufindex = (char *)bp->_dataBuffer.data;
		unsigned long  dataLeft = pathSize;
		while (dataLeft > 0) {
			bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
			if (bytecount == SOCKET_ERROR) {
				bp->_status = -1;
				return 0;
			}
			bufindex += bytecount;
			dataLeft -= bytecount;
		}

		//Uncompress path data
		bp->_pathMat.create(3000, 3, CV_16UC1);
		printf("datasize: %d\n", pathSize);
		dataLeft = bp->_pathMat.size().height * bp->_pathMat.size().width * 
			sizeof(unsigned short);
		bp->_depthCodec.decode((unsigned char *)bp->_dataBuffer.data, pathSize, 
			(unsigned short *)bp->_pathMat.data, &dataLeft);


		/*for(int i = 0; i < bp->_pathMat.rows; i++)
		{
			printf("[%d] %d, %d, %d\n", i, bp->_pathMat.ptr< short>(i)[0], 
				bp->_pathMat.ptr< short>(i)[1], bp->_pathMat.ptr< short>(i)[2]);
		}*/

	}

	// (2) Receive 1 unsigned long on the size of the JPEG bitstream
	unsigned long dataSize = 0;
	if ((bytecount=recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
		bp->_status = -1;
		return 0;

	}


//#ifdef DEBUG
//	fprintf(stderr,"Expect JPEG size : %u\n",dataSize);
//	fflush(stderr);
//#endif
//	// (3) Create a large enough buffer 
//	bp->_dataBuffer.create(1,dataSize,CV_8UC1);
//	
//	// (4) Download the jpeg bitstream
//	char *bufindex = (char *) bp->_dataBuffer.data;
//	unsigned long  dataLeft = dataSize;
//	while (dataLeft > 0) {
//		bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
//
//		if (bytecount == SOCKET_ERROR) {
//			bp->_status = -1;
//			return 0;
//		}
//		bufindex += bytecount;
//		dataLeft -= bytecount;
//	}
//
//	// (5) Uncompress JPEG bitstream to create image data
//	bp->_RGBFrame = cv::imdecode(bp->_dataBuffer,1);
//#ifdef DEBUG
//	fprintf(stderr,"Successfully decode jpeg %d x %d.\n",bp->_RGBFrame.size().height,
//		bp->_RGBFrame.size().width);fflush(stderr);
//#endif
//
//
//	// (6) Use dimension to initialize depth data
//	if ((bp->_depthFrame.size().height != bp->_RGBFrame.size().height) ||
//		(bp->_depthFrame.size().width  != bp->_RGBFrame.size().width)) 
//		bp->_depthFrame.create(bp->_RGBFrame.size().height,bp->_RGBFrame.size().width,CV_16UC1);
//
//	// (7) Send 1 unsigned long (value = 2) to get depth bitstream
//	unsigned long intbuf = 2;
//	if ((bytecount=send(bp->_hsock, (char *)(&intbuf), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
//		bp->_status = -1;
//		return 0;
//	}
//
//	// (8) Receive 1 unsigned long on the size of the depth bitstream
//	if ((bytecount=recv(bp->_hsock, (char *)(&dataSize), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
//		bp->_status = -1;
//		return 0;
//
//	}
//#ifdef DEBUG
//	fprintf(stderr,"Expect Depth Stream size : %u\n",dataSize);
//	fflush(stderr);
//#endif
//
//	// (9) Create a large enough buffer 
//	bp->_dataBuffer.create(1,dataSize,CV_8UC1);
//	
//	// (10) Download the depth bitstream
//	bufindex = (char *)bp->_dataBuffer.data;
//	dataLeft = dataSize;
//	while (dataLeft > 0) {
//		bytecount = recv(bp->_hsock, bufindex, dataLeft, 0);
//		if (bytecount == SOCKET_ERROR) {
//			bp->_status = -1;
//			return 0;
//		}
//		bufindex += bytecount;
//		dataLeft -= bytecount;
//	}
//#ifdef DEBUG
//	cv::Scalar mean,std;
//	cv::meanStdDev(bp->_dataBuffer,mean,std);
//	fprintf(stderr,">>B mean = %f, stdev = %f\n",mean[0],std[0]);	
//#endif
//
//	// (11) Uncompress depth data
//	dataLeft = bp->_depthFrame.size().height * bp->_depthFrame.size().width * 
//		sizeof(unsigned short);
//	bp->_depthCodec.decode((unsigned char *)bp->_dataBuffer.data, dataSize, 
//		(unsigned short *)bp->_depthFrame.data, &dataLeft);
//
//
//	// (12) Send a signal to clients
//	if ((bytecount=send(bp->_hsock, (char *)(&bp->save_img_signal), sizeof(unsigned long), 0)) == SOCKET_ERROR) {
//		bp->_status = -1;
//		return 0;
//	}
//	if( bp->save_img_signal == 111)
//		bp->save_img_signal = 0;
//
//
//	//(13) send eye_center position
//	memcpy(bp->centerStream, bp->local_eye_center.data, 3*sizeof(short int));
//	if ((bytecount=send(bp->_hsock, bp->centerStream, 3 * sizeof(short int), 0)) == SOCKET_ERROR) {
//		bp->_status = -1;
//		return 0;
//	}
//
//
//	//(14) recieve eye_center from client
//	if ((bytecount = recv(bp->_hsock, bp->eye_center_buf, 3*sizeof(short int), 0))==SOCKET_ERROR) {
//		bp->_status = -1;
//		return 0;
//	}
//	bp->recieveEyeCenter( bp->eye_center_buf);

#ifdef DEBUG
	cv::meanStdDev(bp->_depthFrame,mean,std);
	fprintf(stderr,">> mean = %f, stdev = %f\n",mean[0],std[0]);
#endif

	return 0;


}


void RemoteKinect::setSaveSignal()
{
	save_img_signal = 111;
	calib_conn[0] = 1;
}


void RemoteKinect::setCalibConnectionSignal()
{
	calib_conn[0] = (calib_conn[0] != 0) ? 0 : 1;
	printf("calib_conn[0] = %d\n", calib_conn[0]);
	if(calib_conn[0] != 0)
	{
		calib_conn[1] = 0;  //make sure when the camera is capturing images,
		                    //the tracking is off.
	}
}

int RemoteKinect::getPathTrackingSignal()
{
	return calib_conn[1];
}

void RemoteKinect::setPathTrackingSignal(int x)
{
	if(x == 1)
	{
		if(calib_conn[0] == 0)
		{
			calib_conn[1] = x;
			calib_conn[2] = 1500;
			printf("Tracking start ...\n");

		}
	}
	else if(x == 0)
	{
		printf("before: %d, %d, %d\n", calib_conn[0],
			calib_conn[1], calib_conn[2]);
		calib_conn[1] = x;
		calib_conn[2] = 1;
		printf("Tracking is done\n");
		printf("calib_conn: %d, %d, %d\n", calib_conn[0],
			calib_conn[1], calib_conn[2]);
	}
}