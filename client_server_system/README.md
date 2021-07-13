# Client-server camera network calibration system

For research purpose only.

Environment:

  * Windows 10
  * Visual Studio 2013 (release mode)
  * [OpenCV 2.4.10](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.10/) 
  * [Kinect v2 SDK](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
  * [OpenGL](INSTALLATION.md)

For camera calibration, a sphere ball in yellow color is required. the default radius of the sphere ball is 20.3(cm).

By default 4 cameras are used. The number of cameras used in the camera network is scalable.

Each camera is connected to a PC (client). The PC used as a server can also be connected to a client (the reference camera, usually the first one).

## Usage

Execute each client first (ctrl+F5), the client will wait for the signal sent from server.

### RemoteKinect Constructor (server)

For camera network calibration, set calib_conn[2] = 1500  
For point cloud visualization, set calib_conn[2] = 1

### Other Settings

server `data.h`:

Enter the ip address for each client.

client `OpenCV_KinectSDK.h`:

By default sphere's estimated_radius = 203 (mm)

### Calibration Steps:

1. After each client gets ready, execute server side (ctrl+F5). Each client will start capturing RGB-D images.
1. Continue waving the ball around the capturing area of the camera network for about 30secs - 1min
1. Press `c` to stop capturing images and then press `t` to start calibrating each camera
1. The camera pose for each camera is recorded and the calibration scene can be rendered