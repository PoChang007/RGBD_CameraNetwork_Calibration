## Programming Details

### Loops, Functions, I/O

* The `SphereTracking` class consists of several functions for sphere center detection: `LoadImage`, `Compute3Dpoints`, `ComputeSpehereCenter`, `SphereFitting`, etc. In `ThreeDViewerClass`, `if else` is used in `DrawSphereIn3DSpace` function to draw the sphere center in different color from different color based on the given index 
* Color and depth images are read for sphere center detection and 3D point cloud rendering. The estimated sphere center locations and transformation matrices are written into .txt. files. See [File Structure section](#File-Structure)

### Object Oriented Programming

* All class data members are explicitly specified as public or private. Some private data can be obtained by using public getter function
* `SphereTracking` class constructor utilizes member initialization lists
* Functions and data related to sphere tracking are grouped into `SphereTracking` class. Invariant data `_estimated_radius` is hidden from the user `CalibrationHandler`

### Memory Management

* In `ThreeDViewer` class, functions `DrawSphereCenterIn2DImage` and `DrawSphereIn3DSpace` use pass-by-reference for arguments `cam_index` and `frameCounter`
* All of the classes in the project use destructors to deallocate OpenCV matrix
* In CalibrationHandler.cpp, the function `StartProcessing` uses std::move to move the smart pointer (to `SphereTracking`) to the thread
* The smart pointers are used in `rgbdCameraNetwork.cpp` and `CalibrationHandler.cpp` for 3D scene rendering handler and camera calibration handler

### Concurrency

* `std::async` is used in `CalibrationHandler::StartProcessing()` to detect sphere center based on RGB-D images captured from each camera. The images in each camera will be processed in different thread 
* When a thread finishes the task of sphere center detection, the function `CalibrationHandler::StoreClientInfo` is called and will store the necessary data in `CalibrationHandler::cameras` for the use of extrinsic estimation, and then print the detection complete message. A lock guard is used to prevent data race between threads if they try to save the data into `CalibrationHandler::cameras` at the same time