## Documentation

### [SphereTracking](./src/sphereTracking.cpp)

* `LoadImage`: load pre-stored images captured by the camera `n` in the camera network
* `SphereCenterDetection`: find the sphere region in color image, then back-project corresponding pixels in depth image to 3D space for sphere center detection

### [CalibrationHandler](./src/calibrationHandler.cpp)

* `StartProcessing`: start the sphere tracking process asynchronously with each image sequence
* `CalculateCameraExtrinsics`: calculate extrinsics between camera `n` and the reference camera
* `LoadPath`: load estimated sphere center positions
* `CorrespondenceBuild`: get the corresponding pairs between two cameras

### [ThreeDViewer](./src/threeDViewer.cpp)

* `SetUpPointClouds`: transform 3D points captured by camera `n` to the world coordinate
* `DrawCameraFrustum`: draw the frustum of camera `n` in the world coordinate
* `DrawSphereCenterIn2DImage`: draw the sphere center position in 2D image
* `DrawSphereIn3DSpace`: draw the sphere center position in the world coordinate. Sphere centers drawn in different colors represents they are from different cameras

### Others

* Color and depth images are used in sphere center detection and 3D point cloud rendering. The estimated sphere center positions and transformation matrices are written into .txt. files. See [File Structure section](./README.md#file-structure)
* The default value of the sphere radius (`_estimated_radius`) is 203cm
