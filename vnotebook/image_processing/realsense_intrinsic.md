# realsense_camera_parameters

-----

## Intrinsic Camera Parameters
[参考](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#extrinsic-camera-parameters)

-----

The relationship between a stream's 2D and 3D coordinate systems is described by its intrinsic camera parameters

相机内参保存在结构体 `rs2_intrinsics`中

* The `fx` and `fy` fields describe the **focal length** of the image  焦距

* The `ppx` and `ppy` fields describe the pixel coordinates of the principal point (center of projection)

*The center of projection is not necessarily the center of the image*


* The `model` field describes which of several supported distortion models was used to calibrate the image

* the `coeffs` field provides an array of up to five coefficients describing the distortion model


Intrinsic parameters can be retrieved from any `rs2::video_stream_profile` object via a call to `get_intrinsics()`

```cpp

auto video_stream = stream.as<rs2::video_stream_profile>();
rs2_intrinsics intrinsics = video_stream.get_intrinsics();
auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
rs2_distortion model = intrinsics.model;

std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
std::cout << "Distortion Model        : " << model << std::endl;
std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
    intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
```

-----

##  Extrinsic Camera Parameters
The relationship between the separate 3D coordinate systems of separate streams is described by their extrinsic parameters

相机外参保存在结构体 `rs2_extrinsics`中

* The `translation` field contains the 3D translation between the imager's physical positions, specified in **meters**

* The `rotation` field contains a 3x3 orthonormal rotation matrix between the imager's physical orientations

* All 3D coordinate systems are specified in **meters**