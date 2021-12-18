#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::Matrix;

class Camera
{
public:
	Camera();

	// coordinate transform: world, camera, pixel
	Vector3d world2camera(const Vector3d& p_w);
	Vector3d camera2world(const Vector3d& p_c);
	Vector2d camera2pixel(const Vector3d& p_c);
	Vector3d pixel2camera(const Vector2d& p_p);
	Vector3d pixel2world(const Vector2d& p_p);
	Vector2d world2pixel(const Vector3d& p_w);

	// set params
	void setInternalParams(double fx, double cx, double fy, double cy);
	void setExternalParams(Quaterniond Q, Vector3d t);
	void setExternalParams(Matrix<double, 3, 3>  R, Vector3d t);

	// cal depth
	double calDepth(const Vector2d& p_p);

private:
	// 内参
	double fx_, fy_, cx_, cy_, depth_scale_;
	Matrix<double, 3, 3> inMatrix_;

	// 外参
	Quaterniond Q_;
	Matrix<double, 3, 3>  R_;
	Vector3d t_;
	Matrix<double, 4, 4> exMatrix_;
};

#endif // CAMERA_H
