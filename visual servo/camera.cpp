#include "camera.h"

Camera::Camera() {}

Vector3d Camera::world2camera(const Vector3d& p_w)
{
	Vector4d p_w_q{ p_w(0,0),p_w(1,0),p_w(2,0),1 };
	Vector4d p_c_q = exMatrix_ * p_w_q;
	return Vector3d{ p_c_q(0,0),p_c_q(1,0),p_c_q(2,0) };
}

Vector3d Camera::camera2world(const Vector3d& p_c)
{
	Vector4d p_c_q{ p_c(0,0),p_c(1,0),p_c(2,0),1 };
	Vector4d p_w_q = exMatrix_.inverse() * p_c_q;
	return Vector3d{ p_w_q(0,0),p_w_q(1,0),p_w_q(2,0) };
}

Vector2d Camera::camera2pixel(const Vector3d& p_c)
{
	return Vector2d(
		fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
		fy_ * p_c(1, 0) / p_c(2, 0) + cy_
	);
}

Vector3d Camera::pixel2camera(const Vector2d& p_p)
{
	double depth = calDepth(p_p);
	return Vector3d(
		(p_p(0, 0) - cx_) *depth / fx_,
		(p_p(1, 0) - cy_) *depth / fy_,
		depth
	);
}

Vector2d Camera::world2pixel(const Vector3d& p_w)
{
	return camera2pixel(world2camera(p_w));
}

Vector3d Camera::pixel2world(const Vector2d& p_p)
{
	return camera2world(pixel2camera(p_p));
}

double Camera::calDepth(const Vector2d& p_p)
{
	Vector3d p_p_q{ p_p(0,0),p_p(1,0),1 };
	Vector3d rightMatrix = R_.inverse() * inMatrix_.inverse() * p_p_q;
	Vector3d leftMatrix = R_.inverse() * t_;
	return leftMatrix(2, 0) / rightMatrix(2, 0);
}

void Camera::setInternalParams(double fx, double cx, double fy, double cy)
{
	fx_ = fx;
	cx_ = cx;
	fy_ = fy;
	cy_ = cy;

	inMatrix_ << fx, 0, cx,
		0, fy, cy,
		0, 0, 1;
}

void Camera::setExternalParams(Quaterniond Q, Vector3d t)
{
	Q_ = Q;
	R_ = Q.normalized().toRotationMatrix();
	setExternalParams(R_, t);
}

void Camera::setExternalParams(Matrix<double, 3, 3>  R, Vector3d t)
{
	t_ = t;
	R_ = R;

	exMatrix_ << R_(0, 0), R_(0, 1), R_(0, 2), t(0, 0),
		R_(1, 0), R_(1, 1), R_(1, 2), t(1, 0),
		R_(2, 0), R_(2, 1), R_(2, 2), t(2, 0),
		0, 0, 0, 1;
}
