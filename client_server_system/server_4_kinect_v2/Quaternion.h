// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#ifndef __QUATERNION__
#define __QUATERNION__

#include <math.h>
#define TOLERANCE 0.00001f
#define PI 3.141592653589793238462643383279502884197169399375105820974944592

struct pt3D
{
	float x, y, z;
	pt3D()
	{
		x = y = z = 0.0f;
	}
	pt3D(const pt3D &pt)
	{
		x = pt.x;
		y = pt.y;
		z = pt.z;
	}
	pt3D(float a, float b, float c)
	{
		x = a;
		y = b;
		z = c;
	}

	void normalize()
	{
		float mag2 = x * x + y * y + z * z;
		if (fabs(mag2) > TOLERANCE && fabs(mag2 - 1.0f) > TOLERANCE)
		{
			float mag = sqrt(mag2);
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
};

struct Quaternion
{
	float x, y, z, w;
	Quaternion()
	{
		x = 0.0f;
		y = z = 0.0f;
		w = 1.0f;
	}
	Quaternion(float a, float b, float c, float d)
	{
		x = a;
		y = b;
		z = c;
		w = d;
	}
	void normalize()
	{
		float mag2 = w * w + x * x + y * y + z * z;
		if (fabs(mag2) > TOLERANCE && fabs(mag2 - 1.0f) > TOLERANCE)
		{
			float mag = sqrt(mag2);
			x /= mag;
			y /= mag;
			z /= mag;
			w /= mag;
		}
	}
	Quaternion getConjugate()
	{
		return Quaternion(-x, -y, -z, w);
	}

	Quaternion Quaternion::operator*(const Quaternion &rq)
	{
		// the constructor takes its arguments as (x, y, z, w)
		return Quaternion(w * rq.x + x * rq.w + y * rq.z - z * rq.y,
						  w * rq.y + y * rq.w + z * rq.x - x * rq.z,
						  w * rq.z + z * rq.w + x * rq.y - y * rq.x,
						  w * rq.w - x * rq.x - y * rq.y - z * rq.z);
	}

	// Multiplying a quaternion q with a vector v applies the q-rotation to v
	pt3D Quaternion::operator*(const pt3D &pt)
	{
		pt3D vn(pt);
		vn.normalize();

		Quaternion vecQuat, resQuat;
		vecQuat.x = vn.x;
		vecQuat.y = vn.y;
		vecQuat.z = vn.z;
		vecQuat.w = 0.0f;

		resQuat = vecQuat * getConjugate();
		resQuat = *this * resQuat;

		return (pt3D(resQuat.x, resQuat.y, resQuat.z));
	}

	// Convert from Axis Angle
	void Quaternion::FromAxis(const pt3D &v, float angle)
	{
		float sinAngle;
		angle *= 0.5f;
		pt3D vn(v);
		vn.normalize();

		sinAngle = sin(angle);

		x = (vn.x * sinAngle);
		y = (vn.y * sinAngle);
		z = (vn.z * sinAngle);
		w = cos(angle);
	}

	// Convert to Axis/Angles
	void Quaternion::getAxisAngle(pt3D &axis, float &angle)
	{
		float scale = sqrt(x * x + y * y + z * z);
		if (fabs(scale) < TOLERANCE)
		{
			axis.x = 1.0f;
			axis.y = 0.0f;
			axis.z = 0.0f;
			angle = 0.0f;
		}
		else
		{
			axis.x = x / scale;
			axis.y = y / scale;
			axis.z = z / scale;
			angle = acos(w) * 2.0f;
		}
	}

	// Convert to Matrix
	void Quaternion::getMatrix3(float *M)
	{
		normalize();
		float x2 = x * x;
		float y2 = y * y;
		float z2 = z * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;

		// This calculation would be a lot more complicated for non-unit length quaternions
		// Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
		//   OpenGL
		M[0] = 1.0f - 2.0f * (y2 + z2);
		M[3] = 2.0f * (xy - wz);
		M[6] = 2.0f * (xz + wy);

		M[1] = 2.0f * (xy + wz);
		M[4] = 1.0f - 2.0f * (x2 + z2);
		M[7] = 2.0f * (yz - wx);

		M[2] = 2.0f * (xz - wy);
		M[5] = 2.0f * (yz + wx);
		M[8] = 1.0f - 2.0f * (x2 + y2);
	}

	// Convert to Matrix
	void Quaternion::getMatrix4(float *M)
	{
		normalize();
		float x2 = x * x;
		float y2 = y * y;
		float z2 = z * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;

		// This calculation would be a lot more complicated for non-unit length quaternions
		// Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by OpenGL
		M[0] = 1.0f - 2.0f * (y2 + z2);
		M[4] = 2.0f * (xy - wz);
		M[8] = 2.0f * (xz + wy);
		M[12] = 0.0f;

		M[1] = 2.0f * (xy + wz);
		M[5] = 1.0f - 2.0f * (x2 + z2);
		M[9] = 2.0f * (yz - wx);
		M[13] = 0.0f;

		M[2] = 2.0f * (xz - wy);
		M[6] = 2.0f * (yz + wx);
		M[10] = 1.0f - 2.0f * (x2 + y2);
		M[14] = 0.0f;

		M[3] = M[7] = M[11] = 0.0f;
		M[15] = 1.0f;
	}
};

#endif