#ifndef ZHI_VECTOR3_H_
#define ZHI_VECTOR3_H_

#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
#include <sstream>


#define M_PI 3.14159265358979323846

#define Deg2Rad (M_PI * 2.f/360.f)
#define Rag2Deg (1.f / Deg2Rad)

#define kEpsilonNormalSqrt 1e-15
#define kEpsilon 1e-5

struct alignas(float) vector3
{
	float x;
	float y;
	float z;

	vector3() : x(0), y(0), z(0) { }

	vector3(const float vx, const float vy, const float vz) : x(vx), y(vy), z(vz) {}

	vector3(const float vx, const float vy) : x(vx), y(vy), z(0) {}

	vector3(const vector3& v) : x(v.x), y(v.y), z(v.z) {}

	void setzero() {
		x = y = z = 0;
	}

	void setConstant(float f) {
		x = f;
		y = f;
		z = f;
	}

	static vector3 zero() {
		return vector3(0, 0, 0);
	}

	vector3 operator + (const vector3& v) const {
		return vector3(x + v.x, y + v.y, z + v.z);
	}

	vector3 operator - (const vector3& v) const {
		return vector3(x - v.x, y - v.y, z - v.z);
	}

	vector3 operator / (const vector3& v) const {
		if (fabsf(v.x) <= kEpsilon || fabsf(v.y) <= kEpsilon || fabsf(v.z) <= kEpsilon) {
			std::cerr << "Over flow!\n";
			return *this;
		}
		return vector3(x / v.x, y / v.y, z / v.z);
	}

	vector3 operator * (const vector3& v) const {
		return vector3(x * v.x, y * v.y, z * v.z);
	}

	vector3 operator + (const float f) const {
		return vector3(x + f, y + f, z + f);
	}

	vector3 operator - (const float f) const {
		return vector3(x - f, y - f, z - f);
	}

	vector3 operator / (const float f) const {
		if (fabsf(f) < kEpsilon) {
			std::cerr << "Over flow!\n";
			return *this;
		}
		return vector3(x / f, y / f, z / f);
	}

	vector3 operator * (const float f) const {
		return vector3(x * f, y * f, z * f);
	}

	vector3 operator -() const {
		return vector3(-x, -y, -z);
	}

	vector3& operator += (const vector3& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	vector3& operator -= (const vector3& v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	vector3& operator *= (const float v) {
		x *= v;
		y *= v;
		z *= v;
		return *this;
	}

	vector3& operator /= (const float v) {
		x /= v;
		y /= v;
		z /= v;
		return *this;
	}

	vector3& operator  = (const vector3& v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	bool operator == (const vector3& v)const {
		return (x == v.x) && (y == v.y) && (z == v.z);
	}

	bool operator != (const vector3& v)const {
		return x != v.x || y != v.y || z != v.z;
	}

	float& operator [](int idx) {
		switch (idx) {
		case 0:
			return this->x;
		case 1:
			return this->y;
		case 2:
			return this->z;
		default:
			std::cerr << "vector index out of range!";
			break;
		}
	}

	int size() {
		return 3;
	}

	vector3 cwiseMin(const vector3& v) {
		float retx = (std::min)(x, v.x);
		float rety = (std::min)(y, v.y);
		float retz = (std::min)(z, v.z);

		return vector3(retx, rety, retz);
	}

	vector3 cwiseMax(const vector3& v) {
		float retx = (std::max)(x, v.x);
		float rety = (std::max)(y, v.y);
		float retz = (std::max)(z, v.z);

		return vector3(retx, rety, retz);
	}

	float magnitude() const {
		return sqrt(x * x + y * y + z * z);
	}

	vector3& normalize() {
		float mag = magnitude();
		if (mag > kEpsilon) {
			*this /= mag;
		}
		else {
			this->zero();
		}
		return *this;
	}

	float sqrMagnitude() const {
		return x * x + y * y + z * z;
	}

	std::string ToString() const {
		std::stringstream ss;
		ss << "( " << x << ", " << y << ", " << z << " )";
		std::string s = ss.str();
		return s;
	}

	static float Dot(const vector3& a, const vector3& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static vector3 Cross(const vector3& left, const vector3& right) {
		float x = left.y * right.z - left.z * right.y;
		float y = left.z * right.x - left.x * right.z;
		float z = left.x * right.y - right.y * left.x;
		return vector3(x, y, z);
	}

	static float Angle(const vector3& from, const vector3& to) {
		float denominator = (float)sqrt(from.sqrMagnitude() * to.sqrMagnitude());

		if (denominator < kEpsilonNormalSqrt)
			return 0.f;
		float dot = std::clamp(Dot(from, to) / denominator, -1.f, 1.f);

		return (float)acos(dot) * Rag2Deg;
	}

	static float Distance(const vector3& a, const vector3& b) {
		float diff_x = a.x - b.x;
		float diff_y = a.y - b.y;
		float diff_z = a.z - b.z;

		return sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
	}

	static vector3 SmoothDamp(vector3 current, vector3 target, vector3& currentVelocity, float smoothTime, float deltaTime) {
		return SmoothDamp(current, target, currentVelocity, smoothTime, INFINITY, deltaTime);
	}

	static vector3 SmoothDamp(vector3 current, vector3 target, vector3& currentVelocity, float smoothTime, float maxSpeed, float deltaTime) {
		float output_x = 0.f;
		float output_y = 0.f;
		float output_z = 0.f;

		smoothTime = (std::max)(0.0001f, smoothTime);
		float omega = 2.f / smoothTime;

		float x = omega * deltaTime;
		float exp = 1.f / (1.f + x + 0.48f * x * x + 0.235f * x * x * x);

		float change_x = current.x - target.x;
		float change_y = current.y - target.y;
		float change_z = current.z - target.z;
		vector3 originalTo = target;

		float maxChange = maxSpeed * smoothTime;

		float maxChangeSq = maxChange * maxChange;
		float sqrmag = change_x * change_x + change_y * change_y + change_z * change_z;
		if (sqrmag > maxChangeSq)
		{
			auto mag = sqrtf(sqrmag);
			change_x = change_x / mag * maxChange;
			change_y = change_y / mag * maxChange;
			change_z = change_z / mag * maxChange;
		}

		target.x = current.x - change_x;
		target.y = current.y - change_y;
		target.z = current.z - change_z;

		float temp_x = (currentVelocity.x + omega * change_x) * deltaTime;
		float temp_y = (currentVelocity.y + omega * change_y) * deltaTime;
		float temp_z = (currentVelocity.z + omega * change_z) * deltaTime;

		currentVelocity.x = (currentVelocity.x - omega * temp_x) * exp;
		currentVelocity.y = (currentVelocity.y - omega * temp_y) * exp;
		currentVelocity.z = (currentVelocity.z - omega * temp_z) * exp;

		output_x = target.x + (change_x + temp_x) * exp;
		output_y = target.y + (change_y + temp_y) * exp;
		output_z = target.z + (change_z + temp_z) * exp;

		float origMinusCurrent_x = originalTo.x - current.x;
		float origMinusCurrent_y = originalTo.y - current.y;
		float origMinusCurrent_z = originalTo.z - current.z;

		float outMinusOrig_x = output_x - originalTo.x;
		float outMinusOrig_y = output_y - originalTo.y;
		float outMinusOrig_z = output_z - originalTo.z;

		if (origMinusCurrent_x * outMinusOrig_x + origMinusCurrent_y * outMinusOrig_y + origMinusCurrent_z * outMinusOrig_z > 0)
		{
			output_x = originalTo.x;
			output_y = originalTo.y;
			output_z = originalTo.z;

			currentVelocity.x = (output_x - originalTo.x) / deltaTime;
			currentVelocity.y = (output_y - originalTo.y) / deltaTime;
			currentVelocity.z = (output_z - originalTo.z) / deltaTime;
		}

		return vector3(output_x, output_y, output_z);
	}

	void printvec() {
		std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
	}
};
#endif 


