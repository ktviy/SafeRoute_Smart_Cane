// Small 3-vector math, header only. No Arduino dependency.
#pragma once
#include <cmath>

namespace sr {

static const float kPi = 3.14159265358979f;
static const float kG = 9.80665f;

inline float deg2rad(float d) { return d * (kPi / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / kPi); }
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

struct Vec3 {
  float x, y, z;
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
  Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const {
    return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
  }
  float norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const {
    float n = norm();
    return n > 1e-9f ? *this * (1.0f / n) : Vec3(0, 0, 1);
  }
};

}  // namespace sr
