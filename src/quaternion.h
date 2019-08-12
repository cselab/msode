#pragma once

#include "types.h"

struct Quaternion
{
    Quaternion(real w, real3 u) :
        w(w), x(u.x), y(u.y), z(u.z)
    {}

    Quaternion(real w, real x, real y, real z) :
        w(w), x(x), y(y), z(z)
    {}

    inline Quaternion& operator+=(const Quaternion& q)
    {
        x += q.x;
        y += q.y;
        z += q.z;
        w += q.w;
        return *this;
    }

    inline Quaternion& operator-=(const Quaternion& q)
    {
        x -= q.x;
        y -= q.y;
        z -= q.z;
        w -= q.w;
        return *this;
    }

    inline Quaternion& operator*=(real a)
    {
        x *= a;
        y *= a;
        z *= a;
        w *= a;
        return *this;
    }

    friend inline Quaternion operator+(Quaternion q1, const Quaternion& q2) {q1 += q2; return q1;}
    friend inline Quaternion operator-(Quaternion q1, const Quaternion& q2) {q1 -= q2; return q1;}

    friend inline Quaternion operator*(real a, Quaternion q) {q *= a; return q;}
    friend inline Quaternion operator*(Quaternion q, real a) {return a * q;}

    friend inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
    {
        return {q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
                q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
                q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
                q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w};
    }
    
    inline Quaternion& operator*=(const Quaternion& q)
    {
        *this = (*this) * q;
        return *this;
    }

    real w;        // real part
    real x, y, z;  // vector part
};
