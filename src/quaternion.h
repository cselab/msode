#pragma once

#include "types.h"
#include "math.h"
#include "log.h"

#include <cmath>
#include <iostream>

struct Quaternion
{
    Quaternion(real w, real3 u) :
        w(w), x(u.x), y(u.y), z(u.z)
    {}

    Quaternion(real w, real x, real y, real z) :
        w(w), x(x), y(y), z(z)
    {}

    // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    Quaternion(real3 v1, real3 v2) :
        Quaternion(std::sqrt(dot(v1, v1) * dot(v2, v2)) + dot(v1, v2), cross(v1, v2))
    {
        Expect(length(v1) > 0._r && length(v2) > 0._r, "vector length must be greater than zero");
        this->normalize();
        Ensure(length(rotate(v1)-v2) < 1e-6_r, "constructor from 2 vectors failed");
    }

    Quaternion(const Quaternion& q) = default;
    Quaternion& operator=(const Quaternion& q) = default;
    
    ~Quaternion() = default;

    inline real realPart() const {return w;}
    inline real3 vectorPart() const {return {x, y, z};}

    inline Quaternion conjugate() const {return {w, -x, -y, -z};}

    inline real norm() const {return std::sqrt(w*w + x*x + y*y + z*z);}

    inline Quaternion& normalize()
    {
        Expect(norm() > 0, "can not normalize zero quaternion");
        const real factor = 1.0_r / norm();
        return *this *= factor;
    }
    
    inline Quaternion normalized() const
    {
        Quaternion ret = *this;
        ret.normalize();
        return ret;
    }

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
        return {q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
                q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
                q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
                q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w};
    }
    
    inline Quaternion& operator*=(const Quaternion& q)
    {
        *this = (*this) * q;
        return *this;
    }

    inline real3 rotate(real3 v) const
    {
        Quaternion qv(0.0_r, v);
        const auto& q = *this;
        return (q * qv * q.conjugate()).vectorPart();
    }

    friend inline std::ostream& operator<<(std::ostream& stream, const Quaternion& q)
    {
        return stream << "[" << q.realPart() << ", " << q.vectorPart() << "]";
    }
    
    real w;        // real part
    real x, y, z;  // vector part
};
