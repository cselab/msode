#pragma once

#include "types.h"
#include "math.h"
#include "log.h"

#include <cmath>
#include <iostream>

struct Quaternion
{
    static inline Quaternion createFromComponents(real w, real x, real y, real z)
    {
        return {w, x, y, z};
    }

    static inline Quaternion createFromComponents(real w, real3 v)
    {
        return {w, v};
    }

    static inline Quaternion createFromRotation(real angle, real3 axis)
    {
        const real alpha = 0.5_r * angle;
        const real3 u = ::normalized(axis);
        return {std::cos(alpha), std::sin(alpha) * u};
    }

    static inline Quaternion createFromVectors(real3 from, real3 to)
    {
        return {from, to};
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
        return stream << q.realPart() << " " << q.vectorPart();
    }
    
    real w {0.0_r}; // real part
    real x {0.0_r}; // vector part
    real y {0.0_r};
    real z {0.0_r};

private:
    Quaternion(real w, real x, real y, real z) :
        w(w), x(x), y(y), z(z)
    {}

    Quaternion(real w, real3 u) :
        w(w), x(u.x), y(u.y), z(u.z)
    {}

    static inline real3 anyOrthogonal(real3 v)
    {
        const real x = std::abs(v.x);
        const real y = std::abs(v.y);
        const real z = std::abs(v.z);

        constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
        constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
        constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};
        
        const real3 other = x < y ? (x < z ? ex : ez) : (y < z ? ey : ez);
        return cross(v, other);
    }
    
    // https://stackoverflow.com/a/11741520/11630848
    Quaternion(real3 u, real3 v)
    {
        constexpr real tolerance = 1e-6_r;
        Expect(length(u) > 0._r && length(v) > 0._r, "vector length must be greater than zero");

        const real k_cos_theta = dot(u, v);
        const real k = std::sqrt(dot(u, u) * dot(v, v));

        if (std::abs(k_cos_theta + k) == 0.0_r) // opposite directions
        {
            w = 0.0_r;
            const real3 n = anyOrthogonal(u);
            x = n.x;
            y = n.y;
            z = n.z;
        }
        else
        {
            w = k_cos_theta + k;
            const real3 n = cross(u, v);
            x = n.x;
            y = n.y;
            z = n.z;
        }
        this->normalize();
        Ensure(length(rotate(u)-v) < tolerance, "constructor from 2 vectors failed by " + std::to_string(length(rotate(u)-v)));
    }
};
