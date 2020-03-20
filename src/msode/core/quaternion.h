#pragma once

#include "types.h"
#include "math.h"
#include "log.h"

#include <array>
#include <cmath>
#include <iostream>

namespace msode
{

using RotMatrix = std::array<std::array<real, 3>, 3>; 

struct Quaternion
{
    static inline Quaternion createIdentity()
    {
        return {0.0_r, 1.0_r, 0.0_r, 0.0_r};
    }
    
    static inline Quaternion createFromComponents(real w, real x, real y, real z)
    {
        return {w, x, y, z};
    }

    static inline Quaternion createFromComponents(real w, real3 v)
    {
        return {w, v};
    }

    static inline Quaternion createPureScalar(real w)
    {
        return {w, 0.0_r, 0.0_r, 0.0_r};
    }

    static inline Quaternion createPureVector(real3 v)
    {
        return {0.0_r, v};
    }

    static inline Quaternion createFromRotation(real angle, real3 axis)
    {
        const real alpha = 0.5_r * angle;
        const real3 u = msode::normalized(axis);
        return {std::cos(alpha), std::sin(alpha) * u};
    }


    // https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    static inline Quaternion createFromMatrix(const RotMatrix& R)
    {
        auto makeQ = [](real t, real w, real x, real y, real z)
        {
            const auto q = createFromComponents(w, x, y, z);
            return q * (0.5_r / std::sqrt(t));
        };
        
        if (R[2][2] < 0.0_r)
        {
            if (R[0][0] > R[1][1])
            {
                const real t = 1.0_r + R[0][0] - R[1][1] - R[2][2];
                return makeQ(t, R[2][1] - R[1][2], t, R[1][0] + R[0][1], R[0][2] + R[2][0]);
            }
            else
            {
                const real t = 1.0_r - R[0][0] + R[1][1] - R[2][2];
                return makeQ(t, R[0][2] - R[2][0], R[1][0] + R[0][1], t, R[2][1] + R[1][2]);
            }
        }
        else
        {
            if (R[0][0] < -R[1][1])
            {
                const real t = 1.0_r - R[0][0] - R[1][1] + R[2][2];
                return makeQ(t, R[1][0] - R[0][1], R[0][2] + R[2][0], R[2][1] + R[1][2], t);
            }
            else
            {
                const real t = 1.0_r + R[0][0] + R[1][1] + R[2][2];
                return makeQ(t, t, R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]);
            }
        }
    }

    static inline Quaternion createFromVectors(real3 from, real3 to)
    {
        return {from, to};
    }
    
    Quaternion() = default; // create an UNINITIALIZED quaternion
    Quaternion(const Quaternion& q) = default;
    Quaternion& operator=(const Quaternion& q) = default;
    
    ~Quaternion() = default;

    real realPart() const {return w;}
    real3 vectorPart() const {return {x, y, z};}

    RotMatrix getRotationMatrix() const
    {
        const std::array<real, 3> row0 {1.0_r - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w};
        const std::array<real, 3> row1 {2*x*y + 2*z*w, 1.0_r - 2*x*x - 2*z*z, 2*y*z - 2*x*w};
        const std::array<real, 3> row2 {2*x*z - 2*y*w, 2*y*z + 2*x*w, 1.0_r - 2*x*x - 2*y*y};
        return {row0, row1, row2};
    }

    Quaternion conjugate() const {return {w, -x, -y, -z};}

    real norm() const {return std::sqrt(w*w + x*x + y*y + z*z);}

    Quaternion& normalize()
    {
        MSODE_Expect(norm() > 0, "can not normalize zero quaternion");
        const real factor = 1.0_r / norm();
        return *this *= factor;
    }
    
    Quaternion normalized() const
    {
        Quaternion ret = *this;
        ret.normalize();
        return ret;
    }

    Quaternion& operator+=(const Quaternion& q)
    {
        x += q.x;
        y += q.y;
        z += q.z;
        w += q.w;
        return *this;
    }

    Quaternion& operator-=(const Quaternion& q)
    {
        x -= q.x;
        y -= q.y;
        z -= q.z;
        w -= q.w;
        return *this;
    }

    Quaternion& operator*=(real a)
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
    
    Quaternion& operator*=(const Quaternion& q)
    {
        *this = (*this) * q;
        return *this;
    }

    real3 rotate(real3 v) const
    {
        Quaternion qv(0.0_r, v);
        const auto& q = *this;
        return (q * qv * q.conjugate()).vectorPart();
    }

    real3 inverseRotate(real3 v) const
    {
        Quaternion qv(0.0_r, v);
        const auto& q = *this;
        return (q.conjugate() * qv * q).vectorPart();
    }

    friend inline std::ostream& operator<<(std::ostream& stream, const Quaternion& q)
    {
        return stream << q.realPart() << " " << q.vectorPart();
    }
    
    real w;       // real part
    real x, y, z; // vector part

private:
    Quaternion(real w_, real x_, real y_, real z_) :
        w(w_), x(x_), y(y_), z(z_)
    {}

    Quaternion(real w_, real3 u) :
        w(w_), x(u.x), y(u.y), z(u.z)
    {}
    
    // https://stackoverflow.com/a/11741520/11630848
    Quaternion(real3 u, real3 v)
    {
        MSODE_Expect(length(u) > 0._r && length(v) > 0._r, "vector length must be greater than zero");

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
        MSODE_Ensure(length(rotate(u)-v) < 1e-6_r, "constructor from 2 vectors failed by %g", length(rotate(u)-v));
    }
};

} // namespace msode
