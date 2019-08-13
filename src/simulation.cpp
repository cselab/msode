#include "simulation.h"
#include "math.h"

#include <tuple>

Simulation::Simulation(const RigidBody& initialRB,
                       const MagneticField& initialMF) :
    rigidBody(initialRB),
    magneticField(initialMF)
{}

real3 operator*(const PropulsionMatrix::SubMatrix& A, const real3& v)
{
    return {A[0] * v.x,
            A[1] * v.y,
            A[2] * v.z};
}

// TODO rotate Matrix
static inline std::tuple<real3, real3> computeVelocities(const PropulsionMatrix& m,
                                                         const real3& F, const real3& T)
{
    const real3 v = m.A * F + m.B * T;
    const real3 w = m.B * F + m.C * T;
    return {v, w};
}

void Simulation::advance(long nsteps, real dt)
{
    for (long step = 0; step < nsteps; ++step)
    {
        auto q = rigidBody.q;
        
        const real3 B      = magneticField(t);
        const real3 m      = q.conjugate().rotate(rigidBody.magnMoment);
        const real3 torque = cross(m, B);
        constexpr real3 force {0.0_r, 0.0_r, 0.0_r};
        
        std::tie(rigidBody.v, rigidBody.omega) =
            computeVelocities(rigidBody.propulsion, force, torque);
        
        const Quaternion _omega (0.0_r, rigidBody.omega);
        const auto dq_dt = 0.5_r * q * _omega;
        
        rigidBody.r += dt * rigidBody.v;
        q += dt * dq_dt;
        
        rigidBody.q = q.normalized();
        
        t += dt;

        // std::cout << "t = " << t << " : " << B << " " << rigidBody << std::endl;
        std::cout << t << " " << rigidBody.r.x << " " << rigidBody.r.y << " " << rigidBody.r.z << std::endl;

        // std::cout << t << " " << m << " " << B << " " << torque << std::endl;
    }
}

std::ostream& operator<<(std::ostream& stream, const RigidBody& b)
{
    return stream << "q = " << b.q << ", r = " << b.r << ", omega = " << b.omega;
}
