#include "simulation.h"
#include "math.h"

#include <tuple>

Simulation::Simulation(const std::vector<RigidBody>& initialRBs,
                       const MagneticField& initialMF) :
    rigidBodies(initialRBs),
    magneticField(initialMF)
{}

real3 operator*(const PropulsionMatrix::SubMatrix& A, const real3& v)
{
    return {A[0] * v.x,
            A[1] * v.y,
            A[2] * v.z};
}

static inline std::tuple<real3, real3> computeVelocities(const PropulsionMatrix& m,
                                                         const real3& F, const real3& T)
{
    const real3 v = m.A * F + m.B * T;
    const real3 w = m.B * F + m.C * T;
    return {v, w};
}

void Simulation::activateDump(const std::string& fname, long dumpEvery)
{
    Expect(dumpEvery > 0, "expect positive dumpEvery");
    this->dumpEvery = dumpEvery;
    file.open(fname);
    Ensure(file.is_open(), "could not open file for writing");
}

void Simulation::run(long nsteps, real dt)
{
    for (long step = 0; step < nsteps; ++step)
    {
        if (timeStep % dumpEvery == 0) dump();
        advance(dt);
    }
}

void Simulation::advance(real dt)
{
    const real3 B = magneticField(t);
    
    for (auto& rigidBody : rigidBodies)
    {
        auto q    = rigidBody.q;
        auto qInv = q.conjugate();
        
        const real3 m      = qInv.rotate(rigidBody.magnMoment);
        const real3 torque = cross(m, B);
        constexpr real3 force {0.0_r, 0.0_r, 0.0_r};
        
        std::tie(rigidBody.v, rigidBody.omega) = computeVelocities(rigidBody.propulsion,
                                                                   q.rotate(force),
                                                                   q.rotate(torque));

        rigidBody.v     = qInv.rotate(rigidBody.v    );
        rigidBody.omega = qInv.rotate(rigidBody.omega);
        
        const Quaternion _omega (0.0_r, rigidBody.omega);
        const auto dq_dt = 0.5_r * q * _omega;
        
        rigidBody.r += dt * rigidBody.v;
        q += dt * dq_dt;
        
        rigidBody.q = q.normalized();
    }
    
    t += dt;
    ++timeStep;
}

void Simulation::dump()
{
    file << t;
    for (const auto& rigidBody : rigidBodies)
        file << " " << rigidBody;
    file << "\n";
}

std::ostream& operator<<(std::ostream& stream, const RigidBody& b)
{
    return stream << b.q << ' ' << b.r << ' ' << b.omega;
}

std::ostream& operator<<(std::ostream& stream, const PropulsionMatrix& m)
{
    return stream << m.A[0] << ' ' << m.A[1] << ' ' << m.A[0] << '\n'
                  << m.B[0] << ' ' << m.B[1] << ' ' << m.B[1] << '\n'
                  << m.C[0] << ' ' << m.C[1] << ' ' << m.C[2] << '\n';
}
