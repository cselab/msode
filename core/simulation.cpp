#include "simulation.h"
#include "math.h"

#include <tuple>

namespace msode
{

Simulation::Simulation(const std::vector<RigidBody>& initialRBs,
                       const MagneticField& initialMF) :
    rigidBodies(initialRBs),
    magneticField(initialMF)
{}

static inline real3 operator*(const PropulsionMatrix::SubMatrix& A, const real3& v)
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

void Simulation::reset(const std::vector<RigidBody>& initialRBs, const MagneticField& initialMF)
{
    currentTimeStep = 0;
    currentTime     = 0._r;
    rigidBodies = initialRBs;
    magneticField = initialMF;
}

void Simulation::activateDump(const std::string& fname, long dumpEvery)
{
    MSODE_Expect(dumpEvery > 0, "expect positive dumpEvery");
    this->dumpEvery = dumpEvery;

    if (file.is_open())
        file.close();

    file.open(fname);
    MSODE_Ensure(file.is_open(), "could not open file for writing");
}

void Simulation::run(long nsteps, real dt)
{
    MSODE_Expect(nsteps > 0, "expect positive number of steps");
    MSODE_Expect(dt > 0._r, "expect positive time step");
    
    for (long step = 0; step < nsteps; ++step)
        advance(dt);
}

void Simulation::advance(real dt)
{
    if (file.is_open() && currentTimeStep % dumpEvery == 0)
        dump();
    
    magneticField.advance(currentTime, dt);
    const real3 B = magneticField(currentTime);
    
    for (auto& rigidBody : rigidBodies)
    {
        Quaternion q = rigidBody.q;
        const Quaternion qInv = q.conjugate();
        
        const real3 m      = qInv.rotate(rigidBody.magnMoment);
        const real3 torque = cross(m, B);
        constexpr real3 force {0.0_r, 0.0_r, 0.0_r};
        
        std::tie(rigidBody.v, rigidBody.omega) = computeVelocities(rigidBody.propulsion,
                                                                   q.rotate(force),
                                                                   q.rotate(torque));

        rigidBody.v     = qInv.rotate(rigidBody.v    );
        rigidBody.omega = qInv.rotate(rigidBody.omega);
        
        const auto _omega = Quaternion::createFromComponents(0.0_r, rigidBody.omega);
        const auto dq_dt = 0.5_r * q * _omega;
        
        rigidBody.r += dt * rigidBody.v;
        q += dt * dq_dt;
        
        rigidBody.q = q.normalized();
    }
    
    currentTime += dt;
    ++currentTimeStep;
}

void Simulation::dump()
{
    const real omega = magneticField.omega(currentTime);
    const real3 dir  = magneticField.rotatingDirection(currentTime);
    
    file << currentTime << " " << omega << " "  << dir.x << " "  << dir.y << " "  << dir.z;

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

} // namespace msode
