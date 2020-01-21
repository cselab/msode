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

void Simulation::activateDump(const std::string& fname, long dumpEvery_)
{
    MSODE_Expect(dumpEvery_ > 0, "expect positive dumpEvery");
    this->dumpEvery = dumpEvery_;

    if (file.is_open())
        file.close();

    file.open(fname);
    MSODE_Ensure(file.is_open(), "could not open file for writing");
}

void Simulation::runForwardEuler(long nsteps, real dt)
{
    MSODE_Expect(nsteps > 0, "expect positive number of steps");
    MSODE_Expect(dt > 0._r, "expect positive time step");
    
    for (long step = 0; step < nsteps; ++step)
        advanceForwardEuler(dt);
}

void Simulation::runRK4(long nsteps, real dt)
{
    MSODE_Expect(nsteps > 0, "expect positive number of steps");
    MSODE_Expect(dt > 0._r, "expect positive time step");
    
    for (long step = 0; step < nsteps; ++step)
        advanceRK4(dt);
}

void Simulation::advanceForwardEuler(real dt)
{
    if (file.is_open() && currentTimeStep % dumpEvery == 0)
        dump();

    stepForwardEuler(dt);
    
    ++currentTimeStep;
}

void Simulation::advanceRK4(real dt)
{
    if (file.is_open() && currentTimeStep % dumpEvery == 0)
        dump();

    stepRK4(dt);

    ++currentTimeStep;
}

static inline std::tuple<real3, real3, Quaternion>
computeDerivatives(const RigidBody& b, real3 B)
{
    const Quaternion q = b.q;
    const Quaternion qInv = q.conjugate();
        
    const real3 m      = qInv.rotate(b.magnMoment);
    const real3 torque = cross(m, B);
    constexpr real3 force {0.0_r, 0.0_r, 0.0_r};

    real3 v, omega;
    std::tie(v, omega) = computeVelocities(b.propulsion,
                                           q.rotate(force),
                                           q.rotate(torque));

    v     = qInv.rotate(v    );
    omega = qInv.rotate(omega);
        
    const auto _omega = Quaternion::createPureVector(omega);
    const auto dq_dt = 0.5_r * q * _omega;

    return {v, omega, dq_dt};
}

void Simulation::stepForwardEuler(real dt)
{
    const real3 B = magneticField(currentTime);
    
    for (auto& rigidBody : rigidBodies)
    {
        Quaternion dq_dt;
        std::tie(rigidBody.v, rigidBody.omega, dq_dt) = computeDerivatives(rigidBody, B);
        
        rigidBody.r += dt * rigidBody.v;
        rigidBody.q += dt * dq_dt;
        
        rigidBody.q = rigidBody.q.normalized();
    }

    magneticField.advance(currentTime, dt);
    currentTime += dt;
}

void Simulation::stepRK4(real dt)
{
    const real dt_half = 0.5_r * dt;

    const real3 B0 = magneticField(currentTime);
    magneticField.advance(currentTime, dt_half);

    const real3 Bh = magneticField(currentTime+dt_half); // B half
    magneticField.advance(currentTime, dt_half);

    const real3 B1 = magneticField(currentTime+dt);
    
    for (auto& rigidBody : rigidBodies)
    {
        Quaternion dq_dt1, dq_dt2, dq_dt3, dq_dt4;
        real3 v1, v2, v3, v4, omegaWork;

        // compute k1 = f(y0, t)
        RigidBody bWork = rigidBody;
        std::tie(v1, omegaWork, dq_dt1) = computeDerivatives(bWork, B0);

        bWork.r += 0.5_r * dt * v1;
        bWork.q += 0.5_r * dt * dq_dt1;
        bWork.q = bWork.q.normalized();

        // compute k2 = f(y0 + dt/2 * k1, t + dt/2)
        std::tie(v2, omegaWork, dq_dt2) = computeDerivatives(bWork, Bh);

        bWork = rigidBody;
        bWork.r += 0.5_r * dt * v2;
        bWork.q += 0.5_r * dt * dq_dt2;
        bWork.q = bWork.q.normalized();

        // compute k3 = f(y0 + dt/2 * k2, t + dt/2)
        std::tie(v3, omegaWork, dq_dt3) = computeDerivatives(bWork, Bh);

        bWork = rigidBody;
        bWork.r += dt * v3;
        bWork.q += dt * dq_dt3;
        bWork.q = bWork.q.normalized();

        // compute k4 = f(y0 + dt * k3, t + dt)
        std::tie(v4, omegaWork, dq_dt4) = computeDerivatives(bWork, B1);


        // compute y1 = y0 + (k1/6 + k2/3 + k3/3 + k4/6) * dt
        constexpr real one_third = 1.0_r / 3.0_r;
        constexpr real one_sixth = 1.0_r / 6.0_r;

        rigidBody.r += dt * (one_sixth * (v1     + v4    ) + one_third * (v2     + v3    ));
        rigidBody.q += dt * (one_sixth * (dq_dt1 + dq_dt4) + one_third * (dq_dt2 + dq_dt3));
        rigidBody.q = rigidBody.q.normalized();
        
        rigidBody.v     = v4;
        rigidBody.omega = omegaWork;
    }
    
    currentTime += dt;
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
