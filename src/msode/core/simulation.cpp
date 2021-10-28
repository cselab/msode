// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "simulation.h"
#include "math.h"
#include "velocity_field/none.h"

#include <tuple>

namespace msode
{

Simulation::Simulation(std::vector<RigidBody> initialRBs,
                       MagneticField initialMF) :
    Simulation(std::move(initialRBs), std::move(initialMF), std::make_unique<VelocityFieldNone>())
{}

Simulation::Simulation(std::vector<RigidBody> initialRBs, MagneticField initialMF,
                       std::unique_ptr<BaseVelocityField> velocityField) :
    rigidBodies_(std::move(initialRBs)),
    magneticField_(std::move(initialMF)),
    velocityField_(std::move(velocityField))
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

void Simulation::reset(std::vector<RigidBody> initialRBs, MagneticField initialMF)
{
    currentTimeStep_ = 0;
    currentTime_     = 0._r;
    rigidBodies_ = std::move(initialRBs);
    magneticField_ = std::move(initialMF);
}

void Simulation::activateDump(const std::string& fname, long dumpEvery)
{
    MSODE_Expect(dumpEvery > 0, "expect positive dumpEvery");
    dumpEvery_ = dumpEvery;

    if (file_.is_open())
        file_.close();

    file_.open(fname);
    MSODE_Ensure(file_.is_open(), "could not open file for writing");
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
    if (file_.is_open() && currentTimeStep_ % dumpEvery_ == 0)
        dump();

    _stepForwardEuler(dt);

    ++currentTimeStep_;
}

void Simulation::advanceRK4(real dt)
{
    if (file_.is_open() && currentTimeStep_ % dumpEvery_ == 0)
        dump();

    _stepRK4(dt);

    ++currentTimeStep_;
}

static inline std::tuple<real3, real3>
computeVelocities(const RigidBody& b, real3 B,
                  const BaseVelocityField *velocityField,
                  real time)
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

    v     +=         velocityField->getVelocity (b.r, time);
    omega += 0.5_r * velocityField->getVorticity(b.r, time);

    const auto T = velocityField->getDeformationRateTensor(b.r, time);
    const real3 p = qInv.rotate({1.0_r, 0.0_r, 0.0_r});
    const real L = (b.aspectRatio*b.aspectRatio - 1.0_r) / (b.aspectRatio*b.aspectRatio + 1.0_r);
    omega += L * cross(p, multiply(T, p));

    return {v, omega};
}

static inline Quaternion quaternionDerivative(Quaternion q, real3 omega)
{
    return 0.5_r * q * Quaternion::createPureVector(omega);
}


static inline void addThermalNoise(RigidBody& b,
                                   real dt,
                                   std::mt19937& gen,
                                   std::normal_distribution<real>& normal)
{
    if (b.transDiffusion > 0)
    {
        const real3 eta {normal(gen), normal(gen), normal(gen)};
        b.v += std::sqrt(2 * b.transDiffusion / dt) * eta;
    }
    if (b.rotDiffusion > 0)
    {
        const real3 eta {normal(gen), normal(gen), normal(gen)};
        b.omega += std::sqrt(2 * b.rotDiffusion / dt) * eta;
    }
}


void Simulation::_stepForwardEuler(real dt)
{
    const real3 B = magneticField_(currentTime_);

    for (auto& rigidBody : rigidBodies_)
    {
        std::tie(rigidBody.v, rigidBody.omega) = computeVelocities(rigidBody, B,
                                                                   velocityField_.get(),
                                                                   currentTime_);
        addThermalNoise(rigidBody, dt, gen_, normal_);

        const Quaternion dq_dt = quaternionDerivative(rigidBody.q, rigidBody.omega);

        rigidBody.r += dt * rigidBody.v;
        rigidBody.q += dt * dq_dt;

        rigidBody.q = rigidBody.q.normalized();
    }

    magneticField_.advance(currentTime_, dt);
    currentTime_ += dt;
}

void Simulation::_stepRK4(real dt)
{
    const real dt_half = 0.5_r * dt;

    const real3 B0 = magneticField_(currentTime_);
    magneticField_.advance(currentTime_, dt_half);

    const real3 Bh = magneticField_(currentTime_ + dt_half); // B half
    magneticField_.advance(currentTime_ + dt_half, dt_half);

    const real3 B1 = magneticField_(currentTime_ + dt);

    for (auto& rigidBody : rigidBodies_)
    {
        MSODE_Ensure(rigidBody.transDiffusion == 0 && rigidBody.rotDiffusion == 0,
                     "SDE not implemented for RK4. Expect zero diffusion.");

        real3 v1, v2, v3, v4;

        // compute k1 = f(y0, t)
        RigidBody bWork = rigidBody;
        std::tie(v1, bWork.omega) = computeVelocities(bWork, B0,
                                                      velocityField_.get(),
                                                      currentTime_);
        const auto dq_dt1 = quaternionDerivative(bWork.q, bWork.omega);

        bWork.r = rigidBody.r + dt_half * v1;
        bWork.q = rigidBody.q + dt_half * dq_dt1;
        bWork.q = bWork.q.normalized();
        bWork.v = v1;

        // compute k2 = f(y0 + dt/2 * k1, t + dt/2)
        std::tie(v2, bWork.omega) = computeVelocities(bWork, Bh,
                                                      velocityField_.get(),
                                                      currentTime_+dt_half);
        const auto dq_dt2 = quaternionDerivative(bWork.q, bWork.omega);

        bWork.r = rigidBody.r + dt_half * v2;
        bWork.q = rigidBody.q + dt_half * dq_dt2;
        bWork.q = bWork.q.normalized();
        bWork.v = v2;

        // compute k3 = f(y0 + dt/2 * k2, t + dt/2)
        std::tie(v3, bWork.omega) = computeVelocities(bWork, Bh,
                                                      velocityField_.get(),
                                                      currentTime_+dt_half);
        const auto dq_dt3 = quaternionDerivative(bWork.q, bWork.omega);

        bWork.r = rigidBody.r + dt * v3;
        bWork.q = rigidBody.q + dt * dq_dt3;
        bWork.q = bWork.q.normalized();
        bWork.v = v3;

        // compute k4 = f(y0 + dt * k3, t + dt)
        std::tie(v4, rigidBody.omega) = computeVelocities(bWork, B1,
                                                          velocityField_.get(),
                                                          currentTime_+dt);
        const auto dq_dt4 = quaternionDerivative(rigidBody.q, rigidBody.omega);

        // compute y1 = y0 + (k1/6 + k2/3 + k3/3 + k4/6) * dt
        constexpr real one_third = 1.0_r / 3.0_r;
        constexpr real one_sixth = 1.0_r / 6.0_r;

        rigidBody.r += dt * (one_sixth * (v1     + v4    ) + one_third * (v2     + v3    ));
        rigidBody.q += dt * (one_sixth * (dq_dt1 + dq_dt4) + one_third * (dq_dt2 + dq_dt3));
        rigidBody.q = rigidBody.q.normalized();
        rigidBody.v = v4;
    }

    currentTime_ += dt;
}

void Simulation::dump()
{
    const real omega = magneticField_.omega(currentTime_);
    const real3 dir  = magneticField_.rotatingDirection(currentTime_);

    file_ << currentTime_ << " " << omega << " "  << dir.x << " "  << dir.y << " "  << dir.z;

    for (const auto& rigidBody : rigidBodies_)
        file_ << " " << rigidBody;

    file_ << "\n";
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
