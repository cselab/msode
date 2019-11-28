#pragma once

#include "environment.h"

#include <log.h>
#include <math.h>
#include <types.h>

#include <tuple>
#include <vector>

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

struct MagnFieldFromActionBase
{
    MagnFieldFromActionBase(real minOmega_, real maxOmega_) :
        minOmega(minOmega_),
        maxOmega(maxOmega_)
    {}

    virtual int numActions() const = 0;

    virtual std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const = 0;
    virtual std::tuple<real3, real3, real3> getFrameReference() const = 0;
    virtual void setAction(const std::vector<double>& action) = 0;
    virtual void advance(real) {}

    virtual real getOmega(real t) const = 0;
    virtual real3 getAxis(real t) const = 0;

protected:
    const real minOmega;
    const real maxOmega;
};


struct MagnFieldFromActionChange : MagnFieldFromActionBase
{
    MagnFieldFromActionChange(real minOmega_, real maxOmega_, real actionDt_) :
        MagnFieldFromActionBase(minOmega_, maxOmega_),
        actionDt(actionDt_)
    {}

    MagnFieldFromActionChange(const MagnFieldFromActionChange&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionChange> *env_) {env = env_;}

    int numActions() const override {return 4;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{minOmega, -1.0, -1.0, -1.0},
                {maxOmega,  1.0,  1.0,  1.0}};
    }

    std::tuple<real3, real3, real3> getFrameReference() const override {return {ex, ey, ez};}
    
    void setAction(const std::vector<double>& action) override
    {
        MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
        dOmega = action[0];
        dAxis.x = action[1];
        dAxis.y = action[2];
        dAxis.z = action[3];
    }

    void advance(real t) override
    {
        // advance
        lastOmega += omegaActionChange(t);
        lastAxis += axisActionChange(t);
        lastActionTime = t;

        // constraints
        lastOmega = std::max(lastOmega, minOmega);
        lastOmega = std::min(lastOmega, maxOmega);
        lastAxis = normalized(lastAxis);
    }

    real getOmega(real t) const override
    {
        return lastOmega + omegaActionChange(t);
    }

    real3 getAxis(real t) const override
    {
        return lastAxis + axisActionChange(t);
    }


private:
    const MSodeEnvironment<MagnFieldFromActionChange> *env {nullptr};
    const real actionDt;
    
    real lastOmega {0._r};
    real3 lastAxis {1._r, 0._r, 0._r};
    real lastActionTime {0._r};
    
    real dOmega {0._r};
    real3 dAxis {0._r, 0._r, 0._r};

    static inline real transitionSmoothKernel(real x) { return x * x * (3.0_r - 2.0_r * x); }
    static inline real transitionLinearKernel(real x) { return x; }

    real omegaActionChange(real t) const
    {
        const real tau = (t - lastActionTime) / actionDt;
        return transitionLinearKernel(tau) * dOmega;
    }

    real3 axisActionChange(real t) const
    {
        const real tau = (t - lastActionTime) / actionDt;
        return transitionSmoothKernel(tau) * dAxis;
    }
};


struct MagnFieldFromActionDirect : MagnFieldFromActionBase
{
    MagnFieldFromActionDirect(real minOmega_, real maxOmega_) :
        MagnFieldFromActionBase(minOmega_, maxOmega_)
    {}

    MagnFieldFromActionDirect(const MagnFieldFromActionDirect&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionDirect> *env_) {env = env_;}

    int numActions() const override {return 4;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{minOmega, -1.0, -1.0, -1.0},
                {maxOmega,  1.0,  1.0,  1.0}};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
        constexpr real tolerance = 1e-6_r;
    
        omega = std::min(maxOmega, std::max(minOmega, static_cast<real>(action[0])));
        axis.x = action[1];
        axis.y = action[2];
        axis.z = action[3];
        if (dot(axis, axis) < tolerance)
            axis.x = 1.0_r;
        axis = normalized(axis);
    }

    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}

private:

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};

    const MSodeEnvironment<MagnFieldFromActionDirect> *env{nullptr};
};

struct MagnFieldFromActionFromTargets : MagnFieldFromActionBase
{
    MagnFieldFromActionFromTargets(real maxOmega_) :
        MagnFieldFromActionBase(0.0, maxOmega_)
    {}

    MagnFieldFromActionFromTargets(const MagnFieldFromActionFromTargets&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromTargets> *env_) {env = env_;}
    
    int numActions() const override {return 1 + env->getBodies().size();}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        std::vector<double> lo{-maxOmega}, hi{+maxOmega};

        for (size_t i = 0; i < env->getBodies().size(); ++i)
        {
            lo.push_back(0.0_r);
            hi.push_back(1.0_r);
        }
        
        return {std::move(lo), std::move(hi)};
    }

    std::tuple<real3, real3, real3> getFrameReference() const {return {ex, ey, ez};}
    
    void setAction(const std::vector<double>& action) override
    {
        MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                     std::string("expect action of size ") + std::to_string(numActions()));

        omega = std::min(+maxOmega, std::max(-maxOmega, static_cast<real>(action[0])));

        axis = real3{0.0_r, 0.0_r, 0.0_r};

        const auto& bodies  = env->getBodies();
        const auto& targets = env->getTargetPositions();

        for (size_t i = 0; i < bodies.size(); ++i)
        {
            auto u = normalized(bodies[i].r - targets[i]);
            axis += action[i+1] * u;
        }
        
        axis = normalized(axis);
    }

    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}

private:

    const MSodeEnvironment<MagnFieldFromActionFromTargets> *env {nullptr};
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


struct MagnFieldFromActionFromLocalFrame : MagnFieldFromActionBase
{
    MagnFieldFromActionFromLocalFrame(real minOmega_, real maxOmega_) :
        MagnFieldFromActionBase(minOmega_, maxOmega_)
    {}

    MagnFieldFromActionFromLocalFrame(const MagnFieldFromActionFromLocalFrame&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromLocalFrame> *env_) {env = env_;}
    
    int numActions() const override {return 1+3;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{minOmega, -1.0, -1.0, -1.0},
                {maxOmega, +1.0, +1.0, +1.0}};
    }

    std::tuple<real3, real3, real3> getFrameReference() const override
    {
        constexpr int NotFound = -1;
        
        const auto& bodies  = env->getBodies();
        const auto& targets = env->getTargetPositions();

        auto selectId = [&bodies, &targets](int start/*, real3 n = real3{0.0_r, 0.0_r, 0.0_r}*/) -> int
        {
            constexpr real minDist = 1e-1_r;

            for (size_t i = start; i < bodies.size(); ++i)
            {
                real3 dr = bodies[i].r - targets[i];
                // dr -= dot(n, dr) * n;
                // MSODE_Ensure(dot(dr, n) < 1e-5_r, "dr is not othogonal to n");
                const real l = length(dr);
                if (l > minDist) return i;
            }
            return NotFound;
        };
        
        const int i1 = selectId(0);

        if (i1 == NotFound)
            return {ex, ey, ez};
        
        const real3 n1 = normalized(bodies[i1].r - targets[i1]);

        //const int i2 = selectId(i1 + 1, n1);
        const int i2 = selectId(i1 + 1);
        real3 n2;
        
        if (i2 == NotFound)
        {
            n2 = normalized(anyOrthogonal(n1));
        }
        else
        {
            n2 = normalized(bodies[i2].r - targets[i2]);
            n2 -= dot(n1, n2) * n1;
            n2 = normalized(n2);
        }

        const real3 n3 = cross(n1, n2);

        MSODE_Ensure(dot(n1, n2) < 1e-5_r,
                     "n1 and n2 are not orthogonal");
        MSODE_Ensure(std::abs(length(n3) - 1.0_r) < 1e-5_r,
                     "Bad normalization");
        
        return {n1, n2, n3};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                     std::string("expect action of size ") + std::to_string(numActions()));

        const real3 a {static_cast<real>(action[1]),
                       static_cast<real>(action[2]),
                       static_cast<real>(action[3])};
        
        omega = std::min(maxOmega, std::max(minOmega, static_cast<real>(action[0])));

        real3 n1, n2, n3;
        std::tie(n1, n2, n3) = getFrameReference();

        axis = a.x * n1 + a.y * n2 + a.z * n3;
        axis = normalized(axis);
    }

    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}


private:
    const MSodeEnvironment<MagnFieldFromActionFromLocalFrame> *env {nullptr};
protected:
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


struct MagnFieldFromActionFromLocalPlane : MagnFieldFromActionFromLocalFrame
{
    MagnFieldFromActionFromLocalPlane(real minOmega_, real maxOmega_) :
        MagnFieldFromActionFromLocalFrame(minOmega_, maxOmega_)
    {}

    MagnFieldFromActionFromLocalPlane(const MagnFieldFromActionFromLocalPlane&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromLocalPlane> *env_) {env = env_;}
    
    int numActions() const override {return 1+2;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{minOmega, -1.0, -1.0},
                {maxOmega, +1.0, +1.0}};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                     std::string("expect action of size ") + std::to_string(numActions()));

        const real ax = static_cast<real>(action[1]);
        const real ay = static_cast<real>(action[2]);
        
        omega = std::min(+maxOmega, std::max(0.0_r, static_cast<real>(action[0])));

        real3 n1, n2, n3;
        std::tie(n1, n2, n3) = getFrameReference();

        axis = ax * n1 + ay * n2;
        axis = normalized(axis);
    }

private:
    const MSodeEnvironment<MagnFieldFromActionFromLocalPlane> *env {nullptr};
};