#pragma once

#include "environment.h"

#include <log.h>
#include <math.h>
#include <types.h>

#include <vector>

struct MagnFieldFromActionBase
{
    MagnFieldFromActionBase(real maxOmega) :
        maxOmega(maxOmega)
    {}

    virtual int numActions() const = 0;
    
    virtual void setAction(const std::vector<double>& action) = 0;
    virtual void advance(real t) {}

    virtual real getOmega(real t) const = 0;
    virtual real3 getAxis(real t) const = 0;

protected:
    const real maxOmega;
};


struct MagnFieldFromActionChange : MagnFieldFromActionBase
{
    MagnFieldFromActionChange(real maxOmega, real actionDt) :
        MagnFieldFromActionBase(maxOmega),
        actionDt(actionDt)
    {}

    MagnFieldFromActionChange(const MagnFieldFromActionChange&) = default;

    int numActions() const override {return 4;}
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(action.size() == 4, "expect action of size 4");
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
        lastOmega = std::max(lastOmega, 0._r);
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
    MagnFieldFromActionDirect(real maxOmega) :
        MagnFieldFromActionBase(maxOmega)
    {}

    MagnFieldFromActionDirect(const MagnFieldFromActionDirect&) = default;

    int numActions() const override {return 4;}
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(action.size() == 4, "expect action of size 4");
        constexpr real tolerance = 1e-6_r;
    
        omega = std::min(maxOmega, std::max(0._r, static_cast<real>(action[0])));
        axis.x = action[1];
        axis.y = action[2];
        axis.z = action[3];
        if (dot(axis, axis) < tolerance)
            axis.x = 1.0_r;
        axis = normalized(axis);
    }

    real getOmega(real t) const override  {return omega;}
    real3 getAxis(real t) const override  {return axis;}

private:

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};

struct MagnFieldFromActionFromTargets : MagnFieldFromActionBase
{
    MagnFieldFromActionFromTargets(real maxOmega,
                                   const MSodeEnvironment<MagnFieldFromActionFromTargets> *env) :
        MagnFieldFromActionBase(maxOmega),
        env(env)
    {}

    MagnFieldFromActionFromTargets(const MagnFieldFromActionFromTargets&) = default;
    
    int numActions() const override {return 1 + env->getBodies().size();}
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(action.size() == numActions(),
               std::string("expect action of size ") + std::to_string(numActions()));

        omega = std::min(maxOmega, std::max(0._r, static_cast<real>(action[0])));

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

    real getOmega(real t) const override  {return omega;}
    real3 getAxis(real t) const override  {return axis;}

private:

    const MSodeEnvironment<MagnFieldFromActionFromTargets> *env;
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};
