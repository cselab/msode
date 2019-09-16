#pragma once

#include <log.h>
#include <math.h>
#include <types.h>

#include <vector>

struct MagnFieldFromActionChange
{
    MagnFieldFromActionChange(real maxOmega, real actionDt) :
        maxOmega(maxOmega),
        actionDt(actionDt)
    {}

    void setAction(const std::vector<double>& action)
    {
        Expect(action.size() == 4, "expect action of size 4");
        dOmega = action[0];
        dAxis.x = action[1];
        dAxis.y = action[2];
        dAxis.z = action[3];
    }

    void advance(real t)
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

    real getOmega(real t) const
    {
        return lastOmega + omegaActionChange(t);
    }

    real3 getAxis(real t) const
    {
        return lastAxis + axisActionChange(t);
    }


private:
    const real maxOmega;
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


struct MagnFieldFromActionDirect
{
    MagnFieldFromActionDirect(real maxOmega, real actionDt) :
        maxOmega(maxOmega),
        actionDt(actionDt)
    {}

    void setAction(const std::vector<double>& action)
    {
        Expect(action.size() == 4, "expect action of size 4");
        constexpr real tolerance = 1e-6_r;
    
        omega = action[0];
        axis.x = action[1];
        axis.y = action[2];
        axis.z = action[3];
        if (dot(axis, axis) < tolerance)
            axis.x = 1.0_r;
        axis = normalized(axis);
    }

    void advance(real t) {}
    real getOmega(real t) const  {return omega;}
    real3 getAxis(real t) const  {return axis;}

private:
    const real maxOmega;
    const real actionDt;

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};
