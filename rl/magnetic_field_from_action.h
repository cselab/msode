#pragma once

#include <types.h>
#include <math.h>

#include <vector>

struct MagnFieldFromActionChange
{
    MagnFieldFromActionChange(real maxOmega, real actionDt) :
        maxOmega(maxOmega),
        actionDt(actionDt)
    {}

    void setAction(const std::vector<double>& action);

    void advance(real t);
    real getOmega(real t) const;
    real3 getAxis(real t) const;

private:
    const real maxOmega;
    const real actionDt;
        
    real lastOmega {0._r};
    real3 lastAxis {1._r, 0._r, 0._r};
    real lastActionTime {0._r};

    real dOmega {0._r};
    real3 dAxis {0._r, 0._r, 0._r};

    real omegaActionChange(real t) const;
    real3 axisActionChange(real t) const;
};

struct MagnFieldFromActionDirect
{
    MagnFieldFromActionDirect(real maxOmega, real actionDt) :
        maxOmega(maxOmega),
        actionDt(actionDt)
    {}

    void setAction(const std::vector<double>& action);

    void advance(real t);
    real getOmega(real t) const;
    real3 getAxis(real t) const;

private:
    const real maxOmega;
    const real actionDt;

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};
