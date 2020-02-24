#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class FieldFromActionChange : public FieldFromAction
{
public:
    FieldFromActionChange(real minOmega, real maxOmega, real actionDt);
    FieldFromActionChange(const FieldFromActionChange&) = default;

    int numActions() const override;

    ActionBounds getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const override;
    
    void setAction(const std::vector<double>& action) override;

    void advance(real t) override;

    real getOmega(real t) const override;
    real3 getAxis(real t) const override;
    
private:
    const real actionDt_;
    
    real lastOmega_ {0._r};
    real3 lastAxis_ {1._r, 0._r, 0._r};
    real lastActionTime_ {0._r};
    
    real dOmega_ {0._r};
    real3 dAxis_ {0._r, 0._r, 0._r};

    real _omegaActionChange(real t) const;
    real3 _axisActionChange(real t) const;
};

} // namespace rl
} // namespace msode
