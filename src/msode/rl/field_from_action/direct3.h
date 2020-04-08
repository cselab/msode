#pragma once

#include "interface.h"

namespace msode {
namespace rl {

/**
   Same as FieldFromActionDirect but with omega encoded as the vector length
 */
class FieldFromActionDirect3 : public FieldFromAction
{
public:
    FieldFromActionDirect3(real maxOmega);
    FieldFromActionDirect3(const FieldFromActionDirect3&) = default;

    int numActions() const override;

    ActionBounds getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const override;
    
    void setAction(const std::vector<double>& action) override;
    real getOmega(real) const override  {return omega_;}
    real3 getAxis(real) const override  {return axis_;}

private:
    real omega_ {0._r};
    real3 axis_ {1._r, 0._r, 0._r};
};

} // namespace rl
} // namespace msode
