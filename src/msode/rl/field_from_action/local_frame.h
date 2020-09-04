// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class FieldFromActionFromLocalFrame : public FieldFromAction
{
public:
    FieldFromActionFromLocalFrame(real minOmega, real maxOmega);
    FieldFromActionFromLocalFrame(const FieldFromActionFromLocalFrame&) = default;

    int numActions() const override;
    ActionBounds getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const override;

    void setAction(const std::vector<double>& action) override;

    real getOmega(real) const override  {return omega_;}
    real3 getAxis(real) const override  {return axis_;}

protected:
    real omega_ {0._r};
    real3 axis_ {1._r, 0._r, 0._r};
};

} // namespace rl
} // namespace msode
