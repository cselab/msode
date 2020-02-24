#pragma once

#include <msode/core/types.h>

#include <tuple>
#include <vector>

namespace msode {
namespace rl {

class MSodeEnvironment;

using ActionBounds = std::tuple<std::vector<double>, std::vector<double>>;

class FieldFromAction
{
public:
    FieldFromAction(real minOmega, real maxOmega);
    
    void attach(const MSodeEnvironment *env);
    virtual int numActions() const = 0;

    virtual ActionBounds getActionBounds() const = 0;
    virtual std::tuple<real3, real3, real3> getFrameReference() const = 0;
    virtual void setAction(const std::vector<double>& action) = 0;
    virtual void advance(real) {}

    virtual real getOmega(real t) const = 0;
    virtual real3 getAxis(real t) const = 0;

protected:
    const real minOmega_;
    const real maxOmega_;

    const MSodeEnvironment *env_ {nullptr};
};

} // namespace rl
} // namespace msode

