#pragma once

#include <msode/core/log.h>
#include <msode/core/math.h>
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


class FieldFromActionDirect : public FieldFromAction
{
public:
    FieldFromActionDirect(real minOmega, real maxOmega);
    FieldFromActionDirect(const FieldFromActionDirect&) = default;

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

class FieldFromActionFromTargets : public FieldFromAction
{
public:
    FieldFromActionFromTargets(real maxOmega);
    FieldFromActionFromTargets(const FieldFromActionFromTargets&) = default;

    int numActions() const override;

    ActionBounds getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const;
    
    void setAction(const std::vector<double>& action) override;

    real getOmega(real) const override  {return omega_;}
    real3 getAxis(real) const override  {return axis_;}

private:
    real omega_ {0._r};
    real3 axis_ {1._r, 0._r, 0._r};
};


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
