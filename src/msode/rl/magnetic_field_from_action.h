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

class MagnFieldFromActionBase
{
public:
    MagnFieldFromActionBase(real minOmega, real maxOmega);
    
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


class MagnFieldFromActionChange : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionChange(real minOmega, real maxOmega, real actionDt);
    MagnFieldFromActionChange(const MagnFieldFromActionChange&) = default;

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


class MagnFieldFromActionDirect : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionDirect(real minOmega, real maxOmega);
    MagnFieldFromActionDirect(const MagnFieldFromActionDirect&) = default;

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

class MagnFieldFromActionFromTargets : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionFromTargets(real maxOmega);
    MagnFieldFromActionFromTargets(const MagnFieldFromActionFromTargets&) = default;

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


class MagnFieldFromActionFromLocalFrame : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionFromLocalFrame(real minOmega, real maxOmega);
    MagnFieldFromActionFromLocalFrame(const MagnFieldFromActionFromLocalFrame&) = default;

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


class MagnFieldFromActionFromLocalPlane : public MagnFieldFromActionFromLocalFrame
{
public:
    MagnFieldFromActionFromLocalPlane(real minOmega_, real maxOmega_);
    MagnFieldFromActionFromLocalPlane(const MagnFieldFromActionFromLocalPlane&) = default;

    int numActions() const override;

    ActionBounds getActionBounds() const override;
    void setAction(const std::vector<double>& action) override;
};

} // namespace rl
} // namespace msode
