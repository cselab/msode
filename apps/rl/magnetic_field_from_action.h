#pragma once

#include <msode/log.h>
#include <msode/math.h>
#include <msode/types.h>

#include <tuple>
#include <vector>

class MSodeEnvironment;

using msode::real;
using msode::real3;
using namespace msode::literals;

class MagnFieldFromActionBase
{
public:
    MagnFieldFromActionBase(real minOmega_, real maxOmega_);
    
    void attach(const MSodeEnvironment *env_);
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

    const MSodeEnvironment *env {nullptr};
};


class MagnFieldFromActionChange : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionChange(real minOmega_, real maxOmega_, real actionDt_);
    MagnFieldFromActionChange(const MagnFieldFromActionChange&) = default;

    int numActions() const override;

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const override;
    
    void setAction(const std::vector<double>& action) override;

    void advance(real t) override;

    real getOmega(real t) const override;
    real3 getAxis(real t) const override;
    
private:
    const real actionDt;
    
    real lastOmega {0._r};
    real3 lastAxis {1._r, 0._r, 0._r};
    real lastActionTime {0._r};
    
    real dOmega {0._r};
    real3 dAxis {0._r, 0._r, 0._r};

    real omegaActionChange(real t) const;
    real3 axisActionChange(real t) const;
};


class MagnFieldFromActionDirect : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionDirect(real minOmega_, real maxOmega_);
    MagnFieldFromActionDirect(const MagnFieldFromActionDirect&) = default;

    int numActions() const override;

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override;
    void setAction(const std::vector<double>& action) override;
    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}

private:

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};

class MagnFieldFromActionFromTargets : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionFromTargets(real maxOmega_);
    MagnFieldFromActionFromTargets(const MagnFieldFromActionFromTargets&) = default;

    int numActions() const override;

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const;
    
    void setAction(const std::vector<double>& action) override;

    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}

private:

    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


class MagnFieldFromActionFromLocalFrame : public MagnFieldFromActionBase
{
public:
    MagnFieldFromActionFromLocalFrame(real minOmega_, real maxOmega_);
    MagnFieldFromActionFromLocalFrame(const MagnFieldFromActionFromLocalFrame&) = default;

    int numActions() const override;
    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override;
    std::tuple<real3, real3, real3> getFrameReference() const override;
    
    void setAction(const std::vector<double>& action) override;

    real getOmega(real) const override  {return omega;}
    real3 getAxis(real) const override  {return axis;}

protected:
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


class MagnFieldFromActionFromLocalPlane : public MagnFieldFromActionFromLocalFrame
{
public:
    MagnFieldFromActionFromLocalPlane(real minOmega_, real maxOmega_);
    MagnFieldFromActionFromLocalPlane(const MagnFieldFromActionFromLocalPlane&) = default;

    int numActions() const override;

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override;
    void setAction(const std::vector<double>& action) override;
};
