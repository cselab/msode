#pragma once

#include "environment.h"

#include <log.h>
#include <math.h>
#include <types.h>

#include <tuple>
#include <vector>

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

struct MagnFieldFromActionBase
{
    MagnFieldFromActionBase(real maxOmega) :
        maxOmega(maxOmega)
    {}

    virtual int numActions() const = 0;

    virtual std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const = 0;
    virtual std::tuple<real3, real3, real3> getFrameReference() const = 0;
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

    void attach(const MSodeEnvironment<MagnFieldFromActionChange> *env) {this->env = env;}

    int numActions() const override {return 4;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{0.0, -1.0, -1.0, -1.0},
                {maxOmega, 1.0, 1.0, 1.0}};
    }

    std::tuple<real3, real3, real3> getFrameReference() const {return {ex, ey, ez};}
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
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
    const MSodeEnvironment<MagnFieldFromActionChange> *env {nullptr};
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

    void attach(const MSodeEnvironment<MagnFieldFromActionDirect> *env) {this->env = env;}

    int numActions() const override {return 4;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{0.0, -1.0, -1.0, -1.0},
                {maxOmega, 1.0, 1.0, 1.0}};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
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

    const MSodeEnvironment<MagnFieldFromActionDirect> *env{nullptr};
};

struct MagnFieldFromActionFromTargets : MagnFieldFromActionBase
{
    MagnFieldFromActionFromTargets(real maxOmega) :
        MagnFieldFromActionBase(maxOmega)
    {}

    MagnFieldFromActionFromTargets(const MagnFieldFromActionFromTargets&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromTargets> *env) {this->env = env;}
    
    int numActions() const override {return 1 + env->getBodies().size();}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        std::vector<double> lo{-maxOmega}, hi{+maxOmega};

        for (size_t i = 0; i < env->getBodies().size(); ++i)
        {
            lo.push_back(0.0_r);
            hi.push_back(1.0_r);
        }
        
        return {std::move(lo), std::move(hi)};
    }

    std::tuple<real3, real3, real3> getFrameReference() const {return {ex, ey, ez};}
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(static_cast<int>(action.size()) == numActions(),
               std::string("expect action of size ") + std::to_string(numActions()));

        omega = std::min(+maxOmega, std::max(-maxOmega, static_cast<real>(action[0])));

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

    const MSodeEnvironment<MagnFieldFromActionFromTargets> *env {nullptr};
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


struct MagnFieldFromActionFromLocalFrame : MagnFieldFromActionBase
{
    MagnFieldFromActionFromLocalFrame(real maxOmega) :
        MagnFieldFromActionBase(maxOmega)
    {}

    MagnFieldFromActionFromLocalFrame(const MagnFieldFromActionFromLocalFrame&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromLocalFrame> *env) {this->env = env;}
    
    int numActions() const override {return 1+3;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{0.0, -1.0, -1.0, -1.0},
                {maxOmega, +1.0, +1.0, +1.0}};
    }

    std::tuple<real3, real3, real3> getFrameReference() const
    {
        constexpr int NotFound = -1;
        
        const auto& bodies  = env->getBodies();
        const auto& targets = env->getTargetPositions();

        auto selectId = [&bodies, &targets](int start) -> int
        {
            constexpr real minDist = 1e-1_r;

            for (size_t i = start; i < bodies.size(); ++i)
            {
                const real l = length(bodies[i].r - targets[i]);
                if (l > minDist) return i;
            }
            return NotFound;
        };
        
        const int i1 = selectId(0);
        const int i2 = selectId(i1 + 1);

        if (i1 == NotFound)
            return {ex, ey, ez};
        
        const real3 n1 = normalized(bodies[i1].r - targets[i1]);

        real3 n2;
        
        if (i2 == NotFound)
        {
            n2 = normalized(anyOrthogonal(n1));
        }
        else
        {
            real3 n2 = normalized(bodies[i2].r - targets[i2]);
            n2 -= dot(n1, n2) * n1;
            n2 = normalized(n2);
        }
        
        return {n1, n2, cross(n1, n2)};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(static_cast<int>(action.size()) == numActions(),
               std::string("expect action of size ") + std::to_string(numActions()));

        const real3 a {static_cast<real>(action[1]),
                       static_cast<real>(action[2]),
                       static_cast<real>(action[3])};
        
        omega = std::min(+maxOmega, std::max(0.0_r, static_cast<real>(action[0])));

        real3 n1, n2, n3;
        std::tie(n1, n2, n3) = getFrameReference();

        axis = a.x * n1 + a.y * n2 + a.z * n3;
        axis = normalized(axis);
    }

    real getOmega(real t) const override  {return omega;}
    real3 getAxis(real t) const override  {return axis;}

private:

    const MSodeEnvironment<MagnFieldFromActionFromLocalFrame> *env {nullptr};
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};


struct MagnFieldFromActionFromLocalPlane : MagnFieldFromActionBase
{
    MagnFieldFromActionFromLocalPlane(real maxOmega) :
        MagnFieldFromActionBase(maxOmega)
    {}

    MagnFieldFromActionFromLocalPlane(const MagnFieldFromActionFromLocalPlane&) = default;

    void attach(const MSodeEnvironment<MagnFieldFromActionFromLocalPlane> *env) {this->env = env;}
    
    int numActions() const override {return 1+2;}

    std::tuple<std::vector<double>, std::vector<double>> getActionBounds() const override
    {
        return {{0.0, -1.0, -1.0},
                {maxOmega, +1.0, +1.0}};
    }

    std::tuple<real3, real3, real3> getFrameReference() const
    {
        constexpr int NotFound = -1;
        
        const auto& bodies  = env->getBodies();
        const auto& targets = env->getTargetPositions();

        auto selectId = [&bodies, &targets](int start) -> int
        {
            constexpr real minDist = 1e-1_r;

            for (size_t i = start; i < bodies.size(); ++i)
            {
                const real l = length(bodies[i].r - targets[i]);
                if (l > minDist) return i;
            }
            return NotFound;
        };
        
        const int i1 = selectId(0);
        const int i2 = selectId(i1 + 1);

        if (i1 == NotFound)
            return {ex, ey, ez};
        
        const real3 n1 = normalized(bodies[i1].r - targets[i1]);

        real3 n2;
        
        if (i2 == NotFound)
        {
            n2 = normalized(anyOrthogonal(n1));
        }
        else
        {
            real3 n2 = normalized(bodies[i2].r - targets[i2]);
            n2 -= dot(n1, n2) * n1;
            n2 = normalized(n2);
        }
        
        return {n1, n2, cross(n1, n2)};
    }
    
    void setAction(const std::vector<double>& action) override
    {
        Expect(static_cast<int>(action.size()) == numActions(),
               std::string("expect action of size ") + std::to_string(numActions()));

        const real ax = static_cast<real>(action[1]);
        const real ay = static_cast<real>(action[2]);
        
        omega = std::min(+maxOmega, std::max(0.0_r, static_cast<real>(action[0])));

        real3 n1, n2, n3;
        std::tie(n1, n2, n3) = getFrameReference();

        axis = ax * n1 + ay * n2;
        axis = normalized(axis);
    }

    real getOmega(real t) const override  {return omega;}
    real3 getAxis(real t) const override  {return axis;}

private:

    const MSodeEnvironment<MagnFieldFromActionFromLocalPlane> *env {nullptr};
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};
};
