#pragma once

#include <msode/types.h>

#include <memory>
#include <random>

using msode::real;
using msode::real3;
using namespace msode::literals;

class EnvSpace
{
public:
    EnvSpace() = default;

    virtual std::unique_ptr<EnvSpace> clone() const = 0;
    
    virtual real3 getLowestPosition()  const = 0;
    virtual real3 getHighestPosition() const = 0;
    virtual real computeMaxDistanceToTarget() const = 0;

    virtual real3 generatePosition(std::mt19937& gen) const = 0;

public:
    const real3 target {0.0_r, 0.0_r, 0.0_r};
};

class EnvSpaceBox : public EnvSpace
{
public:
    EnvSpaceBox(real L_);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    real3 generatePosition(std::mt19937& gen) const override;

private:
    struct Box
    {
        real3 lo, hi;
        std::array<real3, 8> getCorners() const
        {
            return {real3 {lo.x, lo.y, lo.z},
                    real3 {lo.x, lo.y, hi.z},
                    real3 {lo.x, hi.y, lo.z},
                    real3 {lo.x, hi.y, hi.z},
                    real3 {hi.x, lo.y, lo.z},
                    real3 {hi.x, lo.y, hi.z},
                    real3 {hi.x, hi.y, lo.z},
                    real3 {hi.x, hi.y, hi.z}};
        }
    };

    const Box domain;
};

class EnvSpaceBall : public EnvSpace
{
public:
    EnvSpaceBall(real R_);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    real3 generatePosition(std::mt19937& gen) const override;

private:
    const real R;
};
