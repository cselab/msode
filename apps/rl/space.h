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

    std::vector<real3> generateNewPositionsIfFlag(std::mt19937& gen, int n, bool generateNew);
    virtual std::vector<real3> generateNewPositions(std::mt19937& gen, int n) = 0;

public:
    const real3 target {0.0_r, 0.0_r, 0.0_r};
private:
    bool savedPositionsInitialized {false};
    std::vector<real3> savedPositions;
};

class EnvSpaceBox : public EnvSpace
{
public:
    EnvSpaceBox(real L_);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
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

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real R;
};

class EnvSpaceBallCuriculumMC : public EnvSpaceBall
{
public:
    EnvSpaceBallCuriculumMC(real R_, real targetR_, real sigmaRandomWalk_);

    std::unique_ptr<EnvSpace> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real targetR;
    const real sigmaRandomWalk;
    
    bool initialized {false};
    std::vector<real3> previousPositions;
};
