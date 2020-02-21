#pragma once

#include <msode/core/types.h>

#include <memory>
#include <random>

namespace msode {
namespace rl {

class EnvSpace
{
public:
    EnvSpace() = default;

    virtual std::unique_ptr<EnvSpace> clone() const = 0;
    
    virtual real3 getLowestPosition()  const = 0;
    virtual real3 getHighestPosition() const = 0;
    virtual real computeMaxDistanceToTarget() const = 0;

    const std::vector<real3>& generateNewPositionsIfFlag(std::mt19937& gen, int n, bool generateNew);
    virtual std::vector<real3> generateNewPositions(std::mt19937& gen, int n) = 0;

public:
    const real3 target {0.0_r, 0.0_r, 0.0_r};

private:
    bool savedPositionsInitialized_ {false};
    std::vector<real3> savedPositions_;
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

    const Box domain_;
};

class EnvSpaceBall : public EnvSpace
{
public:
    EnvSpaceBall(real radius);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real radius_;
};

class EnvSpaceBallCuriculumStateRW : public EnvSpaceBall
{
public:
    EnvSpaceBallCuriculumStateRW(real radius, real targetRadius, real sigmaRandomWalk);

    std::unique_ptr<EnvSpace> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;
    
    bool initialized_ {false};
    std::vector<real3> previousPositions_;
};

class MSodeEnvironment;

class EnvSpaceBallCuriculumActionRW : public EnvSpaceBall
{
public:
    EnvSpaceBallCuriculumActionRW(std::unique_ptr<MSodeEnvironment>&& environment,
                                  real radius, real targetRadius, real sigmaRandomWalk);

    std::unique_ptr<EnvSpace> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    std::vector<double> _generateAction(std::mt19937& gen) const;
    
protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;
    
    bool initialized_ {false};
    std::vector<real3> previousPositions_;

    // shared because too lazy to write clone with unique_ptr
    std::shared_ptr<MSodeEnvironment> environment_;
};

} // namespace rl
} // namespace msode
