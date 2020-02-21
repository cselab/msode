#pragma once

#include "types.h"

namespace msode
{

/// Base class that describes the background velocity field
class BaseVelocityField
{
public:
    virtual ~BaseVelocityField();

    /** \returns the velocity at position \p r and time \p t
        \param [in] r The position at which to evaluate the velocity
        \param [in] t The current time
    */
    virtual real3 getVelocity(real3 r, real t) const = 0;
};

/// Velocity field constant in time and space
class VelocityFieldConstant : public BaseVelocityField
{
public:
    /// construct a VelocityFieldConstant with constant velocity \p vel
    VelocityFieldConstant(real3 vel);

    real3 getVelocity(real3 r, real t) const override;
    
private:
    const real3 vel_; ///< velocity everywhere in space and time
};

/// Velocity field with zero velocity
class VelocityFieldNone : public VelocityFieldConstant
{
public:
    /// construct a VelocityFieldNone
    VelocityFieldNone();
};


/// Steady Taylor-green vortex flow, see initial conditions in https://en.wikipedia.org/wiki/Taylor%E2%80%93Green_vortex
class VelocityFieldTaylorGreenVortex : public BaseVelocityField
{
public:
    /** Construct a VelocityFieldTaylorGreenVortex.
        \param [in] magnitude
        \param [in] invPeriod
        
        This method will fail if the parameters are not satisfying the incompressibility condition
        (see https://en.wikipedia.org/wiki/Taylor%E2%80%93Green_vortex)
    */
    VelocityFieldTaylorGreenVortex(real3 magnitude, real3 invPeriod);

    real3 getVelocity(real3 r, real t) const override;
    
private:
    real3 magnitude_; ///< magnitudes along the 3 directions
    real3 invPeriod_; ///< inverse wave lengths along each dimension
};

} // namespace msode
