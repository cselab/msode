#pragma once

#include "types.h"

namespace msode
{

/** \brief Base class that describes the background velocity field
 */
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

class VelocityFieldConstant : public BaseVelocityField
{
public:
    VelocityFieldConstant(real3 vel);

    real3 getVelocity(real3 r, real t) const override;
    
private:
    const real3 vel_; ///< velocity everywhere in space and time
};

class VelocityFieldNone : public VelocityFieldConstant
{
public:
    VelocityFieldNone();
};

} // namespace msode
