#pragma once

#include <msode/core/types.h>

#include <memory>
#include <string>

namespace msode
{

/// Base class that describes the background velocity field
class BaseVelocityField
{
public:
    virtual ~BaseVelocityField();

    virtual std::unique_ptr<BaseVelocityField> clone() const = 0;

    /** \returns the velocity at position \p r and time \p t
        \param [in] r The position at which to evaluate the velocity
        \param [in] t The current time
    */
    virtual real3 getVelocity(real3 r, real t) const = 0;

    /** \returns the vorticity at position \p r and time \p t
        \param [in] r The position at which to evaluate the vorticity
        \param [in] t The current time
    */
    virtual real3 getVorticity(real3 r, real t) const = 0;

    /** dump the velocity and vorticity fields on a uniform grid to a vtk file called \p fileName.
        \param [in] fileName The destination file name. 
        \param [in] dimensions Number of points per dimension
        \param [in] start The lowest corner of the domain
        \param [in] size The size of the domain to dump
        \param [in] t The time at which to dump the field

        This method will fail if it cannot write to the file.
     */
    void dumpToVtkUniformGrid(const std::string& fileName, int3 dimensions, real3 start, real3 size, real t) const;
};

} // namespace msode
