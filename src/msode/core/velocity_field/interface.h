// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/types.h>

#include <functional>
#include <memory>
#include <ostream>
#include <string>

namespace msode
{

/// Symmetric matrix
struct DeformationRateTensor
{
    real xx, xy, xz, yy, yz, zz;
};

using Filter = std::function<bool(real3)>;

static inline real3 multiply(DeformationRateTensor T, real3 v)
{
    return {T.xx * v.x + T.xy * v.y + T.xz * v.z,
            T.xy * v.x + T.yy * v.y + T.yz * v.z,
            T.xz * v.x + T.yz * v.y + T.zz * v.z};
}

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

    /** \returns the deformation rate tensor at position \p r and time \p t
        \param [in] r The position at which to evaluate the tensor
        \param [in] t The current time
    */
    virtual DeformationRateTensor getDeformationRateTensor(real3 r, real t) const = 0;

    /** dump the velocity and vorticity fields on a uniform grid to a vtk file called \p fileName.
        \param [in] fileName The destination file name.
        \param [in] dimensions Number of points per dimension
        \param [in] start The lowest corner of the domain
        \param [in] size The size of the domain to dump
        \param [in] t The time at which to dump the field

        This method will fail if it cannot write to the file.
     */
    void dumpToVtkUniformGrid(const std::string& fileName, int3 dimensions, real3 start, real3 size,
                              real t, Filter filter = [](real3) {return true;}) const;

    /** dump the velocity and vorticity fields on a uniform grid in vtk format to a given stream.
        \param [in] stream The destination stream to dump to.
        \param [in] dimensions Number of points per dimension
        \param [in] start The lowest corner of the domain
        \param [in] size The size of the domain to dump
        \param [in] t The time at which to dump the field
     */
    void dumpToVtkUniformGrid(std::ostream& stream, int3 dimensions, real3 start, real3 size,
                              real t, Filter filter = [](real3) {return true;}) const;
};

} // namespace msode
