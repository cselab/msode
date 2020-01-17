#include "chiSquare.h"

#include <cmath>

namespace chiSquare {

// https://www.codeproject.com/Articles/432194/How-to-Calculate-the-Chi-Squared-P-Value
static inline double incompleteGamma(double s, double z)
{
    if (z < 0.0)
	return 0.0;

    const double sc = std::pow(z, s) * std::exp(-z) / s;

    double sum = 1.0;
    double nom = 1.0;
    double denom = 1.0;

    constexpr int niters = 200;
    
    for (int i = 0; i < niters; ++i)
    {
	nom *= z;
	s += 1.0;
	denom *= s;
	sum += (nom / denom);
    }
 
    return sum * sc;
}

double chiSquare(int Dof, double Cv)
{
    if (Cv < 0 || Dof < 1)
        return 0.0;

    const double k = ((double) Dof) * 0.5;
    const double x = Cv * 0.5;

    if (Dof == 2)
	return std::exp(-x);
 
    double PValue = incompleteGamma(k, x);

    if (std::isnan(PValue) || std::isinf(PValue))
    {
        return 1e-14;
    } 

    PValue /= std::tgamma(k);
	
    return (1.0 - PValue);
}

} // namespace chiSquare
