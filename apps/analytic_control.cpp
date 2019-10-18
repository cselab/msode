#include "simulation.h"
#include "factory.h"

#include <Eigen/LU>

#include <iostream>

using MatrixReal = Matrix<real, Dynamic, Dynamic, RowMajor>;
using ArrayReal  = Array <real, Dynamic, 1>;
constexpr real magneticFieldMagnitude {1.0_r};

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <swimmer.cfg> \n\n", argv[0]);
        return 1;
    }

    MatrixReal V;
    
    return 0;
}
