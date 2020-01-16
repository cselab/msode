#include <msode/analytic_control/optimal_path.h>
#include <msode/analytic_control/apply_strategy.h>

#include <iostream>

int main(int argc, char **argv)
{
    using namespace msode;
    
    if (argc < 4                     ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <nsamples> <outfile> <swimmer0.cfg> <swimmer1.cfg>... \n\n", argv[0]);
        return 1;
    }

    const int nsamples = atoi(argv[1]);
    const char *outFilename = argv[2];
    
    const real magneticFieldMagnitude = 1.0_r;

    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};
    
    std::vector<RigidBody> bodies;
    for (int i = 3; i < argc; ++i)
    {
        const auto body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const analytic_control::MatrixReal U = V.inverse();

    real tSum = 0.0_r;

    FILE *fout = fopen(outFilename, "w");
    for (int sample = 0; sample < nsamples; ++sample)
    {
        const long seed = 242 * sample + 13;
        auto initialPositions = analytic_control::generateRandomPositionsBox(bodies.size(), boxLo, boxHi, seed);
        const real t = analytic_control::computeRequiredTime(magneticFieldMagnitude, bodies, initialPositions, U);
        tSum += t;

        for (auto r : initialPositions)
            fprintf(fout, "%g ", length(r));
        fprintf(fout, "%g\n", t);
    }
    fclose(fout);
    
    const real tMean = tSum / nsamples;

    printf("Average time %g\n", tMean);
    
    return 0;
}
