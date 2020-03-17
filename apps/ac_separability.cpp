/** ac_separability

    Collect optimal travel time in one dimension for N swimmers, along with properties of the velocity matrix.
    See ../launch_scripts/separability/ for application.
 */

#include <msode/analytic_control/helpers.h>
#include <msode/analytic_control/optimal_path.h>
#include <msode/core/simulation.h>

#include <iostream>
#include <random>
#include <omp.h>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

static RigidBody createRigidBody(real Vmax, real omegaC)
{
    const real Bxx = 0.1_r;
    const real m   = Vmax   / (magneticFieldMagnitude * Bxx);
    const real Cxx = omegaC / (magneticFieldMagnitude * m);

    const real Cyy = 0.1_r * Cxx;
    const real Czz = Cyy;

    PropulsionMatrix P;
    P.A[0] = 0.0_r;
    P.A[1] = 0.0_r;
    P.A[2] = 0.0_r;

    P.B[0] = Bxx;
    P.B[1] = 0.0_r;
    P.B[2] = 0.0_r;

    P.C[0] = Cxx;
    P.C[1] = Cyy;
    P.C[2] = Czz;
    
    const real3 magnMoment {0.0_r, m, 0.0_r};
    
    const auto q = Quaternion::createFromComponents(1.0_r, 0.0_r, 0.0_r, 0.0_r);
    const real3 r {0.0_r, 0.0_r, 0.0_r};

    const RigidBody b {q, r, magnMoment, P};
    
    MSODE_Ensure(std::abs(b.stepOutFrequency(magneticFieldMagnitude, 0) - omegaC) < 1e-6_r,
                 "wrong step out frequency");
    
    return b;
}

static real computeSeparability(const analytic_control::MatrixReal& V)
{
    MSODE_Expect(V.cols() == V.rows(), "Expect a square matrix");
    
    real S = 0.0_r;
    const int N = V.cols();

    for (int i = 0; i < N; ++i)
    {
        real Si = 0.0_r;
        for (int j = 0; j < N; ++j)
        {
            if (j == i) continue;
            Si += V(i,j) / V(i,i);
        }

        Si /= (N - 1);
        S += Si;
    }
    S /= N;
    return S;
}

static real computeLambdaRatio(const analytic_control::MatrixReal& V)
{
    MSODE_Expect(V.cols() == V.rows(), "Expect a square matrix");
    
    const auto eigenValues = analytic_control::computeEigenValues(V);

    real minL = std::numeric_limits<real>::max();
    real maxL = std::numeric_limits<real>::lowest();

    for (auto l : eigenValues)
    {
        minL = std::min(minL, std::abs(l));
        maxL = std::max(maxL, std::abs(l));
    }
    
    const real S = maxL / minL;
    return S;
}

static inline real norm2(const analytic_control::MatrixReal& A)
{
    real a = 0.0_r;
    for (int i = 0; i < A.rows(); ++i)
        for (int j = 0; j < A.cols(); ++j)
            a += A(i,j) * A(i,j);
    return std::sqrt(a);
}

static real computeConditionNumber(const analytic_control::MatrixReal& V)
{
    MSODE_Expect(V.cols() == V.rows(), "Expect a square matrix");
    return norm2(V) * norm2(V.inverse());
}

static real computeMeanTimeFromRandomIC(const analytic_control::MatrixReal& V, const std::vector<msode::RigidBody>& bodies)
{
    const analytic_control::MatrixReal U = V.inverse();

    const real L {50.0_r};
    const real3 boxLo {- 0.5_r * L, 0.0_r, 0.0_r};
    const real3 boxHi {+ 0.5_r * L, 0.0_r, 0.0_r};
    const real3 direction {1.0_r, 0.0_r, 0.0_r};
    
    real tSum = 0.0_r;
    const int nsamples = 50000;

    for (int sample = 0; sample < nsamples; ++sample)
    {
        const long seed = 242 * sample + 13;
        const auto initialPositions = analytic_control::generateRandomPositionsBox(bodies.size(), boxLo, boxHi, seed);
        const auto A = analytic_control::computeA(U, initialPositions);
        const real t = analytic_control::computeTime(A, direction);
        tSum += t;
    }
    
    const real tMean = tSum / nsamples;
    return tMean;
}

static real computeMeanTimeFromTestIC(const analytic_control::MatrixReal& V, const std::vector<msode::RigidBody>& bodies)
{
    const analytic_control::MatrixReal U = V.inverse();
    const int n = bodies.size();
    MSODE_Ensure(n < 8*sizeof(int), "must use less than 32 bodies");

    const real L {50.0_r};
    const real3 direction {1.0_r, 0.0_r, 0.0_r};
    
    real tSum = 0.0_r;
    
    const int nsamples = 1<<n;
    std::vector<real3> initialPositions(n);

    for (int sample = 0; sample < nsamples; ++sample)
    {
        for (int i = 0; i < n; ++i)
        {
            const int isRight = ( sample & (1<<i) ) >> i;
            initialPositions[i] = isRight ? real3 {L, 0.0_r, 0.0_r} : real3 {-L, 0.0_r, 0.0_r};
        }
        const auto A = analytic_control::computeA(U, initialPositions);
        const real t = analytic_control::computeTime(A, direction);
        tSum += t;
    }
    
    const real tMean = tSum / nsamples;
    return tMean;
}


int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <number of swimmers> <omega max> <number of samples>\n\n", argv[0]);
        return 1;
    }
    
    const real Vmax = 1.0_r;
    const int nbodies = atoi(argv[1]);
    const real maxOmega = std::stod(argv[2]);
    const real minOmega = 1e-6_r;
    const int nsSamples = atoi(argv[3]);

    const long seed0 = 2323232323;
    const long seed1 = 17;

    std::vector<real> seps(nsSamples), lratios(nsSamples), kappas(nsSamples), times(nsSamples);

    #pragma omp parallel
    {
        std::mt19937 gen(seed0 * omp_get_thread_num() + seed1);
        std::uniform_real_distribution<real> dist(minOmega, maxOmega);
        
        #pragma omp for
        for (int sSample = 0; sSample < nsSamples; ++sSample)
        {
            std::vector<msode::RigidBody> bodies;
    
            for (int i = 0; i < nbodies; ++i)
            {
                const real omegaC = dist(gen);
                const auto b = createRigidBody(Vmax, omegaC);
                bodies.push_back(b);
            }

            const auto V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);

            const real S = computeSeparability(V);
            const real l = computeLambdaRatio(V);
            const real k = computeConditionNumber(V);
            const real T = computeMeanTimeFromTestIC(V, bodies);

            times  [sSample] = T;
            seps   [sSample] = S;
            lratios[sSample] = l;
            kappas [sSample] = k;
        }
    }
    
    for (size_t i = 0; i < times.size(); ++i)
        printf("%g %g %g %g\n", seps[i], lratios[i], kappas[i], times[i]);
    
    return 0;
}
