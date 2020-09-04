// Copyright 2020 ETH Zurich. All Rights Reserved.
/** optimal_path_landscape

    A tool to visualize the function to optimize in the analytical control setup.
    It corresponds to the travel time with respect to 3 given angles.
 */
#include <msode/analytic_control/helpers.h>
#include <msode/analytic_control/optimal_path.h>

#include <iostream>

using namespace msode;

static std::vector<RigidBody> readBodies(const Config& config)
{
    if (!config.is_array())
        msode_die("Expected an array of bodies in config");

    std::vector<RigidBody> bodies;

    for (const auto& c : config)
        bodies.push_back(msode::factory::readRigidBodyFromConfig(c));

    return bodies;
}

static std::vector<real3> randomIC(int n)
{
    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};
    const long seed = 42424242;
    return analytic_control::generateRandomPositionsBox(n, boxLo, boxHi, seed);
}

static std::vector<real3> alignedIC(int n)
{
    const real3 direction = normalized(real3{0.4_r, 0.2_r, 0.8_r});
    std::vector<real3> positions;
    for (int i = 0; i < n; ++i)
        positions.push_back(10.0_r * (i+1) * direction);
    return positions;
}

static std::vector<real3> createIC(const std::string& mode, int n)
{
    if (mode == "random")
        return randomIC(n);
    else if (mode == "aligned")
        return alignedIC(n);
    else
        msode_die("No match for ic mode \"%s\"", mode.c_str());
    return {};
}

static inline Quaternion quaternionFromAngles(real theta, real phi, real psi)
{
    const real3 normal {std::cos(theta) * std::sin(phi),
                        std::sin(theta) * std::sin(phi),
                        std::cos(phi)};

    const auto q = Quaternion::createFromRotation(psi, normal);
    return q.normalized();
}

int main(int argc, char **argv)
{
    if (argc != 3                    ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr,
                "usage : %s <config.json> <initial conditions>\n"
                "where:\n"
                "\t<config.json> contains the list of bodies\n"
                "\t<initial conditions> is either \"random\" or \"aligned\"\n"
                "\n",
                argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude");
    const auto bodies = readBodies(config.at("bodies"));

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const analytic_control::MatrixReal U = V.inverse();

    const std::string icMode = argv[2];
    const auto positions = createIC(icMode, bodies.size());
    const auto A = analytic_control::computeA(U, positions);

    auto F = [&](real theta, real phi, real psi)
    {
        return analytic_control::computeTravelTime(A, quaternionFromAngles(theta, phi, psi));
    };

    const int res = 128;

    const int ntheta = 2*res;
    const int nphi = res;
    const int npsi = res;

    const real dtheta = 2 * M_PI / ntheta;
    const real dphi   = M_PI     / nphi;
    const real dpsi   = 2 * M_PI / npsi;

    // values on the grid
    std::vector<real> Fvals(ntheta * nphi * npsi);

    for (int ips = 0, i = 0; ips < npsi; ++ips)
    {
        const real psi = ips * dpsi;
        for (int iph = 0; iph < nphi; ++iph)
        {
            const real phi = iph * dphi;
            for (int ith = 0; ith < ntheta; ++ith, ++i)
            {
                const real theta = ith * dtheta;
                Fvals[i] = F(theta, phi, psi);
            }
        }
    }

    // dump to vtk

    const char *outFileName = "out.vtk";
    std::ofstream f {outFileName};

    MSODE_Ensure(f.is_open(), "Error opening file '%s'", outFileName);

    f << "# vtk DataFile Version 2.0\n"
      << "Velocity field dumped from msode\n"
      << "ASCII\n";

    f << "DATASET STRUCTURED_POINTS\n"
      << "DIMENSIONS " << ntheta << ' ' << nphi << ' ' << npsi << '\n'
      << "ORIGIN " << 0 << ' ' << 0 << ' ' << 0 << '\n'
      << "SPACING " << dtheta << ' ' << dphi << ' ' << dpsi << '\n';

    f << "POINT_DATA " << Fvals.size() << "\n"
      << "SCALARS travelTime float 1\n"
      << "LOOKUP_TABLE default\n";

    for (auto v : Fvals)
        f << v << '\n';

    return 0;
}
