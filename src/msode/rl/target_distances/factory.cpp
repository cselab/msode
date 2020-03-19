#include "factory.h"

#include "none.h"
#include "euclidean.h"
#include "square.h"
#include "travel_time.h"
#include "travel_time_non_optimal.h"

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<TargetDistance> createTargetDistance(const Config& config)
{
    std::unique_ptr<TargetDistance> td;

    const auto type = config.at("__type").get<std::string>();

    if (type == "None")
    {
        td = std::make_unique<TargetDistanceNone>();
    }
    else if (type == "Euclidean")
    {
        td = std::make_unique<TargetDistanceEuclidean>();
    }
    else if (type == "Square")
    {
        td = std::make_unique<TargetDistanceSquare>();
    }
    else if (type == "TravelTime")
    {
        td = std::make_unique<TargetDistanceTravelTime>(config.at("fieldMagnitude"));
    }
    else if (type == "TravelTimeNonOptimal")
    {
        td = std::make_unique<TargetDistanceTravelTimeNonOptimal>(config.at("fieldMagnitude"));
    }
    else
    {
        msode_die("Could not generate a TargetDistance  object from type '%s'",
                  type.c_str());
    }
    return td;
}

} // namespace factory
} // namespace rl
} // namespace msode
