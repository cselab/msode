#include "config.h"

namespace msode
{

void to_json(json& j, const PropulsionMatrix& p)
{
    j = json{{"A", p.A}, {"B", p.B}, {"C", p.C}};
}

void from_json(const json& j, PropulsionMatrix& p)
{
    j.at("A").get_to(p.A);
    j.at("B").get_to(p.B);
    j.at("C").get_to(p.C);
}


void to_json(json& j, const Quaternion& q)
{
    j = json::array({q.w, q.x, q.y, q.z});
}

void from_json(const json& j, Quaternion& q)
{
    j.at(0).get_to(q.w);
    j.at(1).get_to(q.x);
    j.at(2).get_to(q.y);
    j.at(3).get_to(q.z);
}


void to_json(json& j, const real3& v)
{
    j = json::array({v.x, v.y, v.z});
}

void from_json(const json& j, real3& v)
{
    j.at(0).get_to(v.x);
    j.at(1).get_to(v.y);
    j.at(2).get_to(v.z);
}


} // namespace msode
