#pragma once
#include "uav_path_planner/core.hpp"
#include <yaml-cpp/yaml.h>
#include <cctype>
#include <set>

namespace uav_path_planner {
struct ScenarioObstacle {
  std::string name;
  size_t leg;
  Vec3 center,size;
};
inline std::vector<ScenarioObstacle> loadUnknownObstacles(const std::string &file) {
  const auto list=YAML::LoadFile(file)["obstacles"];
  if (!list.IsSequence() || list.size()==0 || list.size()>32)
    throw std::runtime_error("Scenario must contain 1 to 32 obstacles");
  std::vector<ScenarioObstacle> result; std::set<std::string> names;
  auto vector=[](const YAML::Node &v) {
    if (!v.IsSequence() || v.size()!=3) throw std::runtime_error("Scenario vectors must have three values");
    Vec3 p(v[0].as<double>(),v[1].as<double>(),v[2].as<double>());
    if (!p.allFinite()) throw std::runtime_error("Non-finite scenario vector");
    return p;
  };
  for (const auto &item:list) {
    const auto name=item["name"].as<std::string>(); const int leg=item["leg"].as<int>();
    const Vec3 center=vector(item["center"]),size=vector(item["size"]);
    if (name.empty() || !std::all_of(name.begin(),name.end(),[](unsigned char c){ return std::isalnum(c) || c=='_'; })
        || !names.insert(name).second || leg<1 || leg>100 || (size.array()<=0).any())
      throw std::runtime_error("Invalid obstacle name, leg or size");
    result.push_back({name,static_cast<size_t>(leg),center,size});
  }
  return result;
}
}  // namespace uav_path_planner
