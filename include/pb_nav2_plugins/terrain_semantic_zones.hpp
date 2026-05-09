// Copyright 2026 Jieliang Li
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_NAV2_PLUGINS__TERRAIN_SEMANTIC_ZONES_HPP_
#define PB_NAV2_PLUGINS__TERRAIN_SEMANTIC_ZONES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace pb_nav2_plugins
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

enum class SemanticZoneKind
{
  HIGH_COST,
  SLOWDOWN,
  LETHAL
};

struct SemanticZone
{
  std::string name;
  std::string type;
  SemanticZoneKind kind{SemanticZoneKind::HIGH_COST};
  std::vector<Point2D> polygon;
  unsigned char cost{160};
  std::uint8_t state_id{1};
  double speed_limit{-1.0};
  int priority{50};
  bool enabled{true};
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

struct SemanticZoneMap
{
  std::string frame_id{"map"};
  std::vector<SemanticZone> zones;
};

bool loadSemanticZoneMap(
  const std::string & zones_file, SemanticZoneMap * zone_map, std::string * error);

bool containsPoint(const SemanticZone & zone, double x, double y);

bool isValidPolygon(const SemanticZone & zone);

void computeBounds(SemanticZone * zone);

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__TERRAIN_SEMANTIC_ZONES_HPP_
