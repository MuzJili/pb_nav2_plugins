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

#include "pb_nav2_plugins/terrain_semantic_zones.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace pb_nav2_plugins
{
namespace
{

template <typename T>
T readValue(const YAML::Node & node, const char * key, const T & default_value)
{
  if (!node[key]) {
    return default_value;
  }
  return node[key].as<T>();
}

SemanticZoneKind kindFromType(const std::string & type)
{
  if (type == "lethal") {
    return SemanticZoneKind::LETHAL;
  }
  if (type == "slowdown") {
    return SemanticZoneKind::SLOWDOWN;
  }
  return SemanticZoneKind::HIGH_COST;
}

unsigned char defaultCostForKind(const SemanticZoneKind kind)
{
  switch (kind) {
    case SemanticZoneKind::LETHAL:
      return 254;
    case SemanticZoneKind::SLOWDOWN:
      return 100;
    case SemanticZoneKind::HIGH_COST:
    default:
      return 160;
  }
}

std::uint8_t defaultStateForKind(const SemanticZoneKind kind)
{
  switch (kind) {
    case SemanticZoneKind::LETHAL:
      return 255;
    case SemanticZoneKind::SLOWDOWN:
      return 2;
    case SemanticZoneKind::HIGH_COST:
    default:
      return 1;
  }
}

int defaultPriorityForKind(const SemanticZoneKind kind)
{
  switch (kind) {
    case SemanticZoneKind::LETHAL:
      return 100;
    case SemanticZoneKind::SLOWDOWN:
      return 25;
    case SemanticZoneKind::HIGH_COST:
    default:
      return 50;
  }
}

unsigned char clampCost(const int cost)
{
  return static_cast<unsigned char>(std::clamp(cost, 0, 254));
}

std::uint8_t clampStateId(const int state_id)
{
  return static_cast<std::uint8_t>(std::clamp(state_id, 0, 255));
}

bool parsePoint(const YAML::Node & point_node, Point2D * point, std::string * error)
{
  if (!point_node.IsSequence() || point_node.size() < 2) {
    if (error != nullptr) {
      *error = "point must be a sequence with at least two values";
    }
    return false;
  }

  point->x = point_node[0].as<double>();
  point->y = point_node[1].as<double>();
  return true;
}

bool parsePolygon(const YAML::Node & polygon_node, SemanticZone * zone, std::string * error)
{
  if (!polygon_node || !polygon_node.IsSequence()) {
    return true;
  }

  for (std::size_t point_i = 0; point_i < polygon_node.size(); ++point_i) {
    Point2D point;
    std::string point_error;
    if (!parsePoint(polygon_node[point_i], &point, &point_error)) {
      std::ostringstream message;
      message << "zone '" << zone->name << "' has an invalid polygon point at index " << point_i
              << ": " << point_error;
      if (error != nullptr) {
        *error = message.str();
      }
      return false;
    }
    zone->polygon.push_back(point);
  }
  return true;
}

bool makeLineStripPolygon(
  const Point2D & start, const Point2D & end, const double line_width,
  std::vector<Point2D> * polygon)
{
  if (polygon == nullptr || line_width <= 0.0) {
    return false;
  }

  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double half_width = 0.5 * line_width;
  const double normal_x = -dy / length * half_width;
  const double normal_y = dx / length * half_width;

  polygon->clear();
  polygon->push_back({start.x + normal_x, start.y + normal_y});
  polygon->push_back({end.x + normal_x, end.y + normal_y});
  polygon->push_back({end.x - normal_x, end.y - normal_y});
  polygon->push_back({start.x - normal_x, start.y - normal_y});
  return true;
}

bool parseLine(
  const YAML::Node & line_node, const double line_width, SemanticZone * zone, std::string * error)
{
  if (!line_node) {
    return true;
  }

  if (!line_node.IsSequence() || line_node.size() != 2) {
    if (error != nullptr) {
      *error = "zone '" + zone->name + "' line must contain exactly two points";
    }
    return false;
  }

  Point2D start;
  Point2D end;
  std::string point_error;
  if (!parsePoint(line_node[0], &start, &point_error) ||
    !parsePoint(line_node[1], &end, &point_error))
  {
    if (error != nullptr) {
      *error = "zone '" + zone->name + "' has an invalid line point: " + point_error;
    }
    return false;
  }

  if (!makeLineStripPolygon(start, end, line_width, &zone->polygon)) {
    if (error != nullptr) {
      *error = "zone '" + zone->name + "' has zero-length line or non-positive line_width";
    }
    return false;
  }
  return true;
}

bool makeSegmentZone(
  const YAML::Node & segment_node, const double line_width, const SemanticZone & base_zone,
  const std::size_t segment_index, SemanticZone * segment_zone, std::string * error)
{
  if (!segment_node.IsSequence() || segment_node.size() != 2) {
    std::ostringstream message;
    message << "zone '" << base_zone.name << "' segment " << segment_index
            << " must contain exactly two points";
    if (error != nullptr) {
      *error = message.str();
    }
    return false;
  }

  Point2D start;
  Point2D end;
  std::string point_error;
  if (!parsePoint(segment_node[0], &start, &point_error) ||
    !parsePoint(segment_node[1], &end, &point_error))
  {
    std::ostringstream message;
    message << "zone '" << base_zone.name << "' has an invalid segment " << segment_index << ": "
            << point_error;
    if (error != nullptr) {
      *error = message.str();
    }
    return false;
  }

  *segment_zone = base_zone;
  segment_zone->name = base_zone.name + "_" + std::to_string(segment_index);
  segment_zone->polygon.clear();
  if (!makeLineStripPolygon(start, end, line_width, &segment_zone->polygon)) {
    std::ostringstream message;
    message << "zone '" << base_zone.name << "' segment " << segment_index
            << " is zero-length or has non-positive line_width";
    if (error != nullptr) {
      *error = message.str();
    }
    return false;
  }

  computeBounds(segment_zone);
  return true;
}

}  // namespace

bool loadSemanticZoneMap(
  const std::string & zones_file, SemanticZoneMap * zone_map, std::string * error)
{
  if (zone_map == nullptr) {
    if (error != nullptr) {
      *error = "zone_map output pointer is null";
    }
    return false;
  }

  zone_map->zones.clear();

  try {
    const YAML::Node root = YAML::LoadFile(zones_file);
    zone_map->frame_id = readValue<std::string>(root, "frame_id", "map");

    const YAML::Node zones = root["zones"];
    if (!zones || !zones.IsSequence()) {
      return true;
    }

    for (std::size_t i = 0; i < zones.size(); ++i) {
      const YAML::Node zone_node = zones[i];
      SemanticZone zone;
      zone.name = readValue<std::string>(
        zone_node, "name", std::string("terrain_zone_") + std::to_string(i));
      zone.type = readValue<std::string>(zone_node, "type", "high_cost");
      zone.kind = kindFromType(zone.type);
      zone.enabled = readValue<bool>(zone_node, "enabled", true);
      zone.cost = clampCost(readValue<int>(zone_node, "cost", defaultCostForKind(zone.kind)));
      zone.state_id =
        clampStateId(readValue<int>(zone_node, "state_id", defaultStateForKind(zone.kind)));
      zone.speed_limit = readValue<double>(zone_node, "speed_limit", -1.0);
      zone.priority = readValue<int>(zone_node, "priority", defaultPriorityForKind(zone.kind));

      const double line_width =
        readValue<double>(zone_node, "line_width", readValue<double>(zone_node, "width", 0.2));

      const YAML::Node segments = zone_node["segments"];
      if (segments) {
        if (!segments.IsSequence()) {
          if (error != nullptr) {
            *error = "zone '" + zone.name + "' segments must be a sequence";
          }
          return false;
        }
        for (std::size_t segment_i = 0; segment_i < segments.size(); ++segment_i) {
          SemanticZone segment_zone;
          if (!makeSegmentZone(
                segments[segment_i], line_width, zone, segment_i, &segment_zone, error))
          {
            return false;
          }
          zone_map->zones.push_back(segment_zone);
        }
        continue;
      }

      if (!parsePolygon(zone_node["polygon"], &zone, error)) {
        return false;
      }
      if (zone.polygon.empty() && !parseLine(zone_node["line"], line_width, &zone, error)) {
        return false;
      }

      computeBounds(&zone);
      zone_map->zones.push_back(zone);
    }
  } catch (const YAML::Exception & ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }

  return true;
}

bool containsPoint(const SemanticZone & zone, const double x, const double y)
{
  if (!isValidPolygon(zone)) {
    return false;
  }

  if (x < zone.min_x || x > zone.max_x || y < zone.min_y || y > zone.max_y) {
    return false;
  }

  bool inside = false;
  std::size_t j = zone.polygon.size() - 1;
  for (std::size_t i = 0; i < zone.polygon.size(); j = i++) {
    const auto & pi = zone.polygon[i];
    const auto & pj = zone.polygon[j];
    const bool crosses_y = (pi.y > y) != (pj.y > y);
    if (!crosses_y) {
      continue;
    }
    const double x_intersection = (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x;
    if (x < x_intersection) {
      inside = !inside;
    }
  }
  return inside;
}

bool isValidPolygon(const SemanticZone & zone)
{
  return zone.enabled && zone.polygon.size() >= 3;
}

void computeBounds(SemanticZone * zone)
{
  if (zone == nullptr || zone->polygon.empty()) {
    return;
  }

  zone->min_x = std::numeric_limits<double>::max();
  zone->min_y = std::numeric_limits<double>::max();
  zone->max_x = std::numeric_limits<double>::lowest();
  zone->max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : zone->polygon) {
    zone->min_x = std::min(zone->min_x, point.x);
    zone->min_y = std::min(zone->min_y, point.y);
    zone->max_x = std::max(zone->max_x, point.x);
    zone->max_y = std::max(zone->max_y, point.y);
  }
}

}  // namespace pb_nav2_plugins
