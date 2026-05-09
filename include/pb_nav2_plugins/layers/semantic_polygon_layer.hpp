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

#ifndef PB_NAV2_PLUGINS__LAYERS__SEMANTIC_POLYGON_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__SEMANTIC_POLYGON_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pb_nav2_plugins/terrain_semantic_zones.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb_nav2_costmap_2d
{

class SemanticPolygonLayer : public nav2_costmap_2d::Layer
{
public:
  SemanticPolygonLayer() = default;
  ~SemanticPolygonLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;

  void reset() override { current_ = true; }

  bool isClearable() override { return false; }

private:
  void applyZoneCost(
    nav2_costmap_2d::Costmap2D & master_grid, unsigned int mx, unsigned int my,
    unsigned char zone_cost) const;

  pb_nav2_plugins::SemanticZoneMap zone_map_;
  std::string zones_file_;
  bool use_maximum_{true};
  bool update_full_map_{true};
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__SEMANTIC_POLYGON_LAYER_HPP_
