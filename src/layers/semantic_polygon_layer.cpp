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

#include "pb_nav2_plugins/layers/semantic_polygon_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"

namespace pb_nav2_costmap_2d
{

void SemanticPolygonLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  enabled_ = node->declare_parameter(name_ + ".enabled", true);
  zones_file_ = node->declare_parameter(name_ + ".zones_file", std::string(""));
  use_maximum_ = node->declare_parameter(name_ + ".use_maximum", true);
  update_full_map_ = node->declare_parameter(name_ + ".update_full_map", true);

  current_ = true;

  if (zones_file_.empty()) {
    RCLCPP_WARN(
      node->get_logger(), "SemanticPolygonLayer '%s' has no zones_file configured.",
      name_.c_str());
    return;
  }

  std::string error;
  if (!pb_nav2_plugins::loadSemanticZoneMap(zones_file_, &zone_map_, &error)) {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to load semantic terrain zones from '%s': %s",
      zones_file_.c_str(), error.c_str());
    zone_map_.zones.clear();
    return;
  }

  RCLCPP_INFO(
    node->get_logger(), "Loaded %zu semantic terrain zones from '%s' for layer '%s' in frame '%s'.",
    zone_map_.zones.size(), zones_file_.c_str(), name_.c_str(), zone_map_.frame_id.c_str());
}

void SemanticPolygonLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_ || zone_map_.zones.empty()) {
    return;
  }

  const auto * master = layered_costmap_->getCostmap();
  if (master == nullptr) {
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  const bool transform_required =
    !zone_map_.frame_id.empty() && zone_map_.frame_id != costmap_frame;

  if (update_full_map_ || transform_required) {
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
    *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
    return;
  }

  for (const auto & zone : zone_map_.zones) {
    if (!pb_nav2_plugins::isValidPolygon(zone)) {
      continue;
    }
    *min_x = std::min(*min_x, zone.min_x);
    *min_y = std::min(*min_y, zone.min_y);
    *max_x = std::max(*max_x, zone.max_x);
    *max_y = std::max(*max_y, zone.max_y);
  }
}

void SemanticPolygonLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || zone_map_.zones.empty()) {
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  const bool transform_required =
    !zone_map_.frame_id.empty() && zone_map_.frame_id != costmap_frame;

  geometry_msgs::msg::TransformStamped costmap_to_zone_transform;
  double transform_yaw = 0.0;
  double transform_cos = 1.0;
  double transform_sin = 0.0;

  if (transform_required) {
    auto node = node_.lock();
    if (!node || tf_ == nullptr) {
      return;
    }
    try {
      costmap_to_zone_transform =
        tf_->lookupTransform(zone_map_.frame_id, costmap_frame, tf2::TimePointZero);
      transform_yaw = tf2::getYaw(costmap_to_zone_transform.transform.rotation);
      transform_cos = std::cos(transform_yaw);
      transform_sin = std::sin(transform_yaw);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "SemanticPolygonLayer '%s' cannot transform from costmap frame '%s' to zones frame '%s': "
        "%s",
        name_.c_str(), costmap_frame.c_str(), zone_map_.frame_id.c_str(), ex.what());
      return;
    }
  }

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

  for (int my = min_j; my < max_j; ++my) {
    for (int mx = min_i; mx < max_i; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);

      double zone_x = wx;
      double zone_y = wy;
      if (transform_required) {
        const auto & translation = costmap_to_zone_transform.transform.translation;
        zone_x = transform_cos * wx - transform_sin * wy + translation.x;
        zone_y = transform_sin * wx + transform_cos * wy + translation.y;
      }

      unsigned char best_cost = nav2_costmap_2d::FREE_SPACE;
      bool has_zone = false;
      for (const auto & zone : zone_map_.zones) {
        if (!pb_nav2_plugins::containsPoint(zone, zone_x, zone_y)) {
          continue;
        }
        best_cost = has_zone ? std::max(best_cost, zone.cost) : zone.cost;
        has_zone = true;
      }

      if (!has_zone) {
        continue;
      }

      applyZoneCost(
        master_grid, static_cast<unsigned int>(mx), static_cast<unsigned int>(my), best_cost);
    }
  }
}

void SemanticPolygonLayer::applyZoneCost(
  nav2_costmap_2d::Costmap2D & master_grid, const unsigned int mx, const unsigned int my,
  const unsigned char zone_cost) const
{
  const unsigned char old_cost = master_grid.getCost(mx, my);
  if (!use_maximum_ || old_cost == nav2_costmap_2d::NO_INFORMATION || zone_cost > old_cost) {
    master_grid.setCost(mx, my, zone_cost);
  }
}

}  // namespace pb_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::SemanticPolygonLayer, nav2_costmap_2d::Layer)
