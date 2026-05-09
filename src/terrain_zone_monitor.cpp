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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pb_nav2_plugins/terrain_semantic_zones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_nav2_plugins
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

std::uint8_t clampStateId(const int state_id)
{
  return static_cast<std::uint8_t>(std::clamp(state_id, 0, 255));
}

void setMarkerColor(visualization_msgs::msg::Marker * marker, const SemanticZoneKind kind)
{
  marker->color.a = 0.85f;
  switch (kind) {
    case SemanticZoneKind::LETHAL:
      marker->color.r = 1.0f;
      marker->color.g = 0.05f;
      marker->color.b = 0.05f;
      break;
    case SemanticZoneKind::SLOWDOWN:
      marker->color.r = 0.1f;
      marker->color.g = 0.55f;
      marker->color.b = 1.0f;
      break;
    case SemanticZoneKind::HIGH_COST:
    default:
      marker->color.r = 1.0f;
      marker->color.g = 0.65f;
      marker->color.b = 0.05f;
      break;
  }
}

Point2D centroidOf(const SemanticZone & zone)
{
  Point2D centroid;
  if (zone.polygon.empty()) {
    return centroid;
  }

  for (const auto & point : zone.polygon) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x /= static_cast<double>(zone.polygon.size());
  centroid.y /= static_cast<double>(zone.polygon.size());
  return centroid;
}

bool isBetterZone(const SemanticZone & candidate, const SemanticZone * current)
{
  if (current == nullptr) {
    return true;
  }
  if (candidate.priority != current->priority) {
    return candidate.priority > current->priority;
  }
  return candidate.cost > current->cost;
}

}  // namespace

class TerrainZoneMonitor : public rclcpp::Node
{
public:
  TerrainZoneMonitor()
  : Node("terrain_zone_monitor"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    zones_file_ = declare_parameter<std::string>("zones_file", "");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_yaw_odom");
    footprint_radius_ = std::max(0.0, declare_parameter<double>("footprint_radius", 0.42));
    sample_count_ = std::max(0, static_cast<int>(declare_parameter<int>("sample_count", 16)));
    update_frequency_ = std::max(0.1, declare_parameter<double>("update_frequency", 20.0));
    marker_publish_frequency_ =
      std::max(0.1, declare_parameter<double>("marker_publish_frequency", 1.0));
    normal_state_id_ = clampStateId(declare_parameter<int>("normal_state_id", 0));
    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    state_topic_ = declare_parameter<std::string>("state_topic", "/sentry_terrain_state");
    state_name_topic_ =
      declare_parameter<std::string>("state_name_topic", "/sentry_terrain_zone_name");
    speed_limit_topic_ =
      declare_parameter<std::string>("speed_limit_topic", "/sentry_terrain_speed_limit");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/terrain_semantic_markers");

    loadZones();

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    state_pub_ = create_publisher<std_msgs::msg::UInt8>(state_topic_, state_qos);
    state_name_pub_ = create_publisher<std_msgs::msg::String>(state_name_topic_, state_qos);
    speed_limit_pub_ = create_publisher<std_msgs::msg::Float32>(speed_limit_topic_, state_qos);

    if (publish_markers_) {
      auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, marker_qos);
      publishMarkers();
    }

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / update_frequency_));
    timer_ = create_wall_timer(timer_period, std::bind(&TerrainZoneMonitor::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "terrain_zone_monitor loaded %zu semantic zones from '%s'.",
      zone_map_.zones.size(), zones_file_.c_str());
  }

private:
  void loadZones()
  {
    zone_map_.frame_id = global_frame_;
    zone_map_.zones.clear();

    if (zones_file_.empty()) {
      RCLCPP_WARN(get_logger(), "No terrain semantic zones_file configured.");
      return;
    }

    std::string error;
    if (!loadSemanticZoneMap(zones_file_, &zone_map_, &error)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load terrain semantic zones from '%s': %s", zones_file_.c_str(),
        error.c_str());
      zone_map_.zones.clear();
      zone_map_.frame_id = global_frame_;
    }
  }

  void onTimer()
  {
    publishMarkersIfDue();

    geometry_msgs::msg::TransformStamped robot_transform;
    try {
      robot_transform = tf_buffer_.lookupTransform(
        zone_map_.frame_id.empty() ? global_frame_ : zone_map_.frame_id, robot_base_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "terrain_zone_monitor cannot transform '%s' to '%s': %s", robot_base_frame_.c_str(),
        (zone_map_.frame_id.empty() ? global_frame_ : zone_map_.frame_id).c_str(), ex.what());
      return;
    }

    const auto & translation = robot_transform.transform.translation;
    const auto samples = makeFootprintSamples(translation.x, translation.y);
    const SemanticZone * best_zone = nullptr;

    for (const auto & sample : samples) {
      for (const auto & zone : zone_map_.zones) {
        if (!containsPoint(zone, sample.x, sample.y)) {
          continue;
        }
        if (isBetterZone(zone, best_zone)) {
          best_zone = &zone;
        }
      }
    }

    publishState(best_zone);
  }

  std::vector<Point2D> makeFootprintSamples(const double center_x, const double center_y) const
  {
    std::vector<Point2D> samples;
    samples.reserve(static_cast<std::size_t>(sample_count_) + 1U);
    samples.push_back({center_x, center_y});

    if (footprint_radius_ <= 0.0 || sample_count_ <= 0) {
      return samples;
    }

    for (int i = 0; i < sample_count_; ++i) {
      const double theta = kTwoPi * static_cast<double>(i) / static_cast<double>(sample_count_);
      samples.push_back(
        {center_x + footprint_radius_ * std::cos(theta),
         center_y + footprint_radius_ * std::sin(theta)});
    }
    return samples;
  }

  void publishState(const SemanticZone * zone)
  {
    const std::uint8_t state_id = zone == nullptr ? normal_state_id_ : zone->state_id;
    const std::string zone_name = zone == nullptr ? "normal" : zone->name;
    const double speed_limit = zone == nullptr ? -1.0 : zone->speed_limit;

    std_msgs::msg::UInt8 state_msg;
    state_msg.data = state_id;
    state_pub_->publish(state_msg);

    std_msgs::msg::String state_name_msg;
    state_name_msg.data = zone_name;
    state_name_pub_->publish(state_name_msg);

    std_msgs::msg::Float32 speed_limit_msg;
    speed_limit_msg.data = static_cast<float>(speed_limit);
    speed_limit_pub_->publish(speed_limit_msg);

    if (!has_last_state_ || last_state_id_ != state_id || last_zone_name_ != zone_name) {
      RCLCPP_INFO(
        get_logger(), "Terrain semantic state: id=%u zone='%s' speed_limit=%.3f",
        static_cast<unsigned int>(state_id), zone_name.c_str(), speed_limit);
      has_last_state_ = true;
      last_state_id_ = state_id;
      last_zone_name_ = zone_name;
    }
  }

  void publishMarkersIfDue()
  {
    if (!publish_markers_ || !marker_pub_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / marker_publish_frequency_);
    if (markers_published_ && now - last_marker_publish_time_ < period) {
      return;
    }

    publishMarkers();
    last_marker_publish_time_ = now;
    markers_published_ = true;
  }

  void publishMarkers()
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = zone_map_.frame_id.empty() ? global_frame_ : zone_map_.frame_id;
    clear_marker.header.stamp = now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 1;
    for (const auto & zone : zone_map_.zones) {
      if (!isValidPolygon(zone)) {
        continue;
      }

      visualization_msgs::msg::Marker line_marker;
      line_marker.header = clear_marker.header;
      line_marker.ns = "terrain_semantic_zones";
      line_marker.id = marker_id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.05;
      setMarkerColor(&line_marker, zone.kind);

      for (const auto & point : zone.polygon) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x;
        marker_point.y = point.y;
        marker_point.z = 0.05;
        line_marker.points.push_back(marker_point);
      }
      geometry_msgs::msg::Point first_point;
      first_point.x = zone.polygon.front().x;
      first_point.y = zone.polygon.front().y;
      first_point.z = 0.05;
      line_marker.points.push_back(first_point);
      marker_array.markers.push_back(line_marker);

      visualization_msgs::msg::Marker text_marker;
      text_marker.header = clear_marker.header;
      text_marker.ns = "terrain_semantic_zone_labels";
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.orientation.w = 1.0;
      const auto centroid = centroidOf(zone);
      text_marker.pose.position.x = centroid.x;
      text_marker.pose.position.y = centroid.y;
      text_marker.pose.position.z = 0.35;
      text_marker.scale.z = 0.25;
      text_marker.text = zone.name;
      setMarkerColor(&text_marker, zone.kind);
      marker_array.markers.push_back(text_marker);
    }

    marker_pub_->publish(marker_array);
  }

  std::string zones_file_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string state_topic_;
  std::string state_name_topic_;
  std::string speed_limit_topic_;
  std::string marker_topic_;
  double footprint_radius_{0.42};
  int sample_count_{16};
  double update_frequency_{20.0};
  double marker_publish_frequency_{1.0};
  std::uint8_t normal_state_id_{0};
  bool publish_markers_{true};
  bool has_last_state_{false};
  std::uint8_t last_state_id_{0};
  std::string last_zone_name_;
  bool markers_published_{false};
  std::chrono::steady_clock::time_point last_marker_publish_time_;
  SemanticZoneMap zone_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pb_nav2_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pb_nav2_plugins::TerrainZoneMonitor>());
  rclcpp::shutdown();
  return 0;
}
