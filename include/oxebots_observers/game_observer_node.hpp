// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <algorithm>
#include <vector>

#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/ssl_geometry_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <optional>

class GameObserverNode : public rclcpp::Node
{
public:
  GameObserverNode();
  ~GameObserverNode();

private:
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr robot_subscriber;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_subscriber;
  rclcpp::Subscription<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_subscriber;
  rclcpp::Publisher<oxebots_interfaces::msg::GameData>::SharedPtr game_publisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  std::map<int, oxebots_interfaces::msg::RobotGameData> allies;
  std::map<int, oxebots_interfaces::msg::RobotGameData> enemies;
  oxebots_interfaces::msg::BallPosition ball_data;

  std::optional<oxebots_interfaces::msg::SSLGeometryData> last_geometry_data_;

  void robot_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg);

  void ball_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg);

  void geometry_callback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg);

  void validate_data();

  void publish_game_data();

  void publish_occupancy_grid_map();

  //int contains_robot(
  //  const std::vector<oxebots_interfaces::msg::RobotGameData> & robots,
  //  const oxebots_interfaces::msg::RobotGameData & robot);

  uint team_size;
  bool is_ball_present;
  //uint allies_count;
  //uint enemies_count;
};
