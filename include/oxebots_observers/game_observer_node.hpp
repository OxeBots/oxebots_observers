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

class GameObserverNode : public rclcpp::Node
{
public:
  GameObserverNode();
  ~GameObserverNode();

private:
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr robot_subscriber;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_subscriber;
  rclcpp::Publisher<oxebots_interfaces::msg::GameData>::SharedPtr game_publisher;
  std::vector<oxebots_interfaces::msg::RobotGameData> allies;
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies;
  oxebots_interfaces::msg::BallPosition ball_data;

  void robot_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg);

  void ball_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg);

  void validate_data();

  void publish_game_data();

  int contains_robot(
    const std::vector<oxebots_interfaces::msg::RobotGameData> & robots,
    const oxebots_interfaces::msg::RobotGameData & robot);

  uint team_size;
  bool is_ball_present;
  uint allies_count;
  uint enemies_count;
};
