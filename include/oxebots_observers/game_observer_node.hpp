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
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/ssl_geometry_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ssl_league_msgs/msg/vision_wrapper.hpp" // Novo bridge
#include "ssl_league_msgs/msg/referee.hpp"        // Novo bridge

class GameObserverNode : public rclcpp::Node {
public:
  GameObserverNode();
  ~GameObserverNode();

private:
  // Subscrições (Bridge A-TEAM)
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_sub_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr referee_sub_;

  // Publicadores (Mantidos para a Estratégia Oxebots)
  rclcpp::Publisher<oxebots_interfaces::msg::GameData>::SharedPtr game_publisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  rclcpp::Publisher<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_publisher;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotPosition>::SharedPtr robot_publisher;
  rclcpp::Publisher<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_publisher;

  // Armazenamento Interno (em Milímetros, como o original)
  std::map<int, oxebots_interfaces::msg::RobotGameData> allies;
  std::map<int, oxebots_interfaces::msg::RobotGameData> enemies;
  oxebots_interfaces::msg::BallPosition ball_data;
  std::optional<oxebots_interfaces::msg::SSLGeometryData> last_geometry_data_;

  // Callbacks e Lógica
  void vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr msg);
  void referee_callback(const ssl_league_msgs::msg::Referee::SharedPtr msg);
  void validate_data();
  void publish_game_data();
  void publish_occupancy_grid_map();

  uint team_size;
  bool is_ball_present = false;
  bool is_yellow_team;
  bool invert_sides;

  // Confiança mínima (0.0-1.0) para aceitar uma detecção de bola como posição nova válida.
  // Sem isso, uma detecção de baixa confiança (tipicamente a bola ocluída pela câmera pelo
  // próprio robô, comum bem antes/durante o chute) era aceita como se fosse uma leitura real,
  // congelando ball_data na última posição "vista" mesmo com a bola já tendo saído dali —
  // visto no log como coordenadas de bola idênticas por 14s+ enquanto o robô ficava preso
  // repetindo aproximação+chute contra um alvo fantasma.
  double min_ball_confidence_;
};
