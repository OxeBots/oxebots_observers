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
  explicit GameObserverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
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
  std::map<int, rclcpp::Time> allies_last_seen_;
  std::map<int, rclcpp::Time> enemies_last_seen_;
  std::map<int, uint32_t> allies_last_camera_id_;
  std::map<int, uint32_t> enemies_last_camera_id_;
  oxebots_interfaces::msg::BallPosition ball_data;
  std::optional<rclcpp::Time> last_ball_seen_time_;
  std::optional<uint32_t> last_ball_camera_id_;
  std::optional<rclcpp::Time> last_publish_time_;
  std::optional<oxebots_interfaces::msg::SSLGeometryData> last_geometry_data_;

  // Callbacks e Lógica
  void vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr msg);
  void referee_callback(const ssl_league_msgs::msg::Referee::SharedPtr msg);
  void prune_expired_data(const rclcpp::Time & now_time);
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

  // Confiança mínima (0.0-1.0) para aceitar uma detecção de robô como pose nova válida.
  double min_robot_confidence_;

  // Tempo máximo (ms) sem detecção antes de expirar/remover um robô do mapa de aliados/adversários.
  double robot_expiration_ms_;

  // Tempo máximo (ms) sem detecção válida da bola antes de zerar sua confidence no GameData.
  double ball_expiration_ms_;

  // Janela temporal (ms) para fusão multi-câmera entre frames quase simultâneos (~60 Hz):
  // dentro dessa janela, uma detecção de menor confiança não sobrescreve outra de maior confiança.
  double multi_camera_window_ms_;
};
