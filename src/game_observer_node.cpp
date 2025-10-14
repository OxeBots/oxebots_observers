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

#include "oxebots_observers/game_observer_node.hpp"

GameObserverNode::GameObserverNode()
: Node("game_observer_node")
{
  RCLCPP_DEBUG(get_logger(), "Game Observer Node starting...");
  declare_parameter("robot_amount", 3);
  declare_parameter("subscriber_robot_topic", "robot_data");
  declare_parameter("subscriber_ball_topic", "ball_data");
  declare_parameter("publisher_topic", "game_data");
  declare_parameter("topic_retention", 10);
  declare_parameter("team_size", 3);

  RCLCPP_DEBUG(get_logger(), "Creating the publisher...");

  game_publisher = create_publisher<oxebots_interfaces::msg::GameData>(
    get_parameter("publisher_topic").as_string(), get_parameter("topic_retention").as_int());

  RCLCPP_DEBUG(get_logger(), "Creating the subscribers...");

  robot_subscriber = create_subscription<oxebots_interfaces::msg::RobotPosition>(
    get_parameter("subscriber_robot_topic").as_string(), 10,
    std::bind(&GameObserverNode::robot_callback, this, std::placeholders::_1));

  ball_subscriber = create_subscription<oxebots_interfaces::msg::BallPosition>(
    get_parameter("subscriber_ball_topic").as_string(), 10,
    std::bind(&GameObserverNode::ball_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Initializing the structures...");
  team_size = get_parameter("team_size").as_int();
  //allies.resize(team_size);
  //enemies.resize(team_size);

  RCLCPP_INFO(get_logger(), "Game Observer Node Started");
}

GameObserverNode::~GameObserverNode()
{
  RCLCPP_INFO(this->get_logger(), "Game Observer Node Stopped");
}

// game_observer_node.cpp

void GameObserverNode::robot_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Robot data received");

  // Atualiza ou insere aliados
  for (const auto & ally_robot : msg->allies) {
    int id = ally_robot.id;

    // Condição: Atualize o robô se ele já existe, OU adicione-o se for novo E houver espaço.
    if (allies.count(id) || allies.size() < team_size) {
      allies[id] = ally_robot;
    }
  }

  // Atualiza ou insere inimigos
  for (const auto & enemy_robot : msg->enemies) {
    int id = enemy_robot.id;
    if (enemies.count(id) || enemies.size() < team_size) {
      enemies[id] = enemy_robot;
    }
  }

  validate_data();
}

void GameObserverNode::ball_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Ball data received");
  ball_data = *msg.get();
  is_ball_present = true;
  validate_data();
}

void GameObserverNode::validate_data()
{
  if (is_ball_present) {
    publish_game_data();
    // Os dados dos robôs são mantidos. Apenas resetamos os contadores de recebimento.
    is_ball_present = false;
  } else {
    RCLCPP_DEBUG(get_logger(), "Data not complete yet...");
  }
}

// game_observer_node.cpp

void GameObserverNode::publish_game_data()
{
  RCLCPP_DEBUG(get_logger(), "Publishing game data...");
  oxebots_interfaces::msg::GameData game_data;

  // Converte o mapa de aliados para um vetor
  std::vector<oxebots_interfaces::msg::RobotGameData> allies_vec;
  for (const auto& pair : allies) {
    allies_vec.push_back(pair.second); // pair.second contém o objeto RobotGameData
  }

  // Converte o mapa de inimigos para um vetor
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies_vec;
  for (const auto& pair : enemies) {
    enemies_vec.push_back(pair.second);
  }

  game_data.robots.allies = allies_vec;
  game_data.robots.enemies = enemies_vec;
  game_data.ball = ball_data;
  game_publisher->publish(game_data);
}

/*int GameObserverNode::contains_robot(
  const std::vector<oxebots_interfaces::msg::RobotGameData> & robots,
  const oxebots_interfaces::msg::RobotGameData & robot)
{
  auto it = std::find(robots.begin(), robots.end(), robot);
  if (it != robots.end()) {
    return std::distance(robots.begin(), it);

  } else {
    return -1;
  }
}*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameObserverNode>());
  rclcpp::shutdown();
  return 0;
}
