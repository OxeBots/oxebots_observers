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
#include <nav_msgs/msg/occupancy_grid.hpp>

// Helper function to convert mm to meters
constexpr double mm_to_m(double mm) { return mm / 1000.0; }

GameObserverNode::GameObserverNode()
: Node("game_observer_node")
{
  RCLCPP_DEBUG(get_logger(), "Game Observer Node starting...");
  declare_parameter("robot_amount", 3);
  declare_parameter("subscriber_robot_topic", "robot_data");
  declare_parameter("subscriber_ball_topic", "ball_data");
  declare_parameter("publisher_topic", "game_data");
  declare_parameter("geometry_subscriber_topic", "field_geometry");
  declare_parameter("map_publisher_topic", "map");
  declare_parameter("topic_retention", 10);
  declare_parameter("team_size", 3);
  declare_parameter("map_resolution", 0.05); // 5 cm per cell
  declare_parameter("robot_inflation_radius", 150.0); // 150mm de raio para evitar colisões entre robôs

  RCLCPP_DEBUG(get_logger(), "Creating the publishers...");

  game_publisher = create_publisher<oxebots_interfaces::msg::GameData>(
    get_parameter("publisher_topic").as_string(), get_parameter("topic_retention").as_int());
  
  map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>(
    get_parameter("map_publisher_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());

  RCLCPP_DEBUG(get_logger(), "Creating the subscribers...");

  robot_subscriber = create_subscription<oxebots_interfaces::msg::RobotPosition>(
    get_parameter("subscriber_robot_topic").as_string(), 10,
    std::bind(&GameObserverNode::robot_callback, this, std::placeholders::_1));

  ball_subscriber = create_subscription<oxebots_interfaces::msg::BallPosition>(
    get_parameter("subscriber_ball_topic").as_string(), 10,
    std::bind(&GameObserverNode::ball_callback, this, std::placeholders::_1));
  
  geometry_subscriber = create_subscription<oxebots_interfaces::msg::SSLGeometryData>(
    get_parameter("geometry_subscriber_topic").as_string(), rclcpp::QoS(1).transient_local(),
    std::bind(&GameObserverNode::geometry_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Initializing the structures...");
  team_size = get_parameter("team_size").as_int();

  RCLCPP_INFO(get_logger(), "Game Observer Node Started");
}

GameObserverNode::~GameObserverNode()
{
  RCLCPP_INFO(this->get_logger(), "Game Observer Node Stopped");
}

void GameObserverNode::robot_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Robot data received");

  for (const auto & ally_robot : msg->allies) {
    int id = ally_robot.id;
    if (allies.count(id) || allies.size() < team_size) {
      allies[id] = ally_robot;
    }
  }

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

void GameObserverNode::geometry_callback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Field Geometry received!");
  last_geometry_data_ = *msg;
  // No need to call validate_data here, as map is generated only when robots/ball data arrive
}


void GameObserverNode::validate_data()
{
  if (is_ball_present && last_geometry_data_.has_value()) {
    RCLCPP_INFO_ONCE(get_logger(), "Dados completos recebidos! Iniciando publicação do mapa e game_data.");
    publish_game_data();
    publish_occupancy_grid_map();
    is_ball_present = false;
  } else {
    static int count = 0;
    if (count++ % 100 == 0) {
        RCLCPP_INFO(get_logger(), "Aguardando dados... (bola: %d, geometria: %d)", is_ball_present, last_geometry_data_.has_value());
    }
  }
}

void GameObserverNode::publish_game_data()
{
  RCLCPP_DEBUG(get_logger(), "Publishing game data...");
  oxebots_interfaces::msg::GameData game_data;

  std::vector<oxebots_interfaces::msg::RobotGameData> allies_vec;
  for (const auto& pair : allies) {
    allies_vec.push_back(pair.second);
  }

  std::vector<oxebots_interfaces::msg::RobotGameData> enemies_vec;
  for (const auto& pair : enemies) {
    enemies_vec.push_back(pair.second);
  }

  game_data.robots.allies = allies_vec;
  game_data.robots.enemies = enemies_vec;
  game_data.ball = ball_data;
  game_publisher->publish(game_data);
}

void GameObserverNode::publish_occupancy_grid_map()
{
  if (!last_geometry_data_.has_value()) {
    RCLCPP_WARN(get_logger(), "Cannot publish map: No geometry data received yet.");
    return;
  }

  const auto& field_geometry = last_geometry_data_->field;
  double map_resolution = get_parameter("map_resolution").as_double();

  // Convert field dimensions from mm to meters
  double field_length_m = mm_to_m(field_geometry.field_length);
  double field_width_m = mm_to_m(field_geometry.field_width);
  
  // Adicionar margem de 60cm em todas as direções (total de 1.2m extras por eixo)
  double margin_m = 0.6;
  double total_length_m = field_length_m + (2.0 * margin_m);
  double total_width_m = field_width_m + (2.0 * margin_m);

  // Calculate map dimensions in cells
  unsigned int map_width_cells = static_cast<unsigned int>(total_length_m / map_resolution);
  unsigned int map_height_cells = static_cast<unsigned int>(total_width_m / map_resolution);

  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();

  map_msg.info.resolution = map_resolution;
  map_msg.info.width = map_width_cells;
  map_msg.info.height = map_height_cells;

  // Set map origin (center of the field is (0,0) in world coordinates)
  map_msg.info.origin.position.x = -total_length_m / 2.0;
  map_msg.info.origin.position.y = -total_width_m / 2.0;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0; // No rotation

  map_msg.data.assign(map_width_cells * map_height_cells, 0); // Initialize with free space (0)

  // Mark robots as obstacles with inflation for safety
  double robot_radius_m = mm_to_m(get_parameter("robot_inflation_radius").as_double());
  int robot_radius_cells = static_cast<int>(robot_radius_m / map_resolution);

  auto mark_robot_as_obstacle = [&](double rx_mm, double ry_mm) {
    double rx_m = mm_to_m(rx_mm);
    double ry_m = mm_to_m(ry_mm);

    // Convert robot world coordinates to map grid coordinates
    int gx = static_cast<int>((rx_m - map_msg.info.origin.position.x) / map_resolution);
    int gy = static_cast<int>((ry_m - map_msg.info.origin.position.y) / map_resolution);

    // Mark a square around the robot's center
    for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
      for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
        int current_gx = gx + dx;
        int current_gy = gy + dy;

        // Check bounds
        if (current_gx >= 0 && current_gx < static_cast<int>(map_width_cells) &&
            current_gy >= 0 && current_gy < static_cast<int>(map_height_cells)) {
          map_msg.data[current_gx + current_gy * map_width_cells] = 100; // Occupied
        }
      }
    }
  };

  for (const auto& pair : enemies) {
    mark_robot_as_obstacle(pair.second.x, pair.second.y);
  }

  for (const auto& pair : allies) {
    mark_robot_as_obstacle(pair.second.x, pair.second.y);
  }

  auto mark_ball_as_obstacle = [&](double rx_mm, double ry_mm) {
    double rx_m = mm_to_m(rx_mm);
    double ry_m = mm_to_m(ry_mm);
    double ball_obstacle_radius_m = 0.10; // 100mm radius for the ball obstacle
    int ball_radius_cells = static_cast<int>(ball_obstacle_radius_m / map_resolution);

    int gx = static_cast<int>((rx_m - map_msg.info.origin.position.x) / map_resolution);
    int gy = static_cast<int>((ry_m - map_msg.info.origin.position.y) / map_resolution);

    for (int dx = -ball_radius_cells; dx <= ball_radius_cells; ++dx) {
      for (int dy = -ball_radius_cells; dy <= ball_radius_cells; ++dy) {
        int current_gx = gx + dx;
        int current_gy = gy + dy;
        if (current_gx >= 0 && current_gx < static_cast<int>(map_width_cells) &&
            current_gy >= 0 && current_gy < static_cast<int>(map_height_cells)) {
          map_msg.data[current_gx + current_gy * map_width_cells] = 100;
        }
      }
    }
  };

  // Mark the ball as an obstacle
  mark_ball_as_obstacle(ball_data.x, ball_data.y);

  map_publisher->publish(map_msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameObserverNode>());
  rclcpp::shutdown();
  return 0;
}
