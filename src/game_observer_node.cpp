#include "oxebots_observers/game_observer_node.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

constexpr double mm_to_m(double mm) { return mm / 1000.0; }

GameObserverNode::GameObserverNode() : Node("game_observer_node") {
  this->declare_parameter("is_yellow_team", true);
  this->declare_parameter("team_size", 3);
  this->declare_parameter("map_resolution", 0.05);
  this->declare_parameter("robot_inflation_radius", 150.0);
  this->declare_parameter("topic_retention", 10);
  this->declare_parameter("vision_topic", "/ssl_vision_bridge/vision_messages");
  this->declare_parameter("publisher_topic", "game_data");

  is_yellow_team = this->get_parameter("is_yellow_team").as_bool();
  team_size = this->get_parameter("team_size").as_int();

  game_publisher = this->create_publisher<oxebots_interfaces::msg::GameData>(
    get_parameter("publisher_topic").as_string(), get_parameter("topic_retention").as_int());
  
  map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable());

  vision_sub_ = this->create_subscription<ssl_league_msgs::msg::VisionWrapper>(
    get_parameter("vision_topic").as_string(), 10,
    std::bind(&GameObserverNode::vision_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Game Observer (Tradutor Multi-Camera) iniciado.");
}

GameObserverNode::~GameObserverNode() {}

void GameObserverNode::vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr msg) {
  // 1. Atualizar Geometria (se disponível)
  if (!msg->geometry.empty()) {
    const auto & geo = msg->geometry[0];
    oxebots_interfaces::msg::SSLGeometryData internal_geo;
    internal_geo.field.field_length = static_cast<int>(geo.field.field_length * 1000.0);
    internal_geo.field.field_width = static_cast<int>(geo.field.field_width * 1000.0);
    last_geometry_data_ = internal_geo;
  }

  // 2. Processar Detecção sem apagar robôs de outras câmeras
  if (msg->detection.empty()) return;
  const auto & det = msg->detection[0];

  // Helper para atualizar os robôs no mapa persistente
  auto update_robots = [&](const std::vector<ssl_league_msgs::msg::VisionDetectionRobot> & src, 
                            std::map<int, oxebots_interfaces::msg::RobotGameData> & dest) {
    for (const auto & r : src) {
      oxebots_interfaces::msg::RobotGameData data;
      data.id = r.robot_id;
      data.x = r.pose.position.x * 1000.0;
      data.y = r.pose.position.y * 1000.0;
      // Conversão Quaternion -> Yaw
      const auto & q = r.pose.orientation;
      data.orientation = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      
      // Atualiza ou insere o robô (MANTÉM O ESTADO PERSISTENTE)[cite: 24]
      dest[r.robot_id] = data;
    }
  };

  // NÃO usamos clear() para permitir a fusão de dados de diferentes câmeras[cite: 24]
  if (is_yellow_team) {
    update_robots(det.robots_yellow, allies);
    update_robots(det.robots_blue, enemies);
  } else {
    update_robots(det.robots_blue, allies);
    update_robots(det.robots_yellow, enemies);
  }

  if (!det.balls.empty()) {
    ball_data.x = det.balls[0].pos.x * 1000.0;
    ball_data.y = det.balls[0].pos.y * 1000.0;
    ball_data.z = det.balls[0].pos.z * 1000.0;
    is_ball_present = true;
  }

  validate_data();
}

void GameObserverNode::validate_data() {
  // Publica sempre que houver bola, acumulando robôs de todas as câmeras
  if (is_ball_present) {
    publish_game_data();
    if (last_geometry_data_.has_value()) {
      publish_occupancy_grid_map();
    }
    is_ball_present = false; 
  }
}

void GameObserverNode::publish_game_data() {
  oxebots_interfaces::msg::GameData msg;
  for (const auto& pair : allies) msg.robots.allies.push_back(pair.second);
  for (const auto& pair : enemies) msg.robots.enemies.push_back(pair.second);
  msg.ball = ball_data;
  game_publisher->publish(msg);
}

void GameObserverNode::publish_occupancy_grid_map() {
  if (!last_geometry_data_.has_value()) return;
  const auto& field_geo = last_geometry_data_->field;
  double res = get_parameter("map_resolution").as_double();
  double margin = 0.6;
  double t_l = mm_to_m(field_geo.field_length) + (2.0 * margin);
  double t_w = mm_to_m(field_geo.field_width) + (2.0 * margin);

  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();
  map_msg.info.resolution = res;
  map_msg.info.width = static_cast<uint32_t>(t_l / res);
  map_msg.info.height = static_cast<uint32_t>(t_w / res);
  map_msg.info.origin.position.x = -t_l / 2.0;
  map_msg.info.origin.position.y = -t_w / 2.0;
  map_msg.info.origin.orientation.w = 1.0;
  map_msg.data.assign(map_msg.info.width * map_msg.info.height, 0);

  double inf_r = mm_to_m(get_parameter("robot_inflation_radius").as_double());
  auto mark = [&](double rx_mm, double ry_mm) {
    int gx = static_cast<int>((mm_to_m(rx_mm) - map_msg.info.origin.position.x) / res);
    int gy = static_cast<int>((mm_to_m(ry_mm) - map_msg.info.origin.position.y) / res);
    int rc = static_cast<int>(inf_r / res);
    for (int dx = -rc; dx <= rc; ++dx) {
      for (int dy = -rc; dy <= rc; ++dy) {
        int cx = gx + dx; int cy = gy + dy;
        if (cx >= 0 && cx < (int)map_msg.info.width && cy >= 0 && cy < (int)map_msg.info.height)
          map_msg.data[cx + cy * map_msg.info.width] = 100;
      }
    }
  };
  for (const auto& p : enemies) mark(p.second.x, p.second.y);
  for (const auto& p : allies) mark(p.second.x, p.second.y);
  map_publisher->publish(map_msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameObserverNode>());
  rclcpp::shutdown();
  return 0;
}