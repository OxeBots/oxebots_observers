#include "oxebots_observers/game_observer_node.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

// Helper para converter mm (unidade interna) para metros (exigido pelo OccupancyGrid)
constexpr double mm_to_m(double mm) { return mm / 1000.0; }

GameObserverNode::GameObserverNode() : Node("game_observer_node") {
  // DECLARAÇÃO DOS PARÂMETROS (Obrigatório para evitar o crash
  this->declare_parameter("is_yellow_team", true);
  this->declare_parameter("invert_sides", false);
  this->declare_parameter("team_size", 3);
  this->declare_parameter("map_resolution", 0.05);
  this->declare_parameter("robot_inflation_radius", 150.0);
  this->declare_parameter("topic_retention", 10);
  this->declare_parameter("vision_topic", "/ssl_vision_bridge/vision_messages");
  this->declare_parameter("referee_topic", "/gc_multicast_bridge/referee_messages");
  this->declare_parameter("publisher_topic", "game_data");
  // 0.3 é um chute inicial (a escala típica de confidence da visão SSL costuma ir bem alta,
  // 0.8-1.0, quando a bola está clara, e cair bastante quando parcialmente ocluída) — não pude
  // testar contra dados reais desta sessão, então precisa de ajuste empírico observando o valor
  // real de confidence nos frames onde a bola trava perto do robô.
  this->declare_parameter("min_ball_confidence", 0.3);

  is_yellow_team = this->get_parameter("is_yellow_team").as_bool();
  invert_sides = this->get_parameter("invert_sides").as_bool();
  team_size = this->get_parameter("team_size").as_int();
  min_ball_confidence_ = this->get_parameter("min_ball_confidence").as_double();

  // Publicadores para a Estratégia Oxebots
  game_publisher = this->create_publisher<oxebots_interfaces::msg::GameData>(
    get_parameter("publisher_topic").as_string(), get_parameter("topic_retention").as_int());
  
  map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable());

  ball_publisher = this->create_publisher<oxebots_interfaces::msg::BallPosition>(
    "ball_data", get_parameter("topic_retention").as_int());
  
  robot_publisher = this->create_publisher<oxebots_interfaces::msg::RobotPosition>(
    "robot_data", get_parameter("topic_retention").as_int());

  geometry_publisher = this->create_publisher<oxebots_interfaces::msg::SSLGeometryData>(
    "/field_geometry", rclcpp::QoS(1).transient_local().reliable());

  // Inscrições nos dados do Bridge (A-TEAM)
  vision_sub_ = this->create_subscription<ssl_league_msgs::msg::VisionWrapper>(
    get_parameter("vision_topic").as_string(), 10,
    std::bind(&GameObserverNode::vision_callback, this, std::placeholders::_1));

  referee_sub_ = this->create_subscription<ssl_league_msgs::msg::Referee>(
    get_parameter("referee_topic").as_string(), 10,
    std::bind(&GameObserverNode::referee_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Game Observer Oxebots iniciado (Com suporte a Multi-Câmera e Juiz).");
}

GameObserverNode::~GameObserverNode() {}

void GameObserverNode::vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr msg) {
  if (!msg->geometry.empty()) {
    const auto & geo = msg->geometry[0];
    oxebots_interfaces::msg::SSLGeometryData internal_geo;
    internal_geo.field.field_length = static_cast<int>(geo.field.field_length * 1000.0);
    internal_geo.field.field_width = static_cast<int>(geo.field.field_width * 1000.0);
    internal_geo.field.goal_width = static_cast<int>(geo.field.goal_width * 1000.0);
    internal_geo.field.goal_depth = static_cast<int>(geo.field.goal_depth * 1000.0);
    internal_geo.field.boundary_width = static_cast<int>(geo.field.boundary_width * 1000.0);
    internal_geo.field.penalty_area_depth = static_cast<int>(geo.field.penalty_area_depth * 1000.0);
    internal_geo.field.penalty_area_width = static_cast<int>(geo.field.penalty_area_width * 1000.0);
    internal_geo.field.center_circle_radius = static_cast<int>(geo.field.center_circle_radius * 1000.0);
    internal_geo.field.line_thickness = static_cast<int>(geo.field.line_thickness * 1000.0);
    internal_geo.field.goal_center_to_penalty_mark = static_cast<int>(geo.field.goal_center_to_penalty_mark * 1000.0);
    internal_geo.field.goal_height = static_cast<int>(geo.field.goal_height * 1000.0);
    internal_geo.field.ball_radius = geo.field.ball_radius * 1000.0;
    internal_geo.field.max_robot_radius = geo.field.max_robot_radius * 1000.0;
    
    last_geometry_data_ = internal_geo;
    geometry_publisher->publish(internal_geo);
  }

  // 2. Detecção (FUSÃO DE CÂMERAS)[cite: 24]
  if (msg->detection.empty()) return;
  const auto & det = msg->detection[0];

  auto update_robots = [&](const std::vector<ssl_league_msgs::msg::VisionDetectionRobot> & src, 
                            std::map<int, oxebots_interfaces::msg::RobotGameData> & dest) {
    for (const auto & r : src) {
      oxebots_interfaces::msg::RobotGameData data;
      data.id = r.robot_id;
      
      float x = r.pose.position.x * 1000.0; // mm
      float y = r.pose.position.y * 1000.0; // mm
      const auto & q = r.pose.orientation;
      float orientation = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      if (invert_sides) {
          x = -x;
          y = -y;
          orientation += M_PI;
          while (orientation > M_PI) orientation -= 2.0 * M_PI;
          while (orientation < -M_PI) orientation += 2.0 * M_PI;
      }

      data.x = x;
      data.y = y;
      data.orientation = orientation;
      dest[r.robot_id] = data; // Atualiza ID sem apagar os outros IDs
    }
  };

  if (is_yellow_team) {
    update_robots(det.robots_yellow, allies);
    update_robots(det.robots_blue, enemies);
  } else {
    update_robots(det.robots_blue, allies);
    update_robots(det.robots_yellow, enemies);
  }

  if (!det.balls.empty()) {
    // Entre múltiplas bolas candidatas no mesmo frame, usa a de maior confiança em vez de
    // sempre confiar cegamente em balls[0].
    const auto* best_ball = &det.balls[0];
    for (const auto& b : det.balls) {
      if (b.confidence > best_ball->confidence) best_ball = &b;
    }

    if (best_ball->confidence >= min_ball_confidence_) {
      float bx = best_ball->pos.x * 1000.0;
      float by = best_ball->pos.y * 1000.0;
      if (invert_sides) {
          bx = -bx;
          by = -by;
      }
      ball_data.x = bx;
      ball_data.y = by;
      ball_data.confidence = best_ball->confidence;
      is_ball_present = true;
    }
    // Confiança abaixo do limiar: não atualiza ball_data. Mantém a última posição boa
    // conhecida em vez de aceitar uma leitura de baixa confiança (bola ocluída) como se fosse
    // uma posição nova e real.
  }
  validate_data();
}

void GameObserverNode::referee_callback(const ssl_league_msgs::msg::Referee::SharedPtr msg) {
  // 1. Logs de Status Geral (Compactos via Throttle)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
    "--- ESTADO ATUAL DO JOGO ---");
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Time: %s (Y) vs %s (B) | Placar: %d x %d", 
    msg->yellow.name.c_str(), msg->blue.name.c_str(), msg->yellow.score, msg->blue.score);

  // 2. Monitoramento de Comandos e Estágios
  static uint8_t last_cmd = 255;
  if (msg->command != last_cmd) {
    RCLCPP_INFO(this->get_logger(), "Novo Comando recebido: [%d] no Estágio: [%d]", msg->command, msg->stage);
    last_cmd = msg->command;
  }

  // 3. Acesso aos Cartões e Timeouts
  if (msg->yellow.red_cards > 0 || msg->blue.red_cards > 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "ALERTA: Jogadores expulsos detectados! (Y:%d, B:%d)", msg->yellow.red_cards, msg->blue.red_cards);
  }

  // 4. Acesso à Posição de Falta / Lançamento (Designated Position)
  // O bridge da A-TEAM envia isso como um array opcional (0 ou 1 elemento)
  if (!msg->designated_position.empty()) {
    double px = msg->designated_position[0].x * 1000.0; // Convertendo para mm
    double py = msg->designated_position[0].y * 1000.0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ponto de falta detectado: x=%.1f, y=%.1f", px, py);
  }

  // 5. Verificação de Eventos de Jogo (Faltas detectadas pelo AutoRef)
  if (!msg->game_events.empty()) {
    for (const auto & event : msg->game_events) {
       RCLCPP_DEBUG(this->get_logger(), "Evento de jogo ID: %s", event.id.c_str());
    }
  }

  // DICA: Agora você tem o objeto 'msg' inteiro disponível para popular 
  // suas estruturas de tomada de decisão ou sua Behavior Tree.
}

void GameObserverNode::validate_data() {
  if (is_ball_present) {
    publish_game_data();
    if (last_geometry_data_.has_value()) publish_occupancy_grid_map();
    is_ball_present = false; 
  }
}

void GameObserverNode::publish_game_data() {
  oxebots_interfaces::msg::GameData msg;
  for (const auto& pair : allies) msg.robots.allies.push_back(pair.second);
  for (const auto& pair : enemies) msg.robots.enemies.push_back(pair.second);
  msg.ball = ball_data;
  game_publisher->publish(msg);
  ball_publisher->publish(msg.ball);
  robot_publisher->publish(msg.robots);
}

void GameObserverNode::publish_occupancy_grid_map() {
  if (!last_geometry_data_.has_value()) return;
  const auto& field_geo = last_geometry_data_->field;
  double res = get_parameter("map_resolution").as_double();
  double f_l = mm_to_m(field_geo.field_length);
  double f_w = mm_to_m(field_geo.field_width);
  double margin = 0.6;
  double t_l = f_l + (2.0 * margin);
  double t_w = f_w + (2.0 * margin);

  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.frame_id = "map"; map_msg.header.stamp = now();
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