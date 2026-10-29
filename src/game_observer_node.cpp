#include "oxebots_observers/game_observer_node.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

// Helper para converter mm (unidade interna) para metros (exigido pelo OccupancyGrid)
constexpr double mm_to_m(double mm) { return mm / 1000.0; }

GameObserverNode::GameObserverNode(const rclcpp::NodeOptions & options)
: Node("game_observer_node", options) {
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
  this->declare_parameter("min_robot_confidence", 0.3);
  this->declare_parameter("robot_expiration_ms", 500.0);
  this->declare_parameter("ball_expiration_ms", 500.0);
  this->declare_parameter("multi_camera_window_ms", 25.0);

  is_yellow_team = this->get_parameter("is_yellow_team").as_bool();
  invert_sides = this->get_parameter("invert_sides").as_bool();
  team_size = this->get_parameter("team_size").as_int();
  min_ball_confidence_ = this->get_parameter("min_ball_confidence").as_double();
  min_robot_confidence_ = this->get_parameter("min_robot_confidence").as_double();
  robot_expiration_ms_ = this->get_parameter("robot_expiration_ms").as_double();
  ball_expiration_ms_ = this->get_parameter("ball_expiration_ms").as_double();
  multi_camera_window_ms_ = this->get_parameter("multi_camera_window_ms").as_double();

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
  for (const auto & geo : msg->geometry) {
    if (geo.field.field_length <= 0.0f || geo.field.field_width <= 0.0f) {
      continue;
    }
    oxebots_interfaces::msg::SSLGeometryData internal_geo;
    internal_geo.field.field_length = static_cast<int>( std::lround(geo.field.field_length * 1000.0) );
    internal_geo.field.field_width = static_cast<int>( std::lround(geo.field.field_width * 1000.0) );
    internal_geo.field.goal_width = static_cast<int>( std::lround(geo.field.goal_width * 1000.0) );
    internal_geo.field.goal_depth = static_cast<int>( std::lround(geo.field.goal_depth * 1000.0) );
    internal_geo.field.boundary_width = static_cast<int>( std::lround(geo.field.boundary_width * 1000.0) );
    internal_geo.field.penalty_area_depth = static_cast<int>( std::lround(geo.field.penalty_area_depth * 1000.0) );
    internal_geo.field.penalty_area_width = static_cast<int>( std::lround(geo.field.penalty_area_width * 1000.0) );
    internal_geo.field.center_circle_radius = static_cast<int>( std::lround(geo.field.center_circle_radius * 1000.0) );
    internal_geo.field.line_thickness = static_cast<int>( std::lround(geo.field.line_thickness * 1000.0) );
    internal_geo.field.goal_center_to_penalty_mark = static_cast<int>( std::lround(geo.field.goal_center_to_penalty_mark * 1000.0) );
    internal_geo.field.goal_height = static_cast<int>( std::lround(geo.field.goal_height * 1000.0) );
    internal_geo.field.ball_radius = geo.field.ball_radius * 1000.0;
    internal_geo.field.max_robot_radius = geo.field.max_robot_radius * 1000.0;

    // O grSim e algumas versões do SSL-Vision enviam os segmentos de linha (field_lines / field_arcs),
    // mas deixam os campos opcionais penalty_area_depth, penalty_area_width e center_circle_radius zerados.
    if (internal_geo.field.penalty_area_depth <= 0 || internal_geo.field.penalty_area_width <= 0) {
      for (const auto & line : geo.field.field_lines) {
        if (line.name == "LeftPenaltyStretch" || line.name == "RightPenaltyStretch") {
          int width_mm = static_cast<int>(std::lround(std::abs(line.p2.y - line.p1.y) * 1000.0));
          int depth_mm = static_cast<int>(std::lround((geo.field.field_length / 2.0 - std::abs(line.p1.x)) * 1000.0));
          if (internal_geo.field.penalty_area_width <= 0 && width_mm > 0) {
            internal_geo.field.penalty_area_width = width_mm;
          }
          if (internal_geo.field.penalty_area_depth <= 0 && depth_mm > 0) {
            internal_geo.field.penalty_area_depth = depth_mm;
          }
        } else if (line.name == "LeftFieldLeftPenaltyStretch" || line.name == "LeftFieldRightPenaltyStretch" ||
                   line.name == "RightFieldLeftPenaltyStretch" || line.name == "RightFieldRightPenaltyStretch") {
          int depth_mm = static_cast<int>(std::lround(std::abs(line.p2.x - line.p1.x) * 1000.0));
          int width_mm = static_cast<int>(std::lround(2.0 * std::abs(line.p1.y) * 1000.0));
          if (internal_geo.field.penalty_area_depth <= 0 && depth_mm > 0) {
            internal_geo.field.penalty_area_depth = depth_mm;
          }
          if (internal_geo.field.penalty_area_width <= 0 && width_mm > 0) {
            internal_geo.field.penalty_area_width = width_mm;
          }
        }
      }
    }

    if (internal_geo.field.center_circle_radius <= 0) {
      for (const auto & arc : geo.field.field_arcs) {
        if (arc.name == "CenterCircle" && arc.radius > 0.0f) {
          internal_geo.field.center_circle_radius = static_cast<int>(std::lround(arc.radius * 1000.0));
          break;
        }
      }
    }

    // Fallback proporcional pela regra oficial da SSL caso nem os campos opcionais nem field_lines venham preenchidos
    if (internal_geo.field.penalty_area_depth <= 0 || internal_geo.field.penalty_area_width <= 0) {
      if (internal_geo.field.field_length <= 5500) {
        // SSL Entry League (ex: 4500 x 3000 mm)
        if (internal_geo.field.penalty_area_depth <= 0) internal_geo.field.penalty_area_depth = 500;
        if (internal_geo.field.penalty_area_width <= 0) internal_geo.field.penalty_area_width = 1350;
      } else if (internal_geo.field.field_length <= 10500) {
        // SSL Division B (ex: 9000 x 6000 mm)
        if (internal_geo.field.penalty_area_depth <= 0) internal_geo.field.penalty_area_depth = 1000;
        if (internal_geo.field.penalty_area_width <= 0) internal_geo.field.penalty_area_width = 2000;
      } else {
        // SSL Division A (ex: 12000 x 9000 mm)
        if (internal_geo.field.penalty_area_depth <= 0) internal_geo.field.penalty_area_depth = 1800;
        if (internal_geo.field.penalty_area_width <= 0) internal_geo.field.penalty_area_width = 3600;
      }
    }
    if (internal_geo.field.center_circle_radius <= 0) {
      internal_geo.field.center_circle_radius = 500;
    }

    last_geometry_data_ = internal_geo;
    geometry_publisher->publish(internal_geo);
  }

  // 2. Detecção (FUSÃO DE CÂMERAS COM CONFIANÇA)
  if (msg->detection.empty()) return;

  const rclcpp::Time now_time = this->now();

  // Seleciona, dentro deste pacote (que pode conter múltiplos VisionDetectionFrame de diferentes
  // câmeras), a detecção de maior confiança para cada robô aliado e adversário.
  std::map<int, const ssl_league_msgs::msg::VisionDetectionRobot *> best_allies_in_packet;
  std::map<int, const ssl_league_msgs::msg::VisionDetectionRobot *> best_enemies_in_packet;
  const ssl_league_msgs::msg::VisionDetectionBall * best_ball = nullptr;

  auto collect_best_robots = [&](
      const std::vector<ssl_league_msgs::msg::VisionDetectionRobot> & src,
      std::map<int, const ssl_league_msgs::msg::VisionDetectionRobot *> & best_map) {
    for (const auto & r : src) {
      if (r.confidence < min_robot_confidence_) {
        continue;
      }
      int id = static_cast<int>(r.robot_id);
      auto it = best_map.find(id);
      if (it == best_map.end() || r.confidence >= it->second->confidence) {
        best_map[id] = &r;
      }
    }
  };

  for (const auto & det : msg->detection) {
    if (is_yellow_team) {
      collect_best_robots(det.robots_yellow, best_allies_in_packet);
      collect_best_robots(det.robots_blue, best_enemies_in_packet);
    } else {
      collect_best_robots(det.robots_blue, best_allies_in_packet);
      collect_best_robots(det.robots_yellow, best_enemies_in_packet);
    }

    for (const auto & b : det.balls) {
      if (best_ball == nullptr || b.confidence > best_ball->confidence) {
        best_ball = &b;
      }
    }
  }

  auto apply_robot_updates = [&](
      const std::map<int, const ssl_league_msgs::msg::VisionDetectionRobot *> & best_map,
      std::map<int, oxebots_interfaces::msg::RobotGameData> & dest,
      std::map<int, rclcpp::Time> & last_seen_map) {
    for (const auto & [id, r_ptr] : best_map) {
      const auto & r = *r_ptr;

      // Se outra câmera reportou este mesmo robô dentro da janela multi-câmera com
      // confiança estritamente maior, preserva a pose de maior confiança e apenas renova o stamp.
      auto seen_it = last_seen_map.find(id);
      auto dest_it = dest.find(id);
      if (seen_it != last_seen_map.end() && dest_it != dest.end()) {
        const double age_ms = (now_time - seen_it->second).seconds() * 1000.0;
        if (age_ms >= 0.0 && age_ms <= multi_camera_window_ms_ &&
            dest_it->second.confidence > r.confidence) {
          seen_it->second = now_time;
          dest_it->second.stamp = now_time;
          continue;
        }
      }

      oxebots_interfaces::msg::RobotGameData data;
      data.id = r.robot_id;

      float x = r.pose.position.x * 1000.0f; // mm
      float y = r.pose.position.y * 1000.0f; // mm
      const auto & q = r.pose.orientation;
      float orientation = static_cast<float>(
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));

      if (invert_sides) {
        x = -x;
        y = -y;
        orientation += static_cast<float>(M_PI);
        while (orientation > static_cast<float>(M_PI)) orientation -= static_cast<float>(2.0 * M_PI);
        while (orientation < static_cast<float>(-M_PI)) orientation += static_cast<float>(2.0 * M_PI);
      }

      data.x = x;
      data.y = y;
      data.orientation = orientation;
      data.confidence = r.confidence;
      data.stamp = now_time;
      dest[id] = data;
      last_seen_map[id] = now_time;
    }
  };

  apply_robot_updates(best_allies_in_packet, allies, allies_last_seen_);
  apply_robot_updates(best_enemies_in_packet, enemies, enemies_last_seen_);

  if (best_ball != nullptr && best_ball->confidence >= min_ball_confidence_) {
    bool keep_previous_higher_conf = false;
    if (last_ball_seen_time_.has_value()) {
      const double ball_age_ms = (now_time - *last_ball_seen_time_).seconds() * 1000.0;
      if (ball_age_ms >= 0.0 && ball_age_ms <= multi_camera_window_ms_ &&
          ball_data.confidence > best_ball->confidence) {
        keep_previous_higher_conf = true;
      }
    }

    if (!keep_previous_higher_conf) {
      float bx = best_ball->pos.x * 1000.0f;
      float by = best_ball->pos.y * 1000.0f;
      if (invert_sides) {
        bx = -bx;
        by = -by;
      }
      ball_data.x = bx;
      ball_data.y = by;
      ball_data.z = 0.0f;
      ball_data.confidence = best_ball->confidence;
    }
    ball_data.stamp = now_time;
    last_ball_seen_time_ = now_time;
    is_ball_present = true;
  }
  // Confiança abaixo do limiar ou sem bola no frame: não atualiza (x, y) de ball_data,
  // mantendo a última posição boa conhecida, mas segue para validate_data() para expirar
  // robôs e publicar /game_data mesmo sem bola.

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

void GameObserverNode::prune_expired_data(const rclcpp::Time & now_time) {
  auto prune_map = [&](
      std::map<int, oxebots_interfaces::msg::RobotGameData> & robots_map,
      std::map<int, rclcpp::Time> & last_seen_map) {
    for (auto it = robots_map.begin(); it != robots_map.end(); ) {
      auto seen_it = last_seen_map.find(it->first);
      if (seen_it == last_seen_map.end() ||
          (now_time - seen_it->second).seconds() * 1000.0 > robot_expiration_ms_) {
        if (seen_it != last_seen_map.end()) {
          last_seen_map.erase(seen_it);
        }
        it = robots_map.erase(it);
      } else {
        ++it;
      }
    }
  };

  prune_map(allies, allies_last_seen_);
  prune_map(enemies, enemies_last_seen_);

  if (last_ball_seen_time_.has_value()) {
    const double ball_age_ms = (now_time - *last_ball_seen_time_).seconds() * 1000.0;
    if (ball_age_ms > ball_expiration_ms_) {
      ball_data.confidence = 0.0f;
    }
  } else {
    ball_data.confidence = 0.0f;
  }
}

void GameObserverNode::validate_data() {
  prune_expired_data(this->now());
  publish_game_data();
  if (last_geometry_data_.has_value()) {
    publish_occupancy_grid_map();
  }
  is_ball_present = false;
}

void GameObserverNode::publish_game_data() {
  oxebots_interfaces::msg::GameData msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  for (const auto& pair : allies) msg.robots.allies.push_back(pair.second);
  for (const auto& pair : enemies) msg.robots.enemies.push_back(pair.second);
  msg.ball = ball_data;
  game_publisher->publish(msg);
  if (is_ball_present) {
    ball_publisher->publish(msg.ball);
  }
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

#ifndef OXEBOTS_OBSERVERS_NO_MAIN
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameObserverNode>());
  rclcpp::shutdown();
  return 0;
}
#endif