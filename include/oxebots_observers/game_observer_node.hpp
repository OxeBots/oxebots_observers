#pragma once
#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <algorithm>

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
    rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      robot_subscriber;
    rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr
      ball_subscriber;
    rclcpp::Publisher<oxebots_interfaces::msg::GameData>::SharedPtr
      game_publisher;
    std::vector<oxebots_interfaces::msg::RobotGameData> allies;
    std::vector<oxebots_interfaces::msg::RobotGameData> enemies;
    oxebots_interfaces::msg::BallPosition ball_data;

    void robot_callback(
      const oxebots_interfaces::msg::RobotPosition::SharedPtr msg);
    void ball_callback(
      const oxebots_interfaces::msg::BallPosition::SharedPtr msg);

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
