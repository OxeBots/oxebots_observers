#include "oxebots_observers/game_observer_node.hpp"

GameObserverNode::GameObserverNode() : Node("game_observer_node")
{
    RCLCPP_DEBUG(get_logger(), "Game Observer Node starting...");
    declare_parameter("robot_amount", 3);
    declare_parameter("subscriber_robot_topic", "robot_data");
    declare_parameter("subscriber_ball_topic", "ball_data");
    declare_parameter("publisher_topic", "game_data");
    declare_parameter("topic_retention", 10);

    RCLCPP_DEBUG(get_logger(), "Creating the publisher...");

    game_publisher = create_publisher<oxebots_interfaces::msg::GameData>(
      get_parameter("publisher_topic").as_string(),
      get_parameter("topic_retention").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating the subscribers...");

    robot_subscriber =
      create_subscription<oxebots_interfaces::msg::RobotPosition>(
        get_parameter("subscriber_robot_topic").as_string(), 10,
        std::bind(&GameObserverNode::robot_callback, this,
                  std::placeholders::_1));

    ball_subscriber =
      create_subscription<oxebots_interfaces::msg::BallPosition>(
        get_parameter("subscriber_ball_topic").as_string(), 10,
        std::bind(&GameObserverNode::ball_callback, this,
                  std::placeholders::_1));

    RCLCPP_DEBUG(get_logger(), "Initializing the structures...");
    team_size = get_parameter("team_size").as_int();
    allies.reserve(team_size);
    enemies.reserve(team_size);

    RCLCPP_INFO(get_logger(), "Game Observer Node Started");
}

GameObserverNode::~GameObserverNode()
{
    RCLCPP_INFO(this->get_logger(), "Game Observer Node Stopped");
}

void GameObserverNode::robot_callback(
  const oxebots_interfaces::msg::RobotPosition::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "Robot data received");
    int index = 0;
    for (auto ally : msg.get()->allies)
    {
        index = contains_robot(allies, ally);
        if (index == -1)
        {
            allies.push_back(ally);
            allies_count++;
        }
        else
            allies[index] = ally;
    }
    for (auto enemy : msg.get()->enemies)
    {
        index = contains_robot(enemies, enemy);
        if (index == -1)
        {
            enemies.push_back(enemy);
            enemies_count++;
        }
        else
            enemies[index] = enemy;
    }

    validate_data();
}

void GameObserverNode::ball_callback(
  const oxebots_interfaces::msg::BallPosition::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "Ball data received");
    ball_data = *msg.get();
    is_ball_present = true;
    validate_data();
}

void GameObserverNode::validate_data()
{
    if (is_ball_present && allies_count == team_size &&
        enemies_count == team_size)
    {
        publish_game_data();
        is_ball_present = false;
        allies_count = 0;
        enemies_count = 0;
    }
    else
        RCLCPP_DEBUG(get_logger(), "Data not complete yet...");
}

void GameObserverNode::publish_game_data()
{
    RCLCPP_DEBUG(get_logger(), "Publishing game data...");
    oxebots_interfaces::msg::GameData game_data;
    game_data.robots.allies = allies;
    game_data.robots.enemies = enemies;
    game_data.ball = ball_data;
    game_publisher->publish(game_data);
}

int GameObserverNode::contains_robot(
  const std::vector<oxebots_interfaces::msg::RobotGameData> & robots,
  const oxebots_interfaces::msg::RobotGameData & robot)
{
    auto it = std::find(robots.begin(), robots.end(), robot);
    if (it != robots.end())
        return std::distance(robots.begin(), it);
    else
        return -1;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameObserverNode>());
    rclcpp::shutdown();
    return 0;
}
