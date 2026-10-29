// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_observers/game_observer_node.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "ssl_league_msgs/msg/vision_wrapper.hpp"

class GameObserverTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      rclcpp::Parameter("is_yellow_team", true),
      rclcpp::Parameter("invert_sides", false),
      rclcpp::Parameter("min_ball_confidence", 0.3),
      rclcpp::Parameter("min_robot_confidence", 0.3),
      rclcpp::Parameter("robot_expiration_ms", 120.0),
      rclcpp::Parameter("ball_expiration_ms", 120.0),
      rclcpp::Parameter("multi_camera_window_ms", 50.0)
    });

    observer_node_ = std::make_shared<GameObserverNode>(opts);
    helper_node_ = std::make_shared<rclcpp::Node>("test_observer_helper");

    vision_pub_ = helper_node_->create_publisher<ssl_league_msgs::msg::VisionWrapper>(
      "/ssl_vision_bridge/vision_messages", 10);

    game_sub_ = helper_node_->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10,
      [this](const oxebots_interfaces::msg::GameData::SharedPtr msg) {
        received_msgs_.push_back(*msg);
      });

    executor_.add_node(observer_node_);
    executor_.add_node(helper_node_);
  }

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  static ssl_league_msgs::msg::VisionDetectionRobot makeRobot(
      uint32_t id, float x_m, float y_m, float conf) {
    ssl_league_msgs::msg::VisionDetectionRobot r;
    r.robot_id = id;
    r.pose.position.x = x_m;
    r.pose.position.y = y_m;
    r.pose.orientation.w = 1.0;
    r.confidence = conf;
    return r;
  }

  static ssl_league_msgs::msg::VisionDetectionBall makeBall(
      float x_m, float y_m, float conf) {
    ssl_league_msgs::msg::VisionDetectionBall b;
    b.pos.x = x_m;
    b.pos.y = y_m;
    b.confidence = conf;
    return b;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<GameObserverNode> observer_node_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
  std::vector<oxebots_interfaces::msg::GameData> received_msgs_;
};

TEST_F(GameObserverTest, PublishesWithoutBallAndSetsHeaderStamp) {
  ssl_league_msgs::msg::VisionWrapper wrapper;
  ssl_league_msgs::msg::VisionDetectionFrame frame;
  frame.camera_id = 0;
  frame.robots_yellow.push_back(makeRobot(0, -2.0f, 0.0f, 0.95f));
  // Sem bola no frame
  wrapper.detection.push_back(frame);

  vision_pub_->publish(wrapper);
  spinFor(std::chrono::milliseconds(50));

  ASSERT_FALSE(received_msgs_.empty());
  const auto & gd = received_msgs_.back();
  EXPECT_EQ(gd.header.frame_id, "map");
  EXPECT_GT(gd.header.stamp.sec + gd.header.stamp.nanosec, 0u);
  ASSERT_EQ(gd.robots.allies.size(), 1u);
  EXPECT_EQ(gd.robots.allies[0].id, 0u);
  EXPECT_NEAR(gd.robots.allies[0].x, -2000.0f, 1e-2f);
  EXPECT_FLOAT_EQ(gd.ball.confidence, 0.0f);
}

TEST_F(GameObserverTest, MultiCameraFusionPicksHighestConfidenceAndMergesRobots) {
  ssl_league_msgs::msg::VisionWrapper wrapper;

  // Câmera 0 vê robô aliado 1 com conf 0.6, robô aliado 2 com conf 0.9, e bola com conf 0.5
  ssl_league_msgs::msg::VisionDetectionFrame cam0;
  cam0.camera_id = 0;
  cam0.robots_yellow.push_back(makeRobot(1, -1.0f, 0.5f, 0.6f));
  cam0.robots_yellow.push_back(makeRobot(2, 0.5f, -0.5f, 0.9f));
  cam0.balls.push_back(makeBall(0.1f, 0.2f, 0.5f));

  // Câmera 1 vê robô aliado 1 com conf 0.95 (outra pose), inimigo 0, e bola com conf 0.92
  ssl_league_msgs::msg::VisionDetectionFrame cam1;
  cam1.camera_id = 1;
  cam1.robots_yellow.push_back(makeRobot(1, -1.2f, 0.6f, 0.95f));
  cam1.robots_blue.push_back(makeRobot(0, 1.8f, 0.0f, 0.88f));
  cam1.balls.push_back(makeBall(0.3f, 0.4f, 0.92f));

  wrapper.detection.push_back(cam0);
  wrapper.detection.push_back(cam1);

  vision_pub_->publish(wrapper);
  spinFor(std::chrono::milliseconds(50));

  ASSERT_FALSE(received_msgs_.empty());
  const auto & gd = received_msgs_.back();
  ASSERT_EQ(gd.robots.allies.size(), 2u);
  ASSERT_EQ(gd.robots.enemies.size(), 1u);

  for (const auto & r : gd.robots.allies) {
    if (r.id == 1u) {
      EXPECT_NEAR(r.x, -1200.0f, 1e-2f);
      EXPECT_NEAR(r.y, 600.0f, 1e-2f);
      EXPECT_NEAR(r.confidence, 0.95f, 1e-4f);
    } else if (r.id == 2u) {
      EXPECT_NEAR(r.x, 500.0f, 1e-2f);
      EXPECT_NEAR(r.y, -500.0f, 1e-2f);
    }
  }
  EXPECT_NEAR(gd.ball.x, 300.0f, 1e-2f);
  EXPECT_NEAR(gd.ball.y, 400.0f, 1e-2f);
  EXPECT_NEAR(gd.ball.confidence, 0.92f, 1e-4f);
}

TEST_F(GameObserverTest, ExpiresUnseenRobotsAfterTimeout) {
  // Frame 1: robôs 0 e 1 presentes
  ssl_league_msgs::msg::VisionWrapper wrapper1;
  ssl_league_msgs::msg::VisionDetectionFrame f1;
  f1.robots_yellow.push_back(makeRobot(0, -2.0f, 0.0f, 0.9f));
  f1.robots_yellow.push_back(makeRobot(1, -1.0f, 0.0f, 0.9f));
  wrapper1.detection.push_back(f1);

  vision_pub_->publish(wrapper1);
  spinFor(std::chrono::milliseconds(40));
  ASSERT_FALSE(received_msgs_.empty());
  EXPECT_EQ(received_msgs_.back().robots.allies.size(), 2u);

  // Espera além de robot_expiration_ms (120 ms) e envia Frame 2 apenas com robô 0
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  ssl_league_msgs::msg::VisionWrapper wrapper2;
  ssl_league_msgs::msg::VisionDetectionFrame f2;
  f2.robots_yellow.push_back(makeRobot(0, -2.0f, 0.0f, 0.9f));
  wrapper2.detection.push_back(f2);

  vision_pub_->publish(wrapper2);
  spinFor(std::chrono::milliseconds(40));

  const auto & gd = received_msgs_.back();
  ASSERT_EQ(gd.robots.allies.size(), 1u);
  EXPECT_EQ(gd.robots.allies[0].id, 0u);
}

TEST_F(GameObserverTest, SameCameraDecreasingConfidenceStillUpdatesPoseAndBall) {
  // Frame 1 da câmera 0 com alta confiança (0.98)
  ssl_league_msgs::msg::VisionWrapper w1;
  ssl_league_msgs::msg::VisionDetectionFrame f1;
  f1.camera_id = 0;
  f1.robots_yellow.push_back(makeRobot(1, -1.0f, 0.0f, 0.98f));
  f1.balls.push_back(makeBall(-0.5f, 0.0f, 0.98f));
  w1.detection.push_back(f1);

  vision_pub_->publish(w1);
  spinFor(std::chrono::milliseconds(15));

  // Frame 2 da MESMA câmera 0 dentro de multi_camera_window_ms (50ms no teste),
  // mas com confiança menor (0.85) pois o robô e a bola se afastaram do centro da câmera.
  // A posição DEVE atualizar normalmente e nunca congelar!
  ssl_league_msgs::msg::VisionWrapper w2;
  ssl_league_msgs::msg::VisionDetectionFrame f2;
  f2.camera_id = 0;
  f2.robots_yellow.push_back(makeRobot(1, 1.5f, 0.8f, 0.85f));
  f2.balls.push_back(makeBall(2.0f, 1.2f, 0.85f));
  w2.detection.push_back(f2);

  vision_pub_->publish(w2);
  spinFor(std::chrono::milliseconds(20));

  ASSERT_FALSE(received_msgs_.empty());
  const auto & gd = received_msgs_.back();
  ASSERT_EQ(gd.robots.allies.size(), 1u);
  EXPECT_NEAR(gd.robots.allies[0].x, 1500.0f, 1e-2f);
  EXPECT_NEAR(gd.robots.allies[0].y, 800.0f, 1e-2f);
  EXPECT_NEAR(gd.ball.x, 2000.0f, 1e-2f);
  EXPECT_NEAR(gd.ball.y, 1200.0f, 1e-2f);
}
