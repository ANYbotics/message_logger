/*!
 * @file     LoggerManagerTests.cpp
 * @brief    Handing MELO_* logging over to a node (anybotics#26407).
 */

#include <gtest/gtest.h>

#include <message_logger/message_logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

TEST(AttachToNode, adoptsTheNodeLoggerSoRecordsCanReachRosout) {
  rclcpp::init(0, nullptr);
  const auto node = rclcpp::Node::make_shared("attach_to_node_test");

  message_logger::log::attachToNode(node);

  // The name is the whole mechanism: rcl resolves a record's /rosout publisher by logger name, and
  // only names a node registered resolve.
  EXPECT_TRUE(message_logger::log::LoggerManager::isAttached());
  EXPECT_STREQ(node->get_logger().get_name(), message_logger::log::LoggerManager::getLogger().get_name());

  rclcpp::shutdown();
}

TEST(AttachToNode, adoptsTheNodeClockSoThrottlingFollowsRosTime) {
  rclcpp::init(0, nullptr);
  const auto node = rclcpp::Node::make_shared("attach_to_node_clock_test",
                                              rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));

  message_logger::log::attachToNode(node);

  // A clock built without a node reports system time regardless of use_sim_time, which would make
  // MELO_*_THROTTLE misbehave in simulation. Only the node's clock has ROS time active.
  EXPECT_TRUE(message_logger::log::LoggerClockManager::getLoggerClock().ros_time_is_active());

  rclcpp::shutdown();
}

}  // namespace
