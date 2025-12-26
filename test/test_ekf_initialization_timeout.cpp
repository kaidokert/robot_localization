/*
 * Test that verifies EKF and NavSat initialization don't block with use_sim_time.
 *
 * This replicates the exact deadlock scenario from ekf_node.cpp and
 * navsat_transform_node.cpp main():
 * 1. Create node with use_sim_time:=true
 * 2. Call initialize() or constructor - these should NOT block
 * 3. If they block for >5 seconds, the test fails
 *
 * With the bug: initialize() calls wait_until_started() which blocks forever
 * With the fix: initialize() returns immediately
 */

#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ros_filter_types.hpp"
#include "robot_localization/navsat_transform.hpp"

TEST(EkfInitializationTest, DoesNotBlockWithSimTime)
{
  rclcpp::init(0, nullptr);

  // Configure node with use_sim_time:=true (deadlock trigger)
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node", "--ros-args", "-p", "use_sim_time:=true"});
  options.clock_type(RCL_ROS_TIME);

  // Create node and call initialize() in a separate thread
  // so we can detect if it blocks
  std::promise<bool> init_completed;
  std::future<bool> result = init_completed.get_future();

  std::thread init_thread([&options, &init_completed]() {
    try {
      // This is exactly what ekf_node.cpp main() does:
      auto filter = std::make_shared<robot_localization::RosEkf>(options);
      filter->initialize();  // Should NOT block
      init_completed.set_value(true);
    } catch (const std::exception & e) {
      ADD_FAILURE() << "Exception during initialization: " << e.what();
      init_completed.set_value(false);
    }
  });

  // Wait up to 5 seconds for initialization to complete
  auto status = result.wait_for(std::chrono::seconds(5));

  if (status == std::future_status::timeout) {
    // Initialization is blocked - this is the deadlock!
    init_thread.detach();  // Can't join - it's stuck
    rclcpp::shutdown();
    FAIL() << "EKF initialize() blocked for >5 seconds with use_sim_time:=true. "
           << "This indicates the wait_until_started() deadlock bug.";
  } else {
    // Initialization completed
    EXPECT_TRUE(result.get()) << "Initialization failed with exception";
    init_thread.join();
    rclcpp::shutdown();
  }
}

TEST(NavSatInitializationTest, DoesNotBlockWithSimTime)
{
  rclcpp::init(0, nullptr);

  // Configure node with use_sim_time:=true (deadlock trigger)
  rclcpp::NodeOptions options;
  options.arguments({"navsat_transform_node", "--ros-args", "-p", "use_sim_time:=true"});
  options.clock_type(RCL_ROS_TIME);

  // Create node in a separate thread so we can detect if it blocks
  std::promise<bool> init_completed;
  std::future<bool> result = init_completed.get_future();

  std::thread init_thread([&options, &init_completed]() {
    try {
      // This is exactly what navsat_transform_node.cpp main() does:
      auto navsat_node = std::make_shared<robot_localization::NavSatTransform>(options);
      // Constructor should NOT block
      init_completed.set_value(true);
    } catch (const std::exception & e) {
      ADD_FAILURE() << "Exception during initialization: " << e.what();
      init_completed.set_value(false);
    }
  });

  // Wait up to 5 seconds for initialization to complete
  auto status = result.wait_for(std::chrono::seconds(5));

  if (status == std::future_status::timeout) {
    // Initialization is blocked - this is the deadlock!
    init_thread.detach();  // Can't join - it's stuck
    rclcpp::shutdown();
    FAIL() << "NavSatTransform constructor blocked for >5 seconds with use_sim_time:=true. "
           << "This indicates the wait_until_started() deadlock bug.";
  } else {
    // Initialization completed
    EXPECT_TRUE(result.get()) << "Initialization failed with exception";
    init_thread.join();
    rclcpp::shutdown();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
