#include <string>

// include ROS 1
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "measurement_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "measurement_msgs/msg/range.hpp"


#include "yaml-cpp/yaml.h"

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    printf("No config file provided.\n");
    return 0;
  } 
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  printf("ROS1 Static bridge.\n");
  YAML::Node config = YAML::LoadFile(argv[1]);
  std::map<std::string, ros1_bridge::BridgeHandles> handle_map;

  size_t queue_size = 10;
  for(YAML::const_iterator it=config.begin(); it != config.end(); ++it)
  {
    std::string topic_name = it->first.as<std::string>();
    if(handle_map.find(topic_name) != handle_map.end())
    {
        printf("Static bridge:[%s] already bridged.\n",
            topic_name.c_str());
        continue;
    }
    std::string ros1_type_name = it->second[0].as<std::string>();
    std::string ros2_type_name = it->second[1].as<std::string>();
    handle_map[topic_name] = ros1_bridge::create_bidirectional_bridge(
      ros1_node, ros2_node, ros1_type_name, ros2_type_name, topic_name, queue_size);
    printf("Static bridge: Bridging:[%s]. types:(ROS1)[%s]<->(ROS2)[%s].\n",
        topic_name.c_str(), ros1_type_name.c_str(), ros2_type_name.c_str());
  }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
