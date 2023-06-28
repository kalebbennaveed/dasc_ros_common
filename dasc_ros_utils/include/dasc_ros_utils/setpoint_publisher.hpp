#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"

#include "dasc_msgs/msg/di_trajectory.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
//#ifdef TF2_CPP_HEADERS
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
//#else
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#endif

#include "conversions.hpp"

class SetpointPublisher : public rclcpp::Node {

public:
  SetpointPublisher();

protected:
  // vars
  dasc_msgs::msg::DITrajectory traj_;
  bool traj_is_initialized = false;
 
  // timers
  rclcpp::TimerBase::SharedPtr timer_;

  // params
  std::string px4_robot_name_;
  std::string px4_world_frame_;
  std::string trajectory_topic_;

  // pubs
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_setpoint_viz_;

  // subs

  // for the tf message filter
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<dasc_msgs::msg::DITrajectory> traj_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<dasc_msgs::msg::DITrajectory>>
      tf2_filter_;

  // methods
  void
  trajectory_callback(const dasc_msgs::msg::DITrajectory::SharedPtr msg_ptr);

  void timer_callback();

};
