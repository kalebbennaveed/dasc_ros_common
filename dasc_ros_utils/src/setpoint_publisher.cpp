
#include "dasc_ros_utils/setpoint_publisher.hpp"
//#include <tf2/utils.h>

// #include "tf2/convert.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// namespace tf2 {
//
//	using namespace geometry_msgs::msg;
//	using namespace dasc_msgs::msg;
//
//	// really annoying but galactic doesnt have this defined.
//	void doTransform(const Vector3& in, Vector3 &out, const TransformStamped
//&t) { 		Vector3Stamped inS, outS; 		inS.vector.x = in.x(); 		inS.vector.y = in.y();
//		inS.vector.z = in.z();
//		doTransform(inS, outS, t);
//		out.setX(0.0);
//	}
//
//
//	void doTransform(const Twist& in, Twist &out, const TransformStamped &
//t) { 		doTransform(in.linear, out.linear, t); 		doTransform(in.angular,
//out.angular, t);
//	}
//
//	void doTransform(const Accel & in, Accel &out, const TransformStamped &
//t) { 		doTransform(in.linear, out.linear, t); 		doTransform(in.angular,
//out.angular, t);
//	}
//
//
//	void doTransform(const DITrajectory& in, DITrajectory &out, const
//TransformStamped & t) {
//	  // update the header
//	  out.header.stamp = t.header.stamp;
//	  out.header.frame_id = t.child_frame_id;
//	  out.dt = in.dt;
//
//	  // now do the transform of each pose
//	  out.poses.clear();
//	  for (auto p:in.poses) {
//	  	geometry_msgs::msg::Pose pp;
//	  	tf2::doTransform(p, pp, t);
//	  	out.poses.push_back(pp);
//	  }
//	  // repeat for the twists
//	  out.twists.clear();
//	  for (auto p:in.twists) {
//	  	geometry_msgs::msg::Twist pp;
//	  	tf2::doTransform(p, pp, t);
//	  	out.twists.push_back(pp);
//	  }
//	  // repeat for the accels
//	  out.accelerations.clear();
//	  for (auto p:in.accelerations) {
//	  	geometry_msgs::msg::Accel pp;
//	  	tf2::doTransform(p, pp, t);
//	  	out.accelerations.push_back(pp);
//	    }
//	}
//
//}
//
//
//
//
//

SetpointPublisher::SetpointPublisher() : Node("setpoint_publisher") {
  // declare parameters

  using std::placeholders::_1;
  using namespace std::chrono_literals;

  px4_robot_name_ =
      this->declare_parameter<std::string>("px4_robot_name", "px4_1");

  px4_world_frame_ = this->declare_parameter<std::string>("px4_local_frame",
                                                          "vicon/world/NED");

  trajectory_topic_ =
      this->declare_parameter<std::string>("trajectory_topic", "trajectory");

  // construct the tf buffer
  std::chrono::duration<int> buffer_timeout(1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create a publisher of the px4 setpoint
  pub_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4_robot_name_ + "/fmu/in/trajectory_setpoint", 10);
  pub_setpoint_viz_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      px4_robot_name_ + "/fmu/in/trajectory_setpoint/viz", 10);

  // subscribe to dasc_msgs/DITrajectory
  traj_sub_.subscribe(this, trajectory_topic_);
  tf2_filter_ =
      std::make_shared<tf2_ros::MessageFilter<dasc_msgs::msg::DITrajectory>>(
          traj_sub_, *tf2_buffer_, px4_world_frame_, 100,
          this->get_node_logging_interface(), this->get_node_clock_interface(),
          buffer_timeout);
  tf2_filter_->registerCallback(&SetpointPublisher::trajectory_callback, this);

  // initialize the timer
  timer_ = this->create_wall_timer(
      50ms, std::bind(&SetpointPublisher::timer_callback, this));

  // start by cancelling the timer, so it waits for a trajectory message and not
  // consumes resources
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "starting setpoint publisher node");
}

void SetpointPublisher::timer_callback() {

  // if we got here, it means that there is a message for us to parse
  auto now = this->get_clock()->now();
  double tau = (now - traj_.header.stamp).seconds();

  // get the total length of the messages
  size_t N_poses = traj_.poses.size();
  size_t N_twists = traj_.twists.size();
  size_t N_accels = traj_.accelerations.size();

  size_t N = std::max(std::max(N_poses, N_twists), N_accels);

  if (N == 0) {
    RCLCPP_INFO(this->get_logger(), "woops, no poses in this message");
    timer_->cancel();
    return;
  }

  int index = tau / traj_.dt; // integer division rounds down

  if (index < 0) {
    RCLCPP_INFO(this->get_logger(), "woops, message is for the future");
    return; // not yet time to process this message
  }
  if (index >= int(N)) {
    RCLCPP_INFO(this->get_logger(), "woops, message is in the past!");
    timer_->cancel();
    return; // done parsing this message
  }

  // initialize the message
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.raw_mode = false;
  // msg.timestamp; // TODO(dev): figure out what needs to go here

  // insert target_pose
  if (index < int(N_poses)) {
    auto target_pose = traj_.poses[index];
    msg.position[0] = target_pose.position.x;
    msg.position[1] = target_pose.position.y;
    msg.position[2] = target_pose.position.z;

    msg.yaw = tf2::getYaw(target_pose.orientation);

    // publish the visualization
    geometry_msgs::msg::PoseStamped viz_msg;
    viz_msg.header.stamp = this->get_clock()->now();
    viz_msg.header.frame_id = px4_world_frame_;
    viz_msg.pose = target_pose;
    pub_setpoint_viz_->publish(viz_msg);

  } else {
    for (size_t i = 0; i < 3; i++) {
      msg.position[i] = NAN;
    }
  }

  // insert target twists
  if (index < int(N_twists)) {
    auto target = traj_.twists[index];
    msg.velocity[0] = target.linear.x;
    msg.velocity[1] = target.linear.y;
    msg.velocity[2] = target.linear.z;

    msg.yawspeed = 0.0; // TODO(dev): update this!
  } else {
    for (size_t i = 0; i < 3; i++) {
      msg.velocity[i] = NAN;
    }
  }

  // insert target accels
  if (index < int(N_accels)) {
    auto target = traj_.accelerations[index];
    msg.acceleration[0] = target.linear.x;
    msg.acceleration[1] = target.linear.y;
    msg.acceleration[2] = target.linear.z;
  } else {
    for (size_t i = 0; i < 3; i++) {
      msg.acceleration[i] = NAN;
    }
  }

  // insert target jerks
  for (size_t i = 0; i < 3; i++) {
    msg.jerk[i] = NAN;
  }

  // publish the trajectory setpoint and the vizualization of the setpoint pose
  pub_setpoint_->publish(msg);

}

void SetpointPublisher::trajectory_callback(
    const dasc_msgs::msg::DITrajectory::SharedPtr msg) {

  geometry_msgs::msg::TransformStamped trans;

  try {
    trans = tf2_buffer_->lookupTransform(px4_world_frame_, msg->header.frame_id,
                                         msg->header.stamp);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    return;
  }

  // since the transform was successful we can save the trajectory callback
  tf2::doTransform(*msg, traj_, trans);

  // restart the timer
  if (timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "restarting timer");
    timer_->reset();
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointPublisher>());
  rclcpp::shutdown();
  return 0;
}
