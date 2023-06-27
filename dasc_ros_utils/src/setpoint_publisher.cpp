
#include "dasc_ros_utils/setpoint_publisher.hpp"

SetpointPublisher::SetpointPublisher() : Node("setpoint_publisher") {
  // declare parameters

  using std::placeholders::_1;
  using namespace std::chrono_literals;

  px4_robot_name_ =
      this->declare_parameter<std::string>("px4_robot_name", "px4_1");

  px4_world_frame_ =
      this->declare_parameter<std::string>("px4_local_frame", "vicon/world/NED");

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
  {
    std::string topic_name = px4_robot_name_ + "/fmu/in/trajectory_setpoint";
    pub_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_name, 10);
  }
  {
    std::string topic_name = px4_robot_name_ + "/trajectory_setpoint/viz";
    pub_setpoint_viz_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_name, 10);
  }
  
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

  // start by cancelling the timer, so it waits for a trajectory message and not consumes resources
  timer_->cancel();



  RCLCPP_INFO(this->get_logger(), "starting setpoint publisher node");
}


void SetpointPublisher::timer_callback() {
	RCLCPP_INFO(this->get_logger(), "timer callback");


	// if we got here, it means that there is a message for us to parse
	auto now = this->get_clock()->now();
	double tau = (now - traj_.header.stamp).seconds(); 

	// get the total length of the messages
	size_t N = traj_.poses.size(); // assumes they are all of the same length

	if (N == 0) {
RCLCPP_INFO(this-> get_logger(), "woops, no poses in this message");
		timer_-> cancel();
		return; 
	}

	int index = tau / traj_.dt; // integer division rounds down

	if (index < 0) {
RCLCPP_INFO(this-> get_logger(), "woops, message is for the future");
		return; // not yet time to process this message
	}
	if (index >= N) {
RCLCPP_INFO(this-> get_logger(), "woops, message is in the past!");
		timer_->cancel();
		return; // done parsing this message
	}

	// now we are in the right range, so we can interpolate and publish a message
	auto target_pose = traj_.poses[index];
	//auto target_twist = traj_.twists[index];
	//auto target_accels = traj_.accelerations[index];

	// make it into a trajectory setpoint
	px4_msgs::msg::TrajectorySetpoint msg;
	//msg.timestamp; // TODO(dev): figure out what needs to go here

	// since we assume the traj_ is in an NED frame, we dont need to do the conversion here
	for (size_t i=0; i<3; i++) {
		msg.position[i] = 0.0; // target_pose.position[i];
		msg.velocity[i] = 0.0; // TODO(dev): FIX! target_twist.linear[i];
		msg.acceleration[i] = 0.0; // TODO(dev): fix!!
		msg.jerk[i] = NAN;
	}
	msg.position[0] = target_pose.position.x;
	msg.position[1] = target_pose.position.y;
	msg.position[2] = target_pose.position.z;

	msg.yaw = 0.0;
	msg.yawspeed = 0.0;

	msg.raw_mode = false;


	// publish the trajectory setpoint and the vizualization of the setpoint pose
	pub_setpoint_->publish(msg);

	// publish the visualization
	geometry_msgs::msg::PoseStamped viz_msg;
	viz_msg.header.stamp = this->get_clock()->now();
	viz_msg.header.frame_id = px4_world_frame_;
	viz_msg.pose = target_pose;

	pub_setpoint_viz_->publish(viz_msg);


}


void SetpointPublisher::trajectory_callback(
    const dasc_msgs::msg::DITrajectory::SharedPtr msg) {


	geometry_msgs::msg::TransformStamped trans;

	try {
		trans = tf2_buffer_->lookupTransform(px4_world_frame_, msg->header.frame_id, msg->header.stamp);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
		return;
	}
	
	// since the transform was successful we can save the trajectory callback
	tf2::doTransform(*msg, traj_, trans);

	// restart the timer
	if (timer_ -> is_canceled()) {
		RCLCPP_INFO(this->get_logger(), "restarting timer");
		timer_ -> reset();
	}

}



int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointPublisher>());
  rclcpp::shutdown();
  return 0;
}
