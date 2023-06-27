
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "dasc_msgs/msg/di_trajectory.hpp"

#ifdef TF2_CPP_HEADERS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif


namespace tf2 {

	using namespace geometry_msgs::msg;
	using namespace dasc_msgs::msg;

	void doTransform(const Twist& in, Twist &out, const TransformStamped & t) {
		doTransform(in.linear, out.linear, t);
		doTransform(in.angular, out.angular, t);
	}
	
	void doTransform(const Accel & in, Accel &out, const TransformStamped & t) {
		doTransform(in.linear, out.linear, t);
		doTransform(in.angular, out.angular, t);
	}


	void doTransform(const DITrajectory& in, DITrajectory &out, const TransformStamped & t) { 
	  // update the header
	  out.header.stamp = t.header.stamp;
	  out.header.frame_id = t.child_frame_id;
	  out.dt = in.dt;

	  // now do the transform of each pose
	  out.poses.clear();
	  for (auto p:in.poses) {
	  	geometry_msgs::msg::Pose pp;
	  	tf2::doTransform(p, pp, t);
	  	out.poses.push_back(pp);
	  }
	  // repeat for the twists
	  out.twists.clear();
	  for (auto p:in.twists) {
	  	geometry_msgs::msg::Twist pp;
	  	tf2::doTransform(p, pp, t);
	  	out.twists.push_back(pp);
	  }
	  // repeat for the accels
	  out.accelerations.clear();
	  for (auto p:in.accelerations) {
	  	geometry_msgs::msg::Accel pp;
	  	tf2::doTransform(p, pp, t);
	  	out.accelerations.push_back(pp);
	    }
	}

}
