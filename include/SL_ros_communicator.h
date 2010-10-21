/*!=============================================================================
  ==============================================================================

  \ingroup SLros

  \file    SL_ros_communication.c

  \author  Peter Pastor
  \date    July 2010

  ==============================================================================
  \remarks

      this file contains the functionality to publish SL variables to ROS

  ============================================================================*/
#ifndef SL_ROS_PUBLISHER_H_
#define SL_ROS_PUBLISHER_H_

// ROS includes
#include <ros/ros.h>

// ROS msgs includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace sl2ros
{

class SL_ros_communicator
{
		
public:

	SL_ros_communicator();
	virtual ~SL_ros_communicator();
	
	bool initialize();
	bool publish();

private:

	ros::NodeHandle *node_handle_;
	ros::Publisher nrt_joint_state_publisher_;
	ros::Publisher nrt_endeffector_publisher_;

};

}

#endif /* SL_ROS_PUBLISHER_H_ */
