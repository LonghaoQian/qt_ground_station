/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qt_ground_station/qnode.hpp"
#include "../include/qt_ground_station/math_utils.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_ground_station {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_ground_station");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.       
        moveUAV0 = n.advertise<qt_ground_station::ControlCommand>("/px4_command/control_command", 10);
        mocapUAV0 = n.subscribe<qt_ground_station::Mocap>("/mocap/UAV0", 1000, &QNode::sub_mocapUAV0, this);
        UAV0_log_sub = n.subscribe<qt_ground_station::Topic_for_log>("/px4_command/topic_for_log", 100, &QNode::sub_topic_for_logUpdateUAV0, this);
        UAV0_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV0,this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
                //chatter_publisher.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg) {

   UAV0_mocap = *msg;
   Q_EMIT mocapUAV0_label();
}

qt_ground_station::Mocap QNode::GetMocapUAV0() {

    return UAV0_mocap;
}

void QNode::sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg) {

    UAV0_Topic_for_log = *msg;
}

void QNode::sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    UAV0_q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    UAV0_euler_fcu_target = quaternion_to_euler(UAV0_q_fcu_target);
    UAV0_Thrust_target = msg->thrust;
    Q_EMIT attReferenceUAV0_lable();
}

Eigen::Vector4d QNode::GetAttThrustCommandUAV0() {
    Eigen::Vector4d command;
    command(0) = UAV0_euler_fcu_target(0);
    command(1) = UAV0_euler_fcu_target(1);
    command(2) = UAV0_euler_fcu_target(2);
    command(3) = (double) UAV0_Thrust_target;
    return command;

}

}  // namespace qt_ground_station
