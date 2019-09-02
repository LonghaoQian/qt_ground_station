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

/* put additional include library here */
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
        {

        commandFlagUAV0 = false;
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

/*----------------------------------------------start ros node--------------------------------------------------*/
bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_ground_station");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
        // Add ros publiser hand subscribers here
        /*-------------------------------------pubs-----------------------------------------------*/
        moveUAV0 = n.advertise<qt_ground_station::ControlCommand>("/uav0/px4_command/control_command",100);
        moveUAV1 = n.advertise<qt_ground_station::ControlCommand>("/uav1/px4_command/control_command",100);
        mocapUAV0 = n.subscribe<qt_ground_station::Mocap>("/mocap/UAV0", 1000, &QNode::sub_mocapUAV0, this);

        /*-------------------------------------subs-----------------------------------------------*/
        UAV0_log_sub = n.subscribe<qt_ground_station::Topic_for_log>("/uav0/px4_command/topic_for_log", 100, &QNode::sub_topic_for_logUpdateUAV0, this);
        UAV0_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV0,this);


        /*---------------------------------------------------------------------------------------*/
	start();
	return true;
}

void QNode::run() {
        ros::Rate loop_rate(2);
        /*------------------------------------- ros loop ----------------------------------------*/
	while ( ros::ok() ) {
                pub_commandUAV0();
		ros::spinOnce();
		loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
/*---------------------------------------sub callbacks   ------------------------------------------*/
void QNode::sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg) {

   UAV0_mocap = *msg;
   Q_EMIT mocapUAV0_label();
}
void QNode::sub_mocapUAV1(const qt_ground_station::Mocap::ConstPtr& msg) {

   UAV1_mocap = *msg;
   Q_EMIT mocapUAV1_label();
}
void QNode::sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg) {

    UAV0_Topic_for_log = *msg;
    Q_EMIT UAV0_LogFromDrone_label();
}

void QNode::sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    UAV0_q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    UAV0_euler_fcu_target = quaternion_to_euler(UAV0_q_fcu_target);
    UAV0_Thrust_target = msg->thrust;
    Q_EMIT attReferenceUAV0_lable();
}
/*---------------------------------------pub functions -----------------------------------------*/
void QNode::pub_commandUAV0() {
    if (commandFlagUAV0) {
        moveUAV0.publish(Command_UAV0);
        commandFlagUAV0  = false;
    }
}
/*---------------------------------get values --------------------------------*/

qt_ground_station::Mocap QNode::GetMocapUAV0() {

    return UAV0_mocap;
}

Eigen::Vector4d QNode::GetAttThrustCommandUAV0() {
    Eigen::Vector4d command;
    command(0) = UAV0_euler_fcu_target(0);
    command(1) = UAV0_euler_fcu_target(1);
    command(2) = UAV0_euler_fcu_target(2);
    command(3) = (double) UAV0_Thrust_target;
    return command;

}

qt_ground_station::Topic_for_log QNode::GetDroneStateUAV0() {

    return UAV0_Topic_for_log;
}

/*---------------------------set values ----------------------------------------*/

void QNode::move_ENU_UAV0(float state_desired[4]) {
    commandFlagUAV0  = true;
    Command_UAV0.header.stamp = ros::Time::now();
    Command_UAV0.Mode = Move_ENU;
    generate_com(0, state_desired,Command_UAV0);
}
/*------------------------------------- Utility Functions --------------------------------------*/
void QNode::generate_com(int sub_mode,
                         float state_desired[4],
                         qt_ground_station::ControlCommand& Command_Now)
{
    static int comid = 1;
    Command_Now.Reference_State.Sub_mode  = sub_mode;

//# sub_mode 2-bit value:
//# 0 for position, 1 for vel, 1st for xy, 2nd for z.
//#                   xy position     xy velocity
//# z position       	0b00(0)       0b10(2)
//# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        Command_Now.Reference_State.position_ref[0] = state_desired[0];
        Command_Now.Reference_State.position_ref[1] = state_desired[1];
        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.velocity_ref[0] = state_desired[0];
        Command_Now.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_Now.Reference_State.position_ref[2] = state_desired[2];
        Command_Now.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[2] = 0;
        Command_Now.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;


    Command_Now.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
    Command_Now.Command_ID = comid;
    comid++;
}

void QNode::log( const LogLevel &level,
                 const std::string &msg)
{
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

}  // namespace qt_ground_station
