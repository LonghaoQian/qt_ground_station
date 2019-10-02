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

        commandFlag[0] = false;
        commandFlag[1] = false;
        commandFlag[2] = false;
        commandPayloadFlag = false;
        DroneNumber = 3;
        ispayloaddetected = false;
        ispayloadmocaprecieved = false;
        ispayloadcontrolactivated = false;
        comid = 1;
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
        moveUAV2 = n.advertise<qt_ground_station::ControlCommand>("/uav2/px4_command/control_command",100);

        mocapUAV0 = n.subscribe<qt_ground_station::Mocap>("/mocap/UAV0", 1000, &QNode::sub_mocapUAV0, this);
        mocapUAV1 = n.subscribe<qt_ground_station::Mocap>("/mocap/UAV1", 1000, &QNode::sub_mocapUAV1, this);
        mocapUAV2 = n.subscribe<qt_ground_station::Mocap>("/mocap/UAV2", 1000, &QNode::sub_mocapUAV2, this);
        mocapPayload = n.subscribe<qt_ground_station::Mocap>("/mocap/Payload", 1000, &QNode::sub_mocapPayload, this);
        /*-------------------------------------subs-----------------------------------------------*/
        UAV0_log_sub = n.subscribe<qt_ground_station::Topic_for_log>("/uav0/px4_command/topic_for_log", 100, &QNode::sub_topic_for_logUpdateUAV0, this);
        UAV1_log_sub = n.subscribe<qt_ground_station::Topic_for_log>("/uav1/px4_command/topic_for_log", 100, &QNode::sub_topic_for_logUpdateUAV1, this);
        UAV2_log_sub = n.subscribe<qt_ground_station::Topic_for_log>("/uav2/px4_command/topic_for_log", 100, &QNode::sub_topic_for_logUpdateUAV2, this);

        UAV0_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV0,this);
        UAV1_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV1,this);
        UAV2_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV2,this);


        /*---------------------------------------------------------------------------------------*/
	start();
	return true;
}

void QNode::run() {
        ros::Rate loop_rate(4);
        /*------------------------------------- ros loop ----------------------------------------*/
	while ( ros::ok() ) {
                pub_command();
		ros::spinOnce();
       /*---------------------emit signals to tigger label update --------------------------*/

                for (int i = 0; i <DroneNumber; i++)
                {
                    if(UavLogList[i].islogreceived)
                    {
                        UavLogList[i].isconnected =true;
                    } else{
                        UavLogList[i].isconnected = false;
                    }
                    UavLogList[i].islogreceived = false;
                }

        /*----------------detect whether payload mocap is published --------------------------*/
                if(ispayloadmocaprecieved) {
                    ispayloaddetected = true;
                } else {
                    ispayloaddetected = false;
                }
                ispayloadmocaprecieved = false;

                /* signal a ros loop update  */
                Q_EMIT rosLoopUpdate();

		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
/*---------------------------------------sub callbacks   ------------------------------------------*/
void QNode::sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg) {
   mocap[0] = *msg;
}
void QNode::sub_mocapUAV1(const qt_ground_station::Mocap::ConstPtr& msg) {
   mocap[1] = *msg;
}
void QNode::sub_mocapUAV2(const qt_ground_station::Mocap::ConstPtr& msg) {
   mocap[2] = *msg;
}
void QNode::sub_mocapPayload(const qt_ground_station::Mocap::ConstPtr &msg) {
    mocap_payload = *msg;
    ispayloadmocaprecieved = true;
}
void QNode::sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg) {

    UavLogList[0].log = *msg;
    UavLogList[0].islogreceived = true;
}
void QNode::sub_topic_for_logUpdateUAV1(const qt_ground_station::Topic_for_log::ConstPtr &msg) {

    UavLogList[1].log = *msg;
    UavLogList[1].islogreceived = true;
}
void QNode::sub_topic_for_logUpdateUAV2(const qt_ground_station::Topic_for_log::ConstPtr &msg) {

    UavLogList[2].log = *msg;
    UavLogList[2].islogreceived = true;
}

void QNode::sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    UavLogList[0].q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    UavLogList[0].euler_fcu_target = quaternion_to_euler(UavLogList[0].q_fcu_target);
    UavLogList[0].Thrust_target= msg->thrust;

}
void QNode::sub_setpoint_rawUpdateUAV1(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    UavLogList[1].q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    UavLogList[1].euler_fcu_target = quaternion_to_euler(UavLogList[1].q_fcu_target);
    UavLogList[1].Thrust_target= msg->thrust;

}
void QNode::sub_setpoint_rawUpdateUAV2(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    UavLogList[2].q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    UavLogList[2].euler_fcu_target = quaternion_to_euler(UavLogList[2].q_fcu_target);
    UavLogList[2].Thrust_target= msg->thrust;

}
/*---------------------------------------pub functions -----------------------------------------*/
void QNode::pub_command() {

    if (commandFlag[0]) {
        moveUAV0.publish(Command_List[0]);
        commandFlag[0] = false;
    }
    if (commandFlag[1]) {
        moveUAV1.publish(Command_List[1]);
        commandFlag[1] = false;
    }
    if (commandFlag[2]) {
        moveUAV2.publish(Command_List[2]);
        commandFlag[2] = false;
    }
}
/*---------------------------------get values --------------------------------*/

qt_ground_station::Mocap QNode::GetMocap(int ID) {

    if( ID == -1) {
        return mocap_payload;
    } else {
        return mocap[ID];
    };
}

qt_ground_station::uav_log QNode::GetUAVLOG(int ID) {
    return UavLogList[ID];
}

bool QNode::IsPayloadDetected() {
    return ispayloaddetected;
}

bool QNode::IsPayloadControlSwitched() {
    return ispayloadcontrolactivated;
}
/*--------------------------- sent commands ----------------------------------------*/
void QNode::takeoff(int ID) {
    commandFlag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Takeoff;
}

void QNode::move_ENU(int ID,
                    float state_desired[4]) {

    commandFlag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Move_ENU;
    generate_com(0, state_desired,Command_List[ID]);

}
void QNode::land(int ID) {
    commandFlag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Land;
}

void QNode::disarm(int ID){
    commandFlag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Disarm;
}

void QNode::payload_pose(float pose_desired[6]) {
    // set mode to payload_stabilization to all quadrotors
    for(int i = 0;i<DroneNumber; i++) {
        commandFlag[i] = true;
        Command_List[i].header.stamp = ros::Time::now();
        Command_List[i].Mode = Payload_Stabilization;
        generate_com_for_payload(pose_desired, Command_List[i]);
    }
    ispayloadcontrolactivated = true;
}

void QNode::payload_land() {
    // set mode to payload_stabilization to all quadrotors
    for(int i = 0;i<DroneNumber; i++) {
        commandFlag[i] = true;
        Command_List[i].header.stamp = ros::Time::now();
        Command_List[i].Mode = Payload_Land;
    }
    ispayloadcontrolactivated = true;
}

/*------------------------------------- Utility Functions --------------------------------------*/
void QNode::generate_com(int sub_mode,
                         float state_desired[4],
                         qt_ground_station::ControlCommand& Command_Now)
{
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

void QNode::generate_com_for_payload(float state_desired[6],qt_ground_station::ControlCommand& Command_Now)
{


    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Reference_State.Sub_mode  = -1;
    for(int i =0; i<3; i++) {
        Command_Now.Reference_State.position_ref[i] = state_desired[i];
    }

    Command_Now.Reference_State.roll_ref  = state_desired[3]/57.3;
    Command_Now.Reference_State.pitch_ref = state_desired[4]/57.3;
    Command_Now.Reference_State.yaw_ref   = state_desired[5]/57.3;

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
