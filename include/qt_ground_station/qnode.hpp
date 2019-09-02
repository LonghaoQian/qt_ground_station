/**
 * @file /include/qt_ground_station/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_ground_station_QNODE_HPP_
#define qt_ground_station_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#pragma once
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <rosbag/bag.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <qt_ground_station/Mocap.h>
#include <qt_ground_station/Topic_for_log.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Eigen>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_ground_station {

/*****************************************************************************
** Class
*****************************************************************************/

enum LogLevel {
         Debug,
         Info,
         Warn,
         Error,
         Fatal
 };
enum Command_Type
{
    Idle,
    Takeoff,
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    PPN_land,
    Trajectory_Tracking,
};

struct uav_log {
    bool isconnected;
    bool islogreceived;
    qt_ground_station::Topic_for_log log;
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d    euler_fcu_target;
    float              Thrust_target;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();


	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
/*----------------------------Get states---------------------------------*/
        qt_ground_station::Mocap GetMocap(int ID);
        qt_ground_station::uav_log GetUAVLOG(int ID);
/*----------------------------Send commands------------------------------*/
        void move_ENU(int ID,float state_desired[4]);
        void takeoff(int ID);
        void land(int ID);
        void disarm(int ID);
Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();

        void mocapUAV0_label();
        void mocapUAV1_label();
        void mocapUAV2_label();

        void attReferenceUAV0_lable();
        void attReferenceUAV1_lable();
        void attReferenceUAV2_lable();

        void UAV0_LogFromDrone_label();
        void UAV1_LogFromDrone_label();
        void UAV2_LogFromDrone_label();
private:
        /*-------------------input arguments-----------------------*/
        int init_argc;
	char** init_argv;
        /*-----------------------------------------------------------*/
        int DroneNumber;
        uav_log UavLogList[3];
        bool commandFlag[3];
        qt_ground_station::ControlCommand Command_List[3];
        qt_ground_station::Mocap mocap[3];
        qt_ground_station::Mocap mocap_payload;
        /*------------------- motion pubs   ------------------------*/
        ros::Publisher moveUAV0;
        ros::Publisher moveUAV1;
        ros::Publisher moveUAV2;
        void pub_command();
        /*-------------------- Mocap from motive ---------------------*/
        ros::Subscriber mocapUAV0;
        ros::Subscriber mocapUAV1;
        ros::Subscriber mocapUAV2;

        ros::Subscriber mocapPayload;

        ros::Subscriber UAV0_log_sub;
        ros::Subscriber UAV1_log_sub;
        ros::Subscriber UAV2_log_sub;

        ros::Subscriber UAV0_attitude_target_sub;
        ros::Subscriber UAV1_attitude_target_sub;
        ros::Subscriber UAV2_attitude_target_sub;

        void sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg);
        void sub_mocapUAV1(const qt_ground_station::Mocap::ConstPtr& msg);
        void sub_mocapUAV2(const qt_ground_station::Mocap::ConstPtr& msg);
        void sub_mocapPayload(const qt_ground_station::Mocap::ConstPtr& msg);
        /*-------------------- UAV log subs ---------------------*/

        void sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg);
        void sub_topic_for_logUpdateUAV1(const qt_ground_station::Topic_for_log::ConstPtr &msg);
        void sub_topic_for_logUpdateUAV2(const qt_ground_station::Topic_for_log::ConstPtr &msg);

        void sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
        void sub_setpoint_rawUpdateUAV1(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
        void sub_setpoint_rawUpdateUAV2(const mavros_msgs::AttitudeTarget::ConstPtr& msg);

        QStringListModel logging_model;

        /*---------------------------utility functions ---------------------------*/
        void generate_com(int sub_mode, float state_desired[4],qt_ground_station::ControlCommand& Command_Now);

};

}  // namespace qt_ground_station

#endif /* qt_ground_station_QNODE_HPP_ */
