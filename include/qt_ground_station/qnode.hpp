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

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
        /*-------------------------------------------------------*/
        qt_ground_station::Mocap GetMocapUAV0();
        Eigen::Vector4d GetAttThrustCommandUAV0();
        qt_ground_station::Topic_for_log GetDroneStateUAV0();
Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();
        void mocapUAV0_label();
        void attReferenceUAV0_lable();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
        /*-------------------- Mocap ---------------------*/
        qt_ground_station::DroneState UAV0_state;
        qt_ground_station::Mocap UAV0_mocap;
        qt_ground_station::Mocap UAV1_mocap;
        qt_ground_station::Mocap UAV2_mocap;
        qt_ground_station::Mocap UAV3_mocap;

        ros::Subscriber mocapUAV0;
        ros::Subscriber mocapUAV1;
        ros::Subscriber mocapUAV2;
        ros::Subscriber mocapUAV3;
        ros::Subscriber mocapPayload;
        /*--------------------log_sub---------------------*/
        qt_ground_station::Topic_for_log UAV0_Topic_for_log;
        Eigen::Quaterniond               UAV0_q_fcu_target;
        Eigen::Vector3d                  UAV0_euler_fcu_target;
        float                            UAV0_Thrust_target;
        ros::Subscriber                  UAV0_log_sub;
        ros::Subscriber                  UAV0_attitude_target_sub;

        QStringListModel logging_model;
        void sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg);
        void sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg);
        void sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
};

}  // namespace qt_ground_station

#endif /* qt_ground_station_QNODE_HPP_ */
