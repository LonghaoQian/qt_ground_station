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

Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();
        void mocapUAV0_label();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
        /*-------------------- Mocap ---------------------*/

        ros::Subscriber mocapUAV0;
        ros::Subscriber mocapUAV1;
        ros::Subscriber mocapUAV2;
        ros::Subscriber mocapUAV3;
        ros::Subscriber mocapPayload;
        /*--------------------log_sub---------------------*/
        ros::Subscriber UAV0_log_sub;// = nh.subscribe<px4_command::Topic_for_log>("/px4_command/topic_for_log", 100, log_cb)
        ros::Subscriber UAV0_attitude_target_sub;//  = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 100,att_target_cb);
        QStringListModel logging_model;
        void sub_mocapUAV0(const qt_ground_station::Mocap::ConstPtr& msg);
        void sub_topic_for_logUpdateUAV0(const qt_ground_station::Topic_for_log::ConstPtr &msg);
        void sub_setpoint_rawUpdateUAV0(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
};

}  // namespace qt_ground_station

#endif /* qt_ground_station_QNODE_HPP_ */
