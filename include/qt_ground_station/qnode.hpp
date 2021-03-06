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
#include <functional>
#include <QStringListModel>
// custom msg
#include <qt_ground_station/Mocap.h>
#include <qt_ground_station/ControlParameter.h>
#include <qt_ground_station/Topic_for_log.h>
#include <qt_ground_station/ControlCommand.h>
#include <qt_ground_station/DroneState.h>
#include <qt_ground_station/HomePosition.h>
// custom srv
#include <qt_ground_station/SetHome.h>
#include <qt_ground_station/SinglePayloadAction.h>
#include <qt_ground_station/MultiPayloadAction.h>
#include <qt_ground_station/GeneralInfo.h>
// ros class and msgs
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Eigen>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_ground_station {
// definition of command errors


enum vector_component{
    Vector_X = 0,
    Vector_Y = 1,
    Vector_Z = 2
};
enum quaternion_component{
    QUAT_W = 0,
    QUAT_X = 1,
    QUAT_Y = 2,
    QUAT_Z = 3
};

enum euler_component{
    EULER_ROLL = 0,
    EULER_PITCH,
    EULER_YAW
};

enum UAVindex{
        DRONE_UAV0 = 0,
        DRONE_UAV1,
        DRONE_UAV2
};

enum ENUCommandError{
        DRONE_COMMAND_NORM = 0,
        DRONE_COMMAND_TOOCLOSETOOTHER,
        DRONE_COMMAND_TOOCLOSETOOTHERCOMMAND,
        DRONE_COMMAND_OUTOFBOUND,
};

enum LogLevel {
         Debug,
         Info,
         Warn,
         Error,
         Fatal
 };
// commmand type defined by px4_command
enum Command_Type
{
    Idle,
    Takeoff,
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    Payload_Stabilization_SingleUAV,
    Trajectory_Tracking,
    Payload_Stabilization,
    Payload_Land,
};
// logging info from each drone
struct uav_log {
    bool isconnected{false};
    bool islogreceived{false};
    qt_ground_station::Topic_for_log log;
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d    euler_fcu_target;
    float              Thrust_target{0.0};
};
// ENU command log
struct ENU_command_log {
    float position[4];
};
// Drone general info
struct Drone_GeneralInfo {
    QString controllername;
    int TargetdroneID{0};
    bool isMulti{false};
};
// command geofence

struct CommandGeoFence{
    struct{
        float XMAX{1.6};
        float XMIN{-1.5};
        float YMAX{1.2};
        float YMIN{-1.2};
        float ZMAX{2.1};
        float ZMIN{0.0};
        float RMIN{0.3};
    }Indoor;
    struct{
        float XMAX{50.0};
        float XMIN{-50.0};
        float YMAX{50.0};
        float YMIN{-50.0};
        float ZMAX{20.0};
        float ZMIN{0.0};
        float RMIN{0.3};
    }Outdoor;
};

// parameter info from each drone at the start of the controller 
struct uav_para {
    std::string controllername;
    float dronemass;
    float cablelength;
    float a_j;
    float payloadmass;
    float motor_slope;
    float motor_intercept;
    int num_drone;
    bool isPubAuxiliaryState;
    bool isAddonForcedUsed;
    bool isCrossFeedingTermsUsed;
    float t_jx;
    float t_jy;
    float t_jz;
    float kv_xy;
    float kv_z;
    float kR_xy;
    float kR_z;
    float kL;
    float Kphi_xy;
    float Kphi_z;
    float kr1_x;
    float kr1_y;
    float kr1_z;
    float kr2_x;
    float kr2_y;
    float kr2_z;
    float kp_x;
    float kp_y;
    float kp_z;
    float komega_x;
    float komega_y;
    float komega_z;
    float lambdaj_x;
    float lambdaj_y;
    float lambdaj_z;
    float lambda_T_x;
    float lambda_T_y;
    float lambda_T_z;
    float lambda_R_x;
    float lambda_R_y;
    float lambda_R_z;
    float lambda1_x;
    float lambda1_y;
    float lambda1_z;
    float lambda2_x;
    float lambda2_y;
    float lambda2_z;
    float pxy_error_max;
    float pz_error_max;
    float pxy_int_max;
    float pz_int_max;
    float tilt_max;
    float int_start_error;
    float fp_max_x;
    float fp_max_y;
    float fp_max_z;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
        // TO DO: modify the list to display command history
	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
/*----------------------------Get states---------------------------------*/
        qt_ground_station::Topic_for_log GetLog(int ID);
        qt_ground_station::Mocap    GetMocap(int ID);
        qt_ground_station::uav_log  GetUAVLOG(int ID);
        qt_ground_station::uav_para GetUAVPARA(int ID);
        qt_ground_station::SinglePayloadAction GetSingleAction();
        qt_ground_station::MultiPayloadAction  GetMultiAction();
        qt_ground_station::CommandGeoFence     GetDroneGeoFence();
        bool IsPayloadDetected();
        bool IsPayloadControlSwitched();
        bool isNumberofDronesConsistent();
        bool isCooperativeModeConsistent();
        bool ispayloaddetected{false};
        bool ispayloadmocaprecieved{false};
        bool ispayloadcontrolactivated{false};
/*----------------------------Send commands------------------------------*/
        ENU_command_log command_log[3];
        ENUCommandError command_safty_check(int drone_ID,float target_state[4],bool IsOutdoor);
        void record_ENUCommand(int drone_ID, float target_state[4]);
        void resetcommandlog(int drone_Id);
        void move_ENU(int ID,float state_desired[4]);
        void takeoff(int ID);
        void land(int ID);
        void disarm(int ID);
        void payload_pose(float pose_desired[6]);
        void payload_land();
        void payload_singleUAV(int ID,float pose_desired[4]);
        void perform_action_singleUAV(bool isperform);
        void perfrom_action_multiUAV(bool isperform);
        Eigen::Vector3f UpdateHoverPosition(int ID, float height);
        void UseDroneLocationToSetGPSHome(int ID);
Q_SIGNALS:
	void loggingUpdated();
        void rosLoopUpdate();
        void rosShutdown();
        void rosParamServiceCallUAV0();
        void rosParamServiceCallUAV1();
        void rosParamServiceCallUAV2();
private:
        /*-------------------input arguments-----------------------*/
        int init_argc;
	    char** init_argv;
        /*-----------------------------------------------------------*/
        int DroneNumber;
        int comid;
        Drone_GeneralInfo GeneralInfoList[3];
        uav_log UavLogList[3];
        bool commandFlag[3];
        bool commandPayloadFlag;
        qt_ground_station::ControlCommand Command_List[3];
        qt_ground_station::Mocap mocap[3];
        qt_ground_station::Mocap mocap_payload;
        qt_ground_station::SinglePayloadAction action_msg;
        qt_ground_station::MultiPayloadAction multi_action_msg;
        qt_ground_station::SetHome sethome_msg;
        uav_para UavParaList[3];
        CommandGeoFence DroneFence;
        CommandGeoFence PayloadFence;
        /*------------------- motion pubs   ------------------------*/
        ros::Publisher moveUAV0;
        ros::Publisher moveUAV1;
        ros::Publisher moveUAV2;
        ros::ServiceServer serUpdateParameterUAV0;
        ros::ServiceServer serUpdateParameterUAV1;
        ros::ServiceServer serUpdateParameterUAV2;
        ros::ServiceServer serUpdateGeneralInfoUAV0;
        ros::ServiceServer serUpdateGeneralInfoUAV1;
        ros::ServiceServer serUpdateGeneralInfoUAV2;
        ros::ServiceClient clientSetGPSHome[3];
        void pub_command();
        ros::ServiceClient multiDroneActionClient;
        ros::ServiceClient singleDroneActionClient;
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

        void SubMocapUAV(const qt_ground_station::Mocap::ConstPtr& msg, int id);
        void SubMocapPayload(const qt_ground_station::Mocap::ConstPtr& msg);
        void SubTopicForLog(const qt_ground_station::Topic_for_log::ConstPtr &msg, int id);
        void SubThrustSetpointRaw(const mavros_msgs::AttitudeTarget::ConstPtr& msg,int id);

        QStringListModel logging_model;

        /*---------------------------utility functions ---------------------------*/
        void generate_com(int sub_mode, float state_desired[4],qt_ground_station::ControlCommand& Command_Now);
        void generate_com_for_payload(float state_desired[6],qt_ground_station::ControlCommand& Command_Now);

        /*-----------UAV para serive callbacks --------*/

        bool loadUAV0para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res);
        bool loadUAV1para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res);
        bool loadUAV2para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res);
        void loadUAVXpara(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res, int ID);

        bool LoadUAVGeneralInfo(qt_ground_station::GeneralInfo::Request& req, qt_ground_station::GeneralInfo::Response& res, int ID);

};

}  // namespace qt_ground_station

#endif /* qt_ground_station_QNODE_HPP_ */
