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
        resetcommandlog(0);
        resetcommandlog(1);
        resetcommandlog(2);
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
        namespace arg = std::placeholders;
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

        //UAV0_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV0,this);
        //UAV1_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV1,this);
        //UAV2_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/target_attitude", 100,&QNode::sub_setpoint_rawUpdateUAV2,this);
        UAV0_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/target_attitude", 
                                                                            100,
                                                                            std::bind(&QNode::sub_setpoint_rawUpdate,this,arg::_1,0));
        UAV1_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/target_attitude", 
                                                                            100,
                                                                            std::bind(&QNode::sub_setpoint_rawUpdate,this,arg::_1,1));
        UAV2_attitude_target_sub =n.subscribe<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/target_attitude", 
                                                                            100,
                                                                            std::bind(&QNode::sub_setpoint_rawUpdate,this,arg::_1,2));

        serUpdateParameterUAV0 = n.advertiseService("/uav0/px4_command/parameters", &QNode::loadUAV0para, this);
        serUpdateParameterUAV1 = n.advertiseService("/uav1/px4_command/parameters", &QNode::loadUAV1para, this);
        serUpdateParameterUAV2 = n.advertiseService("/uav2/px4_command/parameters", &QNode::loadUAV2para, this);
        // service to command the drone to perform action
        singleDroneActionClient = n.serviceClient<qt_ground_station::SinglePayloadAction>("/uav2/px4_command/action");
        multiDroneActionClient  = n.serviceClient<qt_ground_station::MultiPayloadAction>("/uav0/px4_command/multi_action");
        serUpdateGeneralInfoUAV0 = n.advertiseService("/uav0/px4_command/generalinfo", &QNode::loadUAV0generalinfo, this);
        serUpdateGeneralInfoUAV1 = n.advertiseService("/uav1/px4_command/generalinfo", &QNode::loadUAV1generalinfo, this);
        serUpdateGeneralInfoUAV2 = n.advertiseService("/uav2/px4_command/generalinfo", &QNode::loadUAV2generalinfo, this);

        // 
        GeneralInfoList[0].controllername = "---";
        GeneralInfoList[0].TargetdroneID = 0;
        GeneralInfoList[0].isMulti = false;

        GeneralInfoList[1].controllername = "---";
        GeneralInfoList[1].TargetdroneID = 0;
        GeneralInfoList[1].isMulti = false;

        GeneralInfoList[2].controllername = "---";
        GeneralInfoList[2].TargetdroneID = 0;
        GeneralInfoList[2].isMulti = false;

        // 
        action_msg.response.status_ok = false;
        action_msg.response.trajectory_type = 0;
        multi_action_msg.response.status_ok = false;
         multi_action_msg.response.trajectory_type = 0;
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

void QNode::sub_setpoint_rawUpdate(const mavros_msgs::AttitudeTarget::ConstPtr& msg,int id){
    UavLogList[id].q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    UavLogList[id].euler_fcu_target = quaternion_to_euler(UavLogList[id].q_fcu_target);
    UavLogList[id].Thrust_target= msg->thrust;
}

void QNode::perform_action_singleUAV(bool isperform){
    if (isperform){
        action_msg.request.perform_action = true;
        action_msg.request.action_type = 0;
    }else{
        action_msg.request.perform_action = false;
        action_msg.request.action_type = 0;
    }
    singleDroneActionClient.call(action_msg);
}

void QNode::perfrom_action_multiUAV(bool isperform){
    if(isperform){
        multi_action_msg.request.perform_action = true;
        multi_action_msg.request.action_type = 0;
    }else{
        multi_action_msg.request.perform_action = false;
        multi_action_msg.request.action_type = 0;
    }
    multiDroneActionClient.call(multi_action_msg);
}

qt_ground_station::SinglePayloadAction QNode::GetSingleAction(){
    return action_msg;
}

qt_ground_station::MultiPayloadAction QNode::GetMultiAction(){
    return multi_action_msg;
}

void QNode::loadUAVXpara(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res,int ID) {

    /*
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
    */
    UavParaList[ID].controllername = req.controllername;
    UavParaList[ID].dronemass = req.dronemass;
    UavParaList[ID].cablelength = req.cablelength;
    UavParaList[ID].a_j = req.a_j;
    UavParaList[ID].payloadmass = req.payloadmass;
    UavParaList[ID].motor_slope = req.motor_slope;
    UavParaList[ID].motor_intercept = req.motor_intercept;
    UavParaList[ID].num_drone = req.num_drone;
    UavParaList[ID].isPubAuxiliaryState  = req.isPubAuxiliaryState;
    UavParaList[ID].isAddonForcedUsed = req.isAddonForcedUsed;
    UavParaList[ID].isCrossFeedingTermsUsed = req.isCrossFeedingTermsUsed;
    UavParaList[ID].t_jx = req.t_jx;
    UavParaList[ID].t_jy = req.t_jy;
    UavParaList[ID].t_jz = req.t_jz;
    UavParaList[ID].kv_xy = req.kv_xy;
    UavParaList[ID].kv_z = req.kv_z;
    UavParaList[ID].kR_xy = req.kR_xy;
    UavParaList[ID].kR_z = req.kR_z;
    UavParaList[ID].kL = req.kL;
    UavParaList[ID].Kphi_xy = req.Kphi_xy;
    UavParaList[ID].Kphi_z = req.Kphi_z;
    UavParaList[ID].kr1_x  =req.kr1_x;
    UavParaList[ID].kr1_y  =req.kr1_y;
    UavParaList[ID].kr1_z  =req.kr1_z;
    UavParaList[ID].kr2_x  =req.kr2_x;
    UavParaList[ID].kr2_y  =req.kr2_y;
    UavParaList[ID].kr2_z  =req.kr2_z;
    UavParaList[ID].kp_x   =req.kp_x;
    UavParaList[ID].kp_y   =req.kp_y;
    UavParaList[ID].kp_z   =req.kp_z;
    UavParaList[ID].komega_x  =req.komega_x;
    UavParaList[ID].komega_y  =req.komega_y;
    UavParaList[ID].komega_z  =req.komega_z;
    UavParaList[ID].lambdaj_x = req.lambdaj_x;
    UavParaList[ID].lambdaj_y = req.lambdaj_y;
    UavParaList[ID].lambdaj_z = req.lambdaj_z;
    UavParaList[ID].lambda_T_x = req.lambda_T_x;
    UavParaList[ID].lambda_T_y = req.lambda_T_y;
    UavParaList[ID].lambda_T_z = req.lambda_T_z;
    UavParaList[ID].lambda_R_x = req.lambda_R_x;
    UavParaList[ID].lambda_R_y = req.lambda_R_y;
    UavParaList[ID].lambda_R_z = req.lambda_R_z;
    UavParaList[ID].lambda1_x  =req.lambda1_x;
    UavParaList[ID].lambda1_y  =req.lambda1_y;
    UavParaList[ID].lambda1_z  =req.lambda1_z;
    UavParaList[ID].lambda2_x  =req.lambda2_x;
    UavParaList[ID].lambda2_y  =req.lambda2_y;
    UavParaList[ID].lambda2_z  =req.lambda2_z;
    UavParaList[ID].pxy_error_max = req.pxy_error_max;
    UavParaList[ID].pz_error_max  = req.pz_error_max;
    UavParaList[ID].pxy_int_max = req.pxy_int_max;
    UavParaList[ID].pz_int_max  = req.pz_int_max;
    UavParaList[ID].tilt_max = req.tilt_max;
    UavParaList[ID].int_start_error = req.int_start_error;
    UavParaList[ID].fp_max_x = req.fp_max_x;
    UavParaList[ID].fp_max_y = req.fp_max_y;
    UavParaList[ID].fp_max_z = req.fp_max_z;
    res.oktostart = true; // reply true back to quadrotor
}

bool QNode::loadUAV0para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res){

    loadUAVXpara(req,res,0);
    Q_EMIT rosParamServiceCallUAV0();
    return true;
}

bool QNode::loadUAV1para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res){

    loadUAVXpara(req,res,1);
    Q_EMIT rosParamServiceCallUAV1();
    return true;
}

bool QNode::loadUAV2para(qt_ground_station::ControlParameter::Request& req, qt_ground_station::ControlParameter::Response& res){

    loadUAVXpara(req,res,2);
    Q_EMIT rosParamServiceCallUAV2();
    return true;
}

bool QNode::loadUAV0generalinfo(qt_ground_station::GeneralInfo::Request& req, 
                                qt_ground_station::GeneralInfo::Response& res){
    loadUAVXgeneralinfo(req, res, 0);
    return true;
}

bool QNode::loadUAV1generalinfo(qt_ground_station::GeneralInfo::Request& req, 
                                qt_ground_station::GeneralInfo::Response& res){
    loadUAVXgeneralinfo(req, res, 1);
    return true;
}

bool QNode::loadUAV2generalinfo(qt_ground_station::GeneralInfo::Request& req, 
                                qt_ground_station::GeneralInfo::Response& res){
    loadUAVXgeneralinfo(req, res, 2);
    return true;
}

void QNode::loadUAVXgeneralinfo(qt_ground_station::GeneralInfo::Request& req, 
                                qt_ground_station::GeneralInfo::Response& res, 
                                int ID){

    GeneralInfoList[ID].controllername = QString::fromStdString(req.controllername);
    GeneralInfoList[ID].TargetdroneID = req.TargetdroneID;
    GeneralInfoList[ID].isMulti = req.isMulti;
    res.oktostart = true; // reply true back to quadrotor
}

bool QNode::isNumberofDronesConsistent(){
    bool isconsistent = true;
    if(UavLogList[0].isconnected) {
        int num_of_drones = UavParaList[0].num_drone;
    
        if (num_of_drones>3){
            isconsistent = false;
        }else if(num_of_drones<1){
            isconsistent = false;
        }

        for(int i = 1;i<num_of_drones;i++){
            if(UavParaList[i].num_drone != num_of_drones) {
                isconsistent = false;
            }
        }
    }else{
        isconsistent = false;
    }
    return isconsistent;
}

bool QNode::isCooperativeModeConsistent(){
    bool isconsistent = true;
    for(int i = 1;i<UavParaList[0].num_drone;i++) {
        if(GeneralInfoList[i].isMulti = false ) {
            isconsistent = false;
        }        
    }

    return isconsistent;
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


qt_ground_station::uav_para QNode::GetUAVPARA(int ID) {

   return UavParaList[ID];
}

bool QNode::IsPayloadDetected() {

    // determinen whether the payload is detected
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

void QNode::payload_singleUAV(int ID,float pose_desired[4]) {
    commandFlag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Payload_Stabilization_SingleUAV;
    generate_com(0, pose_desired,Command_List[ID]);
}

void QNode::record_ENUCommand(int drone_ID, float target_state[4]){
    // swtich to each drones based on drone ID
    if(UavLogList[drone_ID].isconnected){ // only log the 
        command_log[drone_ID].position[0] = target_state[0];
        command_log[drone_ID].position[1] = target_state[1];
        command_log[drone_ID].position[2] = target_state[2];
    }else{
        resetcommandlog(drone_ID);
    }
}

ENUCommandError QNode::command_safty_check(int drone_ID, float target_state[4]){
    ENUCommandError error_msg;
    // check whether the drone command is inside the box
    // get the ID of the other 2 drones
    int otherDroneID[2];

    bool input_is_inside_box = true;

    if(target_state[0]<-1.5 || target_state[0]> 1.6) {
        input_is_inside_box = false;
    }

    if(target_state[1]< -1.2 || target_state[1]> 1.2) {
        input_is_inside_box = false;
    }

    if(target_state[2]< 0|| target_state[2]> 2.1) {
        input_is_inside_box = false;
    }
    // check whether the drone command is close to close to other drones (collsion radius = 0.3 m )
    bool command_is_farway_from_others = true;
  
    switch (drone_ID)
    {
        case 0:{
            otherDroneID[0] = 1;
            otherDroneID[1] = 2;
            break;
        }
        case 1:{
            otherDroneID[0] = 2;
            otherDroneID[1] = 0;
            break;
        }
        case 2:{
            otherDroneID[0] = 0;
            otherDroneID[1] = 1;
            break;
        }
    }
    for(int i = 0;i<2;i++)
    {
        if(UavLogList[otherDroneID[i]].isconnected) // determine whether the other drones exists
        {
            double r_2 = pow((target_state[0]- mocap[otherDroneID[i]].position[0]),2) + 
                         pow((target_state[1]- mocap[otherDroneID[i]].position[1]),2) +
                         pow((target_state[2]- mocap[otherDroneID[i]].position[2]),2);
            if(sqrt(r_2)<0.3) // too close
            {
                command_is_farway_from_others = false;
            }
        }
    }
    // check whether the drone command is close to other drone command positions
    bool command_is_farway_from_othercommands = true;
    for(int i=0; i < 2; i++){
        if(UavLogList[otherDroneID[i]].isconnected) // determine whether the other drones exists
        {
            double r_2 = pow((target_state[0]- command_log[otherDroneID[i]].position[0]),2) + 
                         pow((target_state[1]- command_log[otherDroneID[i]].position[1]),2) + 
                         pow((target_state[2]- command_log[otherDroneID[i]].position[2]),2);
            if(sqrt(r_2)<0.3) // too close
            {
                command_is_farway_from_othercommands  = false;
            }
        }
    }

    if(!input_is_inside_box) {
        error_msg = DRONE_COMMAND_OUTOFBOUND;
    } else if(!command_is_farway_from_others){ 
        error_msg = DRONE_COMMAND_TOOCLOSETOOTHER;
    } else if (!command_is_farway_from_othercommands) {
        error_msg = DRONE_COMMAND_TOOCLOSETOOTHERCOMMAND;
    } else {
        error_msg = DRONE_COMMAND_NORM;
    }

    return error_msg;
}

void QNode::resetcommandlog(int drone_Id) {
    command_log[drone_Id].position[0] = 0;
    command_log[drone_Id].position[1] = 0;
    command_log[drone_Id].position[2] = -1;
    command_log[drone_Id].position[3] = 0;
}

Eigen::Vector3f QNode::UpdateHoverPosition(int ID, float height) {

    Eigen::Vector3f output;
    Eigen::Vector3f t_j;
    t_j(0) = UavParaList[ID].t_jx;
    t_j(1) = UavParaList[ID].t_jy;
    t_j(2) = UavParaList[ID].t_jz;
    Eigen::Vector3f Xp;
    Eigen::Vector4f quaternion;
    for(int i =0 ; i < 3 ; i++) {
        Xp(i) = mocap_payload.position[i];
    }
    for (int i =0; i <4 ; i ++) {
        quaternion(i) = mocap_payload.quaternion[i];
    }
    Eigen::Matrix3f R_IP =  QuaterionToRotationMatrix(quaternion);
    Eigen::Vector3f h;
    h<< 0,
        0,
        height;

    output = h + R_IP * t_j + Xp;

    return output;
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
