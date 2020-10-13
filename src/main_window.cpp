/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qt_ground_station/main_window.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_ground_station {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    /*------------init ros node -----------*/
    bool init_ros_ok = qnode.init();
    if ( !init_ros_ok ) {
            showNoMasterMessage();
    } else {
        //ui.button_connect->setEnabled(false);
    }

    /*********************
    ** Logging
    **********************/
    //ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV0log()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV1log()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV2log()));

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV0pos()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV1pos()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV2pos()));

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV0attReference()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV1attReference()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV2attReference()));

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updatePayloadpos()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateMode()));
    QObject::connect(&qnode, SIGNAL(rosParamServiceCallUAV0()), this, SLOT(updateUAV0Param()));
    QObject::connect(&qnode, SIGNAL(rosParamServiceCallUAV1()), this, SLOT(updateUAV1Param()));
    QObject::connect(&qnode, SIGNAL(rosParamServiceCallUAV2()), this, SLOT(updateUAV2Param()));
    /* -----------------------------update labels --------------------------------*/
    ui.UAV0_connection->setText("<font color='red'>UNCONNECTED</font>");
    ui.UAV0_arm->setText("<font color='red'>DISARMED</font>");
    ui.UAV0_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    ui.UAV0_detection->setText("<font color='red'>UAV0 Undetected!</font>");

    ui.UAV1_connection->setText("<font color='red'>UNCONNECTED</font>");
    ui.UAV1_arm->setText("<font color='red'>DISARMED</font>");
    ui.UAV1_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    ui.UAV1_detection->setText("<font color='red'>UAV1 Undetected!</font>");

    ui.UAV2_connection->setText("<font color='red'>UNCONNECTED</font>");
    ui.UAV2_arm->setText("<font color='red'>DISARMED</font>");
    ui.UAV2_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    ui.UAV2_detection->setText("<font color='red'>UAV2 Undetected!</font>");

    ui.Payload_detection->setText("<font color='red'>Payload Undetected!</font>");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_UAV0_Button_Takeoff_clicked(bool check ) {
    qnode.takeoff(0);
}
void MainWindow::on_UAV1_Button_Takeoff_clicked(bool check ) {
    qnode.takeoff(1);
}
void MainWindow::on_UAV2_Button_Takeoff_clicked(bool check ) {
    qnode.takeoff(2);
}
void MainWindow::on_UAV0_Button_moveENU_clicked(bool check){
    /* read values from line edit */
    float target_state[4];

    target_state[0] =  ui.UAV0_Target_x->text().toFloat();
    target_state[1] =  ui.UAV0_Target_y->text().toFloat();
    target_state[2] =  ui.UAV0_Target_z->text().toFloat();
    target_state[3] = 0;
    /*----------------determine whether the input is in safe range ------------------*/

    qnode.record_ENUCommand(0, target_state); // push the command to ENU_log
    qt_ground_station::ENUCommandError error_msg = qnode.command_safty_check(0, target_state);

    /*----------------send input ------------------*/

    if(error_msg == qt_ground_station::DRONE_COMMAND_NORM){
        /*  update the ENU target label */
        ui.UAV0_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV0_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV0_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/
        qnode.move_ENU(0,target_state);
        UpdateListViewENU(0,target_state);
    } else {
        // reject the command and display the error_msg
        DisplayENUErrorMsg(error_msg);
    };
}

void MainWindow::on_UAV1_Button_moveENU_clicked(bool check){
    /* read values from line edit */
    float target_state[4];

    target_state[0] =  ui.UAV1_Target_x->text().toFloat();
    target_state[1] =  ui.UAV1_Target_y->text().toFloat();
    target_state[2] =  ui.UAV1_Target_z->text().toFloat();
    target_state[3] = 0;
    /*----------------determine whether the input is in safe range ------------------*/

    qnode.record_ENUCommand(1, target_state); // push the command to ENU_log
    qt_ground_station::ENUCommandError error_msg = qnode.command_safty_check(1, target_state);

    /*----------------send input ------------------*/
    if(error_msg == qt_ground_station::DRONE_COMMAND_NORM){
        /*  update the ENU target label */
        ui.UAV1_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV1_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV1_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/
        qnode.move_ENU(1,target_state);
        UpdateListViewENU(1,target_state);
    } else {
        // reject the command and display the error_msg
        DisplayENUErrorMsg(error_msg);
    };
}

void MainWindow::on_UAV2_Button_moveENU_clicked(bool check){
    /* read values from line edit */
    float target_state[4];

    target_state[0] =  ui.UAV2_Target_x->text().toFloat();
    target_state[1] =  ui.UAV2_Target_y->text().toFloat();
    target_state[2] =  ui.UAV2_Target_z->text().toFloat();
    target_state[3] = 0;
    /*----------------determine whether the input is in safe range ------------------*/

    qnode.record_ENUCommand(2, target_state); // push the command to ENU_log
    qt_ground_station::ENUCommandError error_msg = qnode.command_safty_check(2, target_state);
                 
    /*----------------send input ------------------*/

    if(error_msg == qt_ground_station::DRONE_COMMAND_NORM){
        /*  update the ENU target label */
        ui.UAV2_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV2_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV2_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/
        UpdateListViewENU(2,target_state);
        qnode.move_ENU(2,target_state);
    } else {
        // reject the command and display the error_msg
        DisplayENUErrorMsg(error_msg);
    };
}

void  MainWindow::on_UAV2_Move_with_payload_clicked(bool check) {
    /*Enter Single UAV mode*/
    /* read values from line edit */
    float target_state[4];

    target_state[0] =  ui.UAV2_Target_x->text().toFloat();
    target_state[1] =  ui.UAV2_Target_y->text().toFloat();
    target_state[2] =  ui.UAV2_Target_z->text().toFloat();
    target_state[3] = 0;
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    if(target_state[0]<-1.5 || target_state[0]> 1.6) {
        input_is_valid = false;
    }

    if(target_state[1]< -1.2 || target_state[1]> 1.2) {
        input_is_valid = false;
    }

    if(target_state[2]< 0|| target_state[2]> 2) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.UAV2_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV2_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV2_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*------------- set drone into single UAV payload mode ----------------------*/
        qnode.payload_singleUAV(2,target_state);
        UpdateSwitchToSinglePayloadMode(target_state);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}


void MainWindow::on_UAV2_Back_to_ENU_clicked(bool check) {
    /*bounce back to moveENU mode*/
     qnode.payload_land();
     UpdateBacktoENU();
}

void MainWindow::on_UAV0_Button_Land_clicked(bool check) {
    qnode.land(0);
}

void MainWindow::on_UAV1_Button_Land_clicked(bool check) {
    qnode.land(1);
}

void MainWindow::on_UAV2_Button_Land_clicked(bool check) {
    qnode.land(2);
}

void MainWindow::on_UAV0_Button_Disarm_clicked(bool check) {
    qnode.disarm(0);
}

void MainWindow::on_UAV1_Button_Disarm_clicked(bool check) {
    qnode.disarm(1);
}

void MainWindow::on_UAV2_Button_Disarm_clicked(bool check) {
    qnode.disarm(2);
}

void MainWindow::on_Button_DisarmALL_clicked(bool check) {
    qnode.disarm(0);
    qnode.disarm(1);
    qnode.disarm(2);
}

void MainWindow::on_Payload_Pose_Button_clicked(bool check) {
    /* read values from line edit */
    float pose_target[6];

    pose_target[0] =  ui.Payload_Target_x->text().toFloat();
    pose_target[1] =  ui.Payload_Target_y->text().toFloat();
    pose_target[2] =  ui.Payload_Target_z->text().toFloat();

    pose_target[3] =  ui.Payload_Target_roll->text().toFloat();
    pose_target[4] =  ui.Payload_Target_pitch->text().toFloat();
    pose_target[5] =  ui.Payload_Target_yaw->text().toFloat();


    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    /*-------position boundary--------*/

    if(pose_target[0]<-0.8 || pose_target[0]> 0.8) {
        input_is_valid = false;
    }

    if(pose_target[1]< -0.9 || pose_target[1]> 0.9) {
        input_is_valid = false;
    }

    if(pose_target[2]< 0|| pose_target[2]> 1) {
        input_is_valid = false;
    }
    /*-------angle boundary--------*/
    if(pose_target[3]<-10 || pose_target[3]> 10) {
        input_is_valid = false;
    }

    if(pose_target[4]< -10 || pose_target[4]> 10) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        //ui.UAV2_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        //ui.UAV2_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        //ui.UAV2_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- send payload stabilization command --------------------------------*/

        qnode.payload_pose(pose_target);
        UpdateListViewMultiPayload(pose_target);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Payload_Land_Button_clicked(bool check) {
      qnode.payload_land();
      UpdataListViewLand();
}

void MainWindow::on_Flush_MoveENU_Button_clicked(bool check) {
    // record all the command first
    float target_state[4];
    // command for uav 0:
    target_state[0] =  ui.UAV0_Target_x->text().toFloat();
    target_state[1] =  ui.UAV0_Target_y->text().toFloat();
    target_state[2] =  ui.UAV0_Target_z->text().toFloat();
    target_state[3] = 0;
    qnode.record_ENUCommand(0, target_state); // push the command to ENU_log
    // command for uav 1:
    target_state[0] =  ui.UAV1_Target_x->text().toFloat();
    target_state[1] =  ui.UAV1_Target_y->text().toFloat();
    target_state[2] =  ui.UAV1_Target_z->text().toFloat();
    target_state[3] = 0;
    qnode.record_ENUCommand(1, target_state); // push the command to ENU_log
    // command for uav 2:
    target_state[0] =  ui.UAV2_Target_x->text().toFloat();
    target_state[1] =  ui.UAV2_Target_y->text().toFloat();
    target_state[2] =  ui.UAV2_Target_z->text().toFloat();
    target_state[3] = 0;
    qnode.record_ENUCommand(2, target_state); // push the command to ENU_log
    // then perform the actual command:

    on_UAV0_Button_moveENU_clicked(true);
    on_UAV1_Button_moveENU_clicked(true);
    on_UAV2_Button_moveENU_clicked(true);
}

void MainWindow::on_Payload_Move_to_Start_clicked(bool check) {

    // obtain the target position for each drone

    Eigen::Vector3f temp_position;
    float target_state[4];
    float height = ui.Payload_Hovering_Height->text().toFloat();

    if(height >= 1.2) {
        QMessageBox msgBox;
        msgBox.setText("Hovering Point Too High !!");
        msgBox.exec();
    } else {
        for(int i = 0; i < 3 ; i++) {
            temp_position = qnode.UpdateHoverPosition(i,  height);
            target_state[0] =  temp_position(0);
            target_state[1] =  temp_position(1);
            target_state[2] =  temp_position(2);
            target_state[3] = 0;
            qnode.move_ENU(i,target_state);
        }
        UpdateListViewMoveToHoverPoint();
    }
}

void MainWindow::on_Payload_Prelift_clicked(bool check){
    // move to lift payload

    // obtain the target position for each drone

    Eigen::Vector3f temp_position;
    float target_state[4];
    float height = ui.Payload_Prelift_Height ->text().toFloat();

    if(height >= 1.3) {
        QMessageBox msgBox;
        msgBox.setText("Prelift Point Too High !!");
        msgBox.exec();
    } else {
        for(int i = 0; i < 3 ; i++) {
            temp_position = qnode.UpdateHoverPosition(i,  height);
            target_state[0] =  temp_position(0);
            target_state[1] =  temp_position(1);
            target_state[2] =  temp_position(2);
            target_state[3] = 0;
            qnode.move_ENU(i,target_state);
        }
        UpdateListViewPrelift();
    }

}

void MainWindow::on_UAV0_Copypos_clicked(bool check) {
    ui.UAV0_Target_x->setText(ui.UAV0_x->text());
    ui.UAV0_Target_y->setText(ui.UAV0_y->text());
    ui.UAV0_Target_z->setText(ui.UAV0_z->text());
}

void MainWindow::on_UAV1_Copypos_clicked(bool check) {
    ui.UAV1_Target_x->setText(ui.UAV1_x->text());
    ui.UAV1_Target_y->setText(ui.UAV1_y->text());
    ui.UAV1_Target_z->setText(ui.UAV1_z->text());
}

void MainWindow::on_UAV2_Copypos_clicked(bool check) {
    ui.UAV2_Target_x->setText(ui.UAV2_x->text());
    ui.UAV2_Target_y->setText(ui.UAV2_y->text());
    ui.UAV2_Target_z->setText(ui.UAV2_z->text());
}

void MainWindow::on_Toggledisplaymode_Button_clicked(bool check) {
    IsOutDoor = !IsOutDoor;
    QString dispmode;
    if(IsOutDoor){
        dispmode = "Outdoor Mode";
    } else {
        dispmode = "Indoor Mode";
    }

    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : switched to " + dispmode + ".";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::red);
}

void MainWindow::on_Togglecontrolmode_Button_clicked(bool check) {
    IsMulti = !IsMulti;
    QString controlmode;
    if(IsMulti){
        controlmode = "MultiDrone Mode";
    } else {
        controlmode = "SingleDrone Mode";
    }
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : switched to " + controlmode + ".";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::red);
}

void MainWindow::on_ClearLog_Button_clicked(bool check) {
    while(ui.logger1->count()>0) // clear all the items
    {
        ui.logger1->takeItem(0);
    }

}

void MainWindow::on_Multi_action_clicked(bool check){
    qnode.perfrom_action_multiUAV(true);
    if(qnode.GetMultiAction().response.status_ok){
        ui.Multi_action_status->setText("Action Status: OK!");
        ui.Multi_trajecotry_type->setText("Trajectory Type: " + QString::number(qnode.GetMultiAction().response.trajectory_type));
    }else{
        ui.Multi_action_status->setText("Action Status: NOT IN ACTION MODE!");
    }
    UpdataListViewMultiAction();
}

void MainWindow::on_Multi_stopaction_clicked(bool check){
    qnode.perfrom_action_multiUAV(false);
    if(qnode.GetMultiAction().response.status_ok){
        ui.Multi_action_status->setText("Action Status: OK!");
        ui.Multi_trajecotry_type->setText("Trajectory Type: " + QString::number(qnode.GetMultiAction().response.trajectory_type));
    }else{
        ui.Multi_action_status->setText("Action Status: NOT IN ACTION MODE!");
    }
    UpdateListViewMultiStopAction();
}

void MainWindow::on_action_single_clicked(bool check){

    qnode.perform_action_singleUAV(true);
    if(qnode.GetSingleAction().response.status_ok){
        ui.action_status->setText("Action Status: OK!");
        ui.action_trajectory_type->setText("Trajectory Type: " + QString::number(qnode.GetSingleAction().response.trajectory_type));
    }else{
        ui.action_status->setText("Action Status: NOT IN ACTION MODE!");
    }
    UpdateListViewSingleAction();
}

void MainWindow::on_stop_action_single_clicked(bool check){

    qnode.perform_action_singleUAV(false);
    if(qnode.GetSingleAction().response.status_ok){
        ui.action_status->setText("Action Status: OK!");
        ui.action_trajectory_type->setText("Trajectory Type: " + QString::number(qnode.GetSingleAction().response.trajectory_type));
    }else{
        ui.action_status->setText("Action Status: NOT IN ACTION MODE!");
    }
    UpdateListViewSingleStopAction();
}

void MainWindow::on_UAV2_Back_to_ENU_1_clicked(bool check){

    qnode.payload_land();
    UpdateBacktoENU();
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qt_ground_station");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qt_ground_station");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

/******************************SLOTS************************************/
void MainWindow::updateUAV0pos() {
    Eigen::Vector3f Position;
    Eigen::Vector3f Velocity;
    
    UpdateUAVPos(Position, Velocity, 0);

    ui.UAV0_x->setText(QString::number(Position(0), 'f', 3));
    ui.UAV0_y->setText(QString::number(Position(1), 'f', 3));
    ui.UAV0_z->setText(QString::number(Position(2), 'f', 3));

    ui.UAV0_vx->setText(QString::number(Velocity(0), 'f', 3));
    ui.UAV0_vy->setText(QString::number(Velocity(1), 'f', 3));
    ui.UAV0_vz->setText(QString::number(Velocity(2), 'f', 3));
}

void MainWindow::updateUAV1pos() {
    Eigen::Vector3f Position;
    Eigen::Vector3f Velocity;
    
    UpdateUAVPos(Position, Velocity, 1);

    ui.UAV1_x->setText(QString::number(Position(0), 'f', 3));
    ui.UAV1_y->setText(QString::number(Position(1), 'f', 3));
    ui.UAV1_z->setText(QString::number(Position(2), 'f', 3));

    ui.UAV1_vx->setText(QString::number(Velocity(0), 'f', 3));
    ui.UAV1_vy->setText(QString::number(Velocity(1), 'f', 3));
    ui.UAV1_vz->setText(QString::number(Velocity(2), 'f', 3));

}

void MainWindow::updateUAV2pos() {
    Eigen::Vector3f Position;
    Eigen::Vector3f Velocity;
    
    UpdateUAVPos(Position, Velocity, 2);

    ui.UAV2_x->setText(QString::number(Position(0), 'f', 3));
    ui.UAV2_y->setText(QString::number(Position(1), 'f', 3));
    ui.UAV2_z->setText(QString::number(Position(2), 'f', 3));

    ui.UAV2_vx->setText(QString::number(Velocity(0), 'f', 3));
    ui.UAV2_vy->setText(QString::number(Velocity(1), 'f', 3));
    ui.UAV2_vz->setText(QString::number(Velocity(2), 'f', 3));

    ui.action_dron_pos->setText("Drone X: " + QString::number(Position(0), 'f', 3) + " m, Y: "
                                            + QString::number(Position(1), 'f', 3) + " m, Z: "
                                            + QString::number(Position(2), 'f', 3) + " m");
}

void MainWindow::updateUAV0Param() {
    qt_ground_station::uav_para param = qnode.GetUAVPARA(0);
    ui.UAV0_controllername->setText("Controller: " + QString::fromStdString(param.controllername));
    ui.UAV0_Quadmass->setText("Quad Mass (kg): " + QString::number(param.dronemass, 'f', 2));
    ui.UAV0_Payloadmass->setText("Payload Mass (kg): " + QString::number(param.payloadmass, 'f', 2));
    ui.UAV0_Cablelength->setText("Cable Length (m): " + QString::number(param.cablelength, 'f', 2));
    ui.UAV0_motor_para->setText("Motor slop : " + QString::number(param.motor_slope, 'f', 2) + ", Intercept: " +  QString::number(param.motor_intercept, 'f', 2));
    ui.UAV0_a_j->setText("a_0 : " + QString::number(param.a_j, 'f', 2));
    ui.UAV0_t_jx->setText("t_0x(m) : " + QString::number(param.t_jx, 'f', 2));
    ui.UAV0_t_jy->setText("t_0y(m) : " + QString::number(param.t_jy, 'f', 2));
    ui.UAV0_t_jz->setText("t_0z(m) : " + QString::number(param.t_jz, 'f', 2));
    ui.UAV0_Numofdrones->setText("Num of Drones : " + QString::number(param.num_drone));

    QString isAddonForce       = "Addon Force OFF!";
    QString isCrossFeedingTerm = "Cross Feeding Term OFF! ";
    if (param.isAddonForcedUsed) {
        isAddonForce       = "Addon Force ON!";
    }
    if (param.isCrossFeedingTermsUsed) {
        isCrossFeedingTerm = "Cross Feeding Term ON!";
    }

    ui.UAV0_control_option->setText(isAddonForce + ", " + isCrossFeedingTerm);

    ui.UAV0_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " +
                                  QString::number(param.kv_xy, 'f', 2) + ", " +
                                  QString::number(param.kv_z, 'f', 2 ));

    ui.UAV0_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " +
                                  QString::number(param.kR_xy, 'f', 2) + ", " +
                                  QString::number(param.kR_z, 'f', 2 ));
    // Lambda_j : ---- Lambda_p : -----  Lambda_r : ----
    ui.UAV0_UDE_lambda->setText("Laj : " + QString::number(param.lambdaj_x) + ", "
                                         + QString::number(param.lambdaj_y) + ", "
                                         + QString::number(param.lambdaj_z));
    ui.UAV0_inter_drone->setText ("La_T: " + QString::number(param.lambda_T_x) + ", "
                                  + QString::number(param.lambda_T_y) + ", "
                                  + QString::number(param.lambda_T_z) + ", "
                   +"La_R : " + QString::number(param.lambda_R_x) + ", "
                                  + QString::number(param.lambda_R_y) + ", "
                                  + QString::number(param.lambda_R_z));

    ui.UAV0_kL ->setText("kL : " + QString::number(param.kL, 'f', 2));
    ui.UAV0_kphi->setText("kphi: " + QString::number(param.Kphi_xy, 'f', 2) + ", " + QString::number(param.Kphi_xy, 'f', 2) + ", " +  QString::number(param.Kphi_z, 'f', 2 ));
    ui.UAV0_kr ->setText("kr1 : " + QString::number(param.kr1_x) + ", " + QString::number(param.kr1_y) + ", " + QString::number(param.kr1_z) +
                        " kr2 : " + QString::number(param.kr2_x) + ", " + QString::number(param.kr2_y) + ", " + QString::number(param.kr2_z));
    ui.UAV0_lambda -> setText("lambda1 : " + QString::number(param.lambda1_x) + ", " + QString::number(param.lambda1_y) + ", " + QString::number(param.lambda1_z) +
                              "lambda2 : " + QString::number(param.lambda2_x) + ", " + QString::number(param.lambda2_y) + ", " + QString::number(param.lambda2_z));
    ui.UAV0_kp -> setText ("kp : " + QString::number(param.kp_x) + ", " + QString::number(param.kp_y) + ", " + QString::number(param.kp_z)+
                           "komega : " + QString::number(param.komega_x) + ", " + QString::number(param.komega_y) + ", " + QString::number(param.komega_z));
    ui.UAV0_p_error_max->setText("p_error_max: " + QString::number(param.pxy_error_max, 'f', 2) + ", " + QString::number(param.pxy_error_max, 'f', 2) + ", " +  QString::number(param.pz_error_max, 'f', 2 ));
    ui.UAV0_p_int_max->setText("p_int_max: " + QString::number(param.pxy_int_max, 'f', 2) + ", " + QString::number(param.pxy_int_max, 'f', 2) + ", " +  QString::number(param.pz_int_max, 'f', 2 ));
    ui.UAV0_tilt_max->setText("tilt_max : " + QString::number(param.tilt_max, 'f', 2));
    ui.UAV0_fp_max->setText("fp_max: " + QString::number(param.fp_max_x, 'f', 2) + ", " + QString::number(param.fp_max_y, 'f', 2) + ", " +  QString::number(param.fp_max_z, 'f', 2 ));
    ui.UAV0_int_start_error->setText("int_start_error : " + QString::number(param.int_start_error, 'f', 2));


}

void MainWindow::updateUAV1Param() {
    qt_ground_station::uav_para param = qnode.GetUAVPARA(1);
    ui.UAV1_controllername->setText("Controller: " + QString::fromStdString(param.controllername));
    ui.UAV1_Quadmass->setText("Quad Mass (kg): " + QString::number(param.dronemass, 'f', 2));
    ui.UAV1_Payloadmass->setText("Payload Mass (kg): " + QString::number(param.payloadmass, 'f', 2));
    ui.UAV1_Cablelength->setText("Cable Length (m): " + QString::number(param.cablelength, 'f', 2));
    ui.UAV1_motor_para->setText("Motor slop : " + QString::number(param.motor_slope, 'f', 2) + ", Intercept: " +  QString::number(param.motor_intercept, 'f', 2));
    ui.UAV1_a_j->setText("a_1 : " + QString::number(param.a_j, 'f', 2));
    ui.UAV1_t_jx->setText("t_1x(m) : " + QString::number(param.t_jx, 'f', 2));
    ui.UAV1_t_jy->setText("t_1y(m) : " + QString::number(param.t_jy, 'f', 2));
    ui.UAV1_t_jz->setText("t_1z(m) : " + QString::number(param.t_jz, 'f', 2));
    ui.UAV1_Numofdrones->setText("Num of Drones : " + QString::number(param.num_drone));

    QString isAddonForce       = "Addon Force OFF!";
    QString isCrossFeedingTerm = "Cross Feeding Term OFF! ";
    if (param.isAddonForcedUsed) {
        isAddonForce       = "Addon Force ON!";
    }
    if (param.isCrossFeedingTermsUsed) {
        isCrossFeedingTerm = "Cross Feeding Term ON!";
    }

    ui.UAV1_control_option->setText(isAddonForce + ", " + isCrossFeedingTerm);

    ui.UAV1_kr ->setText("kr1 : " + QString::number(param.kr1_x) + ", " + QString::number(param.kr1_y) + ", " + QString::number(param.kr1_z) +
                        " kr2 : " + QString::number(param.kr2_x) + ", " + QString::number(param.kr2_y) + ", " + QString::number(param.kr2_z));
    ui.UAV1_lambda -> setText("lambda1 : " + QString::number(param.lambda1_x) + ", " + QString::number(param.lambda1_y) + ", " + QString::number(param.lambda1_z) +
                              "lambda2 : " + QString::number(param.lambda2_x) + ", " + QString::number(param.lambda2_y) + ", " + QString::number(param.lambda2_z));
    ui.UAV1_kp -> setText ("kp : " + QString::number(param.kp_x) + ", " + QString::number(param.kp_y) + ", " + QString::number(param.kp_z)+
                           "komega : " + QString::number(param.komega_x) + ", " + QString::number(param.komega_y) + ", " + QString::number(param.komega_z));

    ui.UAV1_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " + QString::number(param.kv_xy, 'f', 2) + ", " +  QString::number(param.kv_z, 'f', 2 ));
    ui.UAV1_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " + QString::number(param.kR_xy, 'f', 2) + ", " +  QString::number(param.kR_z, 'f', 2 ));

    // Lambda_j : ---- Lambda_p : -----  Lambda_r : ----
    ui.UAV1_UDE_lambda->setText("Laj : " + QString::number(param.lambdaj_x) + ", "
                                              + QString::number(param.lambdaj_y) + ", "
                                              + QString::number(param.lambdaj_z));

    ui.UAV1_kL ->setText("kL : " + QString::number(param.kL, 'f', 2));
    ui.UAV1_kphi->setText("kphi: " + QString::number(param.Kphi_xy, 'f', 2) + ", " + QString::number(param.Kphi_xy, 'f', 2) + ", " +  QString::number(param.Kphi_z, 'f', 2 ));
    ui.UAV1_p_error_max->setText("p_error_max: " + QString::number(param.pxy_error_max, 'f', 2) + ", " + QString::number(param.pxy_error_max, 'f', 2) + ", " +  QString::number(param.pz_error_max, 'f', 2 ));
    ui.UAV1_p_int_max->setText("p_int_max: " + QString::number(param.pxy_int_max, 'f', 2) + ", " + QString::number(param.pxy_int_max, 'f', 2) + ", " +  QString::number(param.pz_int_max, 'f', 2 ));
    ui.UAV1_tilt_max->setText("tilt_max : " + QString::number(param.tilt_max, 'f', 2));
    ui.UAV1_fp_max->setText("fp_max: " + QString::number(param.fp_max_x, 'f', 2) + ", " + QString::number(param.fp_max_y, 'f', 2) + ", " +  QString::number(param.fp_max_z, 'f', 2 ));
    ui.UAV1_int_start_error->setText("int_start_error : " + QString::number(param.int_start_error, 'f', 2));
}

void MainWindow::updateUAV2Param() {
    qt_ground_station::uav_para param = qnode.GetUAVPARA(2);
    ui.UAV2_controllername->setText("Controller: " + QString::fromStdString(param.controllername));
    ui.UAV2_Quadmass->setText("Quad Mass (kg): " + QString::number(param.dronemass, 'f', 2));
    ui.UAV2_Payloadmass->setText("Payload Mass (kg): " + QString::number(param.payloadmass, 'f', 2));
    ui.UAV2_Cablelength->setText("Cable Length (m): " + QString::number(param.cablelength, 'f', 2));
    ui.UAV2_motor_para->setText("Motor slop : " + QString::number(param.motor_slope, 'f', 2) + ", Intercept: " +  QString::number(param.motor_intercept, 'f', 2));
    ui.UAV2_a_j->setText("a_2 : " + QString::number(param.a_j, 'f', 2));
    ui.UAV2_t_jx->setText("t_2x(m) : " + QString::number(param.t_jx, 'f', 2));
    ui.UAV2_t_jy->setText("t_2y(m) : " + QString::number(param.t_jy, 'f', 2));
    ui.UAV2_t_jz->setText("t_2z(m) : " + QString::number(param.t_jz, 'f', 2));
    ui.UAV2_Numofdrones->setText("Num of Drones : " + QString::number(param.num_drone));

    QString isAddonForce       = "Addon Force OFF!";
    QString isCrossFeedingTerm = "Cross Feeding Term OFF! ";
    if (param.isAddonForcedUsed) {
        isAddonForce       = "Addon Force ON!";
    }
    if (param.isCrossFeedingTermsUsed) {
        isCrossFeedingTerm = "Cross Feeding Term ON!";
    }
    ui.UAV2_control_option->setText(isAddonForce + ", " + isCrossFeedingTerm);

    ui.UAV2_kr ->setText("kr1 : " + QString::number(param.kr1_x) + ", " + QString::number(param.kr1_y) + ", " + QString::number(param.kr1_z) +
                        " kr2 : " + QString::number(param.kr2_x) + ", " + QString::number(param.kr2_y) + ", " + QString::number(param.kr2_z));
    ui.UAV2_lambda -> setText("lambda1 : " + QString::number(param.lambda1_x) + ", " + QString::number(param.lambda1_y) + ", " + QString::number(param.lambda1_z) +
                              "lambda2 : " + QString::number(param.lambda2_x) + ", " + QString::number(param.lambda2_y) + ", " + QString::number(param.lambda2_z));
    ui.UAV2_kp -> setText ("kp : " + QString::number(param.kp_x) + ", " + QString::number(param.kp_y) + ", " + QString::number(param.kp_z)+
                           "komega : " + QString::number(param.komega_x) + ", " + QString::number(param.komega_y) + ", " + QString::number(param.komega_z));

    ui.UAV2_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " + QString::number(param.kv_xy, 'f', 2) + ", " +  QString::number(param.kv_z, 'f', 2 ));
    ui.UAV2_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " + QString::number(param.kR_xy, 'f', 2) + ", " +  QString::number(param.kR_z, 'f', 2 ));

    // Lambda_j : ---- Lambda_p : -----  Lambda_r : ----
    ui.UAV2_UDE_lambda->setText("Laj : " + QString::number(param.lambdaj_x) + ", "
                                              + QString::number(param.lambdaj_y) + ", "
                                              + QString::number(param.lambdaj_z));

    ui.UAV2_kL ->setText("kL : " + QString::number(param.kL, 'f', 2));
    ui.UAV2_kphi->setText("kphi: " + QString::number(param.Kphi_xy, 'f', 2) + ", " + QString::number(param.Kphi_xy, 'f', 2) + ", " +  QString::number(param.Kphi_z, 'f', 2 ));
    ui.UAV2_p_error_max->setText("p_error_max: " + QString::number(param.pxy_error_max, 'f', 2) + ", " + QString::number(param.pxy_error_max, 'f', 2) + ", " +  QString::number(param.pz_error_max, 'f', 2 ));
    ui.UAV2_p_int_max->setText("p_int_max: " + QString::number(param.pxy_int_max, 'f', 2) + ", " + QString::number(param.pxy_int_max, 'f', 2) + ", " +  QString::number(param.pz_int_max, 'f', 2 ));
    ui.UAV2_tilt_max->setText("tilt_max : " + QString::number(param.tilt_max, 'f', 2));
    ui.UAV2_fp_max->setText("fp_max: " + QString::number(param.fp_max_x, 'f', 2) + ", " + QString::number(param.fp_max_y, 'f', 2) + ", " +  QString::number(param.fp_max_z, 'f', 2 ));
    ui.UAV2_int_start_error->setText("int_start_error : " + QString::number(param.int_start_error, 'f', 2));
}

void MainWindow::updatePayloadpos() {
    // TO DO: safty check for this part:
    bool ispayloaddetected = qnode.IsPayloadDetected();
    // check if the drone parameter is set to multi-drone mode

    if(ispayloaddetected) {
        ui.Payload_detection->setText("<font color='green'>Payload Mocap Detected!</font>");
        qt_ground_station::Mocap temp_mocap = qnode.GetMocap(-1);
        ui.Payload_x->setText(QString::number(temp_mocap.position[0], 'f', 3));
        ui.Payload_y->setText(QString::number(temp_mocap.position[1], 'f', 3));
        ui.Payload_z->setText(QString::number(temp_mocap.position[2], 'f', 3));

        ui.Payload_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 3));
        ui.Payload_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 3));
        ui.Payload_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 3));

        ui.Payload_omega_x->setText(QString::number(temp_mocap.angular_velocity[0]*57.3, 'f', 3));
        ui.Payload_omega_y->setText(QString::number(temp_mocap.angular_velocity[1]*57.3, 'f', 3));
        ui.Payload_omega_z->setText(QString::number(temp_mocap.angular_velocity[2]*57.3, 'f', 3));

        ui.action_payload_pos->setText("Payload X: " + QString::number(temp_mocap.position[0], 'f', 3) + "m, Y: "
                                       + QString::number(temp_mocap.position[1], 'f', 3) + "m,Z: "
                                       + QString::number(temp_mocap.position[2], 'f', 3) + "m" );

        Eigen::Quaterniond quaternion_temp;
        quaternion_temp.w() = temp_mocap.quaternion[0];
        quaternion_temp.x() = temp_mocap.quaternion[1];
        quaternion_temp.y() = temp_mocap.quaternion[2];
        quaternion_temp.z() = temp_mocap.quaternion[3];

        Eigen::Vector3d euler_temp =  quaternion_to_euler_w(quaternion_temp);
        ui.Payload_roll->setText(QString::number(euler_temp(0)*57.3, 'f', 3));
        ui.Payload_pitch->setText(QString::number(euler_temp(1)*57.3, 'f', 3));
        ui.Payload_yaw->setText(QString::number(euler_temp(2)*57.3, 'f', 3));

        /*----turn on button----------------*/

        // update the hovering place
        if(IsPayloadModeCorrect()&&IsMulti){// do the hover point and prelift update only if the mode is correct
            ui.Payload_Prelift->setEnabled(true);
            ui.Payload_Pose_Button->setEnabled(true);
            ui.Payload_Move_to_Start->setEnabled(true);        
            qt_ground_station::uav_para param = qnode.GetUAVPARA(0);
            int num_of_drones = param.num_drone;

            ui.Payload_Number_Drones->setText("Number of Drones: " + QString::number(num_of_drones));

            float hover_height   = ui.Payload_Hovering_Height->text().toFloat();
            float prelift_height = ui.Payload_Prelift_Height ->text().toFloat();


            Eigen::Vector3f pos_temp_hover   = qnode.UpdateHoverPosition(0,  hover_height);
            Eigen::Vector3f pos_temp_prelift = qnode.UpdateHoverPosition(0,  prelift_height);

            if(  hover_height >=1.2 ) {
                ui.UAV0_payload_hovering->setText("<font color='red'>UAV0 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                    + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                    + ", " + QString::number(pos_temp_hover(2), 'f', 2) + "</font>");
                if(num_of_drones>=2){
                    pos_temp_hover  = qnode.UpdateHoverPosition(1, hover_height);
                    ui.UAV1_payload_hovering->setText("<font color='red'>UAV1 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                                        + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                                        + ", " + QString::number(pos_temp_hover(2), 'f', 2)+ "</font>");
                }
                if(num_of_drones>=3) {
                    pos_temp_hover = qnode.UpdateHoverPosition(2, hover_height);
                    ui.UAV2_payload_hovering->setText("<font color='red'>UAV2 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                                        + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                                        + ", " + QString::number(pos_temp_hover(2), 'f', 2)+ "</font>");
                }

            } else {

                ui.UAV0_payload_hovering->setText("UAV0 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                    + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                    + ", " + QString::number(pos_temp_hover(2), 'f', 2));
                if(num_of_drones>=2){
                    pos_temp_hover = qnode.UpdateHoverPosition(1, hover_height);
                    ui.UAV1_payload_hovering->setText("UAV1 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_hover(2), 'f', 2));
                }
                if(num_of_drones>=3) {
                    pos_temp_hover = qnode.UpdateHoverPosition(2, hover_height);
                    ui.UAV2_payload_hovering->setText("UAV2 : " + QString::number(pos_temp_hover(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_hover(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_hover(2), 'f', 2));
                }
            }

            if( prelift_height >=1.4 ) {
                ui.UAV0_payload_prelift->setText("<font color='red'>UAV0 : " + QString::number(pos_temp_prelift(0), 'f', 2)
                                                    + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                    + ", " + QString::number(pos_temp_prelift(2), 'f', 2) + "</font>");
                if(num_of_drones>=2){
                    pos_temp_prelift = qnode.UpdateHoverPosition(1, prelift_height);
                    ui.UAV1_payload_prelift->setText("<font color='red'>UAV1 : " + QString::number(pos_temp_prelift(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(2), 'f', 2)+ "</font>");
                }
                if(num_of_drones>=3) {
                    pos_temp_prelift = qnode.UpdateHoverPosition(2, prelift_height);
                    ui.UAV2_payload_prelift->setText("<font color='red'>UAV2 : " + QString::number(pos_temp_prelift(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(2), 'f', 2)+ "</font>");
                }

            } else {

                ui.UAV0_payload_prelift->setText("UAV0 : "  + QString::number(pos_temp_prelift(0), 'f', 2)
                                                    + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                    + ", " + QString::number(pos_temp_prelift(2), 'f', 2));
                if(num_of_drones>=2){
                    pos_temp_prelift = qnode.UpdateHoverPosition(1, prelift_height);
                    ui.UAV1_payload_prelift->setText("UAV1 : "  + QString::number(pos_temp_prelift(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(2), 'f', 2));
                }
                if(num_of_drones>=3) {
                    pos_temp_prelift = qnode.UpdateHoverPosition(2, prelift_height);
                    ui.UAV2_payload_prelift->setText("UAV2 : " + QString::number(pos_temp_prelift(0), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(1), 'f', 2)
                                                        + ", " + QString::number(pos_temp_prelift(2), 'f', 2));
                }

            }
        }else{
            ui.Payload_Number_Drones->setText("<font color='red'>Incorrect mode, check num and isMulit!</font>");
            ui.Payload_Prelift->setEnabled(false);
            ui.Payload_Pose_Button->setEnabled(false);
            ui.Payload_Move_to_Start->setEnabled(false); 
        }


    } else {
        ui.Payload_detection->setText("<font color='red'>Payload Undetected!</font>");
        ui.Payload_x->setText("----");
        ui.Payload_y->setText("----");
        ui.Payload_z->setText("----");

        ui.Payload_vx->setText("----");
        ui.Payload_vy->setText("----");
        ui.Payload_vz->setText("----");

        ui.Payload_omega_x->setText("----");
        ui.Payload_omega_y->setText("----");
        ui.Payload_omega_z->setText("----");

        ui.Payload_roll->setText("----");
        ui.Payload_pitch->setText("----");
        ui.Payload_yaw->setText("----");
        /*----turn off button--------------*/
        ui.Payload_Prelift->setEnabled(false);
        ui.Payload_Pose_Button->setEnabled(false);
        ui.Payload_Move_to_Start->setEnabled(false);
    }

}


void MainWindow::updateUAV0attReference() {
    qt_ground_station::uav_log log = qnode.GetUAVLOG(0);

    ui.UAV0_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 4));
    ui.UAV0_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 4));
    ui.UAV0_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 4));
    ui.UAV0_thrust->setText(QString::number(log.Thrust_target, 'f', 4));
}

void MainWindow::updateUAV1attReference() {
    qt_ground_station::uav_log log = qnode.GetUAVLOG(1);

    ui.UAV1_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 4));
    ui.UAV1_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 4));
    ui.UAV1_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 4));
    ui.UAV1_thrust->setText(QString::number(log.Thrust_target, 'f', 4));
}

void MainWindow::updateUAV2attReference() {
    qt_ground_station::uav_log log = qnode.GetUAVLOG(2);

    ui.UAV2_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 4));
    ui.UAV2_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 4));
    ui.UAV2_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 4));
    ui.UAV2_thrust->setText(QString::number(log.Thrust_target, 'f', 4));
}


void MainWindow::updateUAV0log() {

    qt_ground_station::uav_log log = qnode.GetUAVLOG(0);

    if(log.isconnected) {
        ui.UAV0_detection->setText("<font color='green'>UAV0 Detected</font>");
    } else {
        ui.UAV0_detection->setText("<font color='red'>UAV0 Undetected!</font>");
     }

    if (log.log.Drone_State.connected && log.isconnected) {
        ui.UAV0_connection->setText("<font color='green'>CONNECTED</font>");
        ui.UAV0_Button_Disarm->setEnabled(true);
        //ui.UAV0_Button_Takeoff->setEnabled(true);
        //ui.UAV0_Button_Land->setEnabled(true);
        ui.UAV0_Button_moveENU->setEnabled(true);
        ui.UAV0_voltage->setText(GenerateBatteryInfo(log, 16.8, 14));

    } else {
        ui.UAV0_connection->setText("<font color='red'>UNCONNECTED</font>");
        /*------------disable all the buttons -------------------*/
        ui.UAV0_Button_Disarm->setEnabled(false);
        ui.UAV0_Button_Takeoff->setEnabled(false);
        ui.UAV0_Button_Land->setEnabled(false);
        ui.UAV0_Button_moveENU->setEnabled(false);
        ui.UAV0_voltage->setText("Voltage: --- V");
    }

    if (log.log.Drone_State.armed) {
        ui.UAV0_arm->setText("<font color='green'>ARMED</font>");    
    } else {
        ui.UAV0_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV0_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV0_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 4));
    ui.UAV0_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 4));
    ui.UAV0_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 4));
    /*----------------------------update command mode ---------------------------*/
    switch(log.log.Control_Command.Mode) {
    case Idle:
        ui.UAV0_commandmode->setText("Idle");
        break;

    case Takeoff:
        ui.UAV0_commandmode->setText("Take Off");
        break;

    case Move_ENU:
        ui.UAV0_commandmode->setText("Move ENU");
        break;

    case Move_Body:
        ui.UAV0_commandmode->setText("Move Body");
        break;

    case Hold:
        ui.UAV0_commandmode->setText("Hold");
        break;

    case Land:
        ui.UAV0_commandmode->setText("Land");
        break;
    case Payload_Stabilization_SingleUAV:
        ui.UAV0_commandmode->setText("Single UAV Payload");
        break;
    case Payload_Stabilization:
        ui.UAV0_commandmode->setText("Payload");
        break;

    case Payload_Land:
        ui.UAV0_commandmode->setText("Payload Land");
        break;

    case Disarm:
        ui.UAV0_commandmode->setText("Disarm");
        break;

    default:
        ui.UAV0_commandmode->setText("UNDEFINED MODE");
        break;

    }
    /*-------------------------update mocap feedback flag ----------------------------------*/
    if(log.log.Drone_State.mocapOK && log.isconnected) {
        ui.UAV0_mocapFlag->setText("<font color='green'>OptiTrack OK</font>");
    } else {
        ui.UAV0_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    }
    /*-------------------------update perform multi-drone action button and command state ----------------------------------*/
    if((log.log.Control_Command.Mode == Payload_Stabilization)
        &&log.isconnected&&log.log.Drone_State.armed){ // action feature is available when detected, armed, and in payload mode
        ui.Multi_action_command->setText("<font color='green'>In payload mode!</font>");
        ui.Multi_action->setEnabled(true);
    } else{
        ui.Multi_action_command->setText("<font color='red'>Not in payload mode!</font>");
        ui.Multi_action->setEnabled(false);
    }

}

void MainWindow::updateUAV1log() {

    qt_ground_station::uav_log log = qnode.GetUAVLOG(1);

    if(log.isconnected) {
        ui.UAV1_detection->setText("<font color='green'>UAV1 Detected</font>");
    } else {
        ui.UAV1_detection->setText("<font color='red'>UAV1 Undetected!</font>");
     }

    if (log.log.Drone_State.connected && log.isconnected) {
        ui.UAV1_connection->setText("<font color='green'>CONNECTED</font>");
        ui.UAV1_Button_Disarm->setEnabled(true);
        //ui.UAV1_Button_Takeoff->setEnabled(true);
        //ui.UAV1_Button_Land->setEnabled(true);
        ui.UAV1_Button_moveENU->setEnabled(true);
        ui.UAV1_voltage->setText(GenerateBatteryInfo(log, 16.8, 14));

    } else {
        ui.UAV1_connection->setText("<font color='red'>UNCONNECTED</font>");
        ui.UAV1_Button_Disarm->setEnabled(false);
        ui.UAV1_Button_Takeoff->setEnabled(false);
        ui.UAV1_Button_Land->setEnabled(false);
        ui.UAV1_Button_moveENU->setEnabled(false);
        ui.UAV1_voltage->setText("Voltage: --- V");
    }

    if (log.log.Drone_State.armed) {
        ui.UAV1_arm->setText("<font color='green'>ARMED</font>");
    } else {
        ui.UAV1_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV1_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV1_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 4));
    ui.UAV1_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 4));
    ui.UAV1_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 4));
    /*----------------------------update command mode ---------------------------*/
    switch(log.log.Control_Command.Mode) {
    case Idle:
        ui.UAV1_commandmode->setText("Idle");
        break;
    case Takeoff:
        ui.UAV1_commandmode->setText("Take Off");
        break;
    case Move_ENU:
        ui.UAV1_commandmode->setText("Move ENU");
        break;
    case Move_Body:
        ui.UAV1_commandmode->setText("Move Body");
        break;

    case Hold:
        ui.UAV1_commandmode->setText("Hold");
        break;

    case Land:
        ui.UAV1_commandmode->setText("Land");
        break;
    case Payload_Stabilization_SingleUAV:
        ui.UAV1_commandmode->setText("Single UAV Payload");
        break;
    case Payload_Stabilization:
        ui.UAV1_commandmode->setText("Payload");
        break;
    case Payload_Land:
        ui.UAV1_commandmode->setText("Payload Land");
        break;
    case Disarm:
        ui.UAV1_commandmode->setText("Disarm");
        break;
    default:
        ui.UAV1_commandmode->setText("UNDEFINED MODE");
        break;

    }
    /*-------------------------update mocap feedback flag ----------------------------------*/
    if(log.log.Drone_State.mocapOK && log.isconnected) {
        ui.UAV1_mocapFlag->setText("<font color='green'>OptiTrack OK</font>");
    } else {
        ui.UAV1_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    }

}

void MainWindow::updateUAV2log() {

    qt_ground_station::uav_log log = qnode.GetUAVLOG(2);

    if(log.isconnected) {
        ui.UAV2_detection->setText("<font color='green'>UAV2 Detected</font>");
    } else {
        ui.UAV2_detection->setText("<font color='red'>UAV2 Undetected!</font>");
     }

    if (log.log.Drone_State.connected && log.isconnected) {
        ui.UAV2_connection->setText("<font color='green'>CONNECTED</font>");
        ui.UAV2_Button_Disarm->setEnabled(true);
        //ui.UAV2_Button_Takeoff->setEnabled(true);
        //ui.UAV2_Button_Land->setEnabled(true);
        ui.UAV2_Button_moveENU->setEnabled(true);
        ui.UAV2_voltage->setText(GenerateBatteryInfo(log, 16.8, 14));
    } else {
        ui.UAV2_connection->setText("<font color='red'>UNCONNECTED</font>");
        ui.UAV2_Button_Disarm->setEnabled(false);
        ui.UAV2_Button_Takeoff->setEnabled(false);
        ui.UAV2_Button_Land->setEnabled(false);
        ui.UAV2_Button_moveENU->setEnabled(false);
        ui.UAV2_voltage->setText("Voltage: --- V");
    }

    if (log.log.Drone_State.armed) {
        ui.UAV2_arm->setText("<font color='green'>ARMED</font>");
    } else {
        ui.UAV2_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV2_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV2_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 4));
    ui.UAV2_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 4));
    ui.UAV2_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 4));
    /*----------------------------update command mode ---------------------------*/
    switch(log.log.Control_Command.Mode) {
    case Idle:
        ui.UAV2_commandmode->setText("Idle");
        break;

    case Takeoff:
        ui.UAV2_commandmode->setText("Take Off");
        break;

    case Move_ENU:
        ui.UAV2_commandmode->setText("Move ENU");
        break;

    case Move_Body:
        ui.UAV2_commandmode->setText("Move Body");
        break;

    case Hold:
        ui.UAV2_commandmode->setText("Hold");
        break;

    case Land:
        ui.UAV2_commandmode->setText("Land");
        break;
    case Payload_Stabilization_SingleUAV:
        ui.UAV2_commandmode->setText("Single UAV Payload");
        break;
    case Payload_Stabilization:
        ui.UAV2_commandmode->setText("Payload");
        break;

    case Payload_Land:
        ui.UAV2_commandmode->setText("Payload Land");
        break;
    case Disarm:
        ui.UAV2_commandmode->setText("Disarm");
        break;
    default:
        ui.UAV2_commandmode->setText("UNDEFINED MODE");
        break;

    }
    /*-------------------------update mocap feedback flag ----------------------------------*/
    if(log.log.Drone_State.mocapOK && log.isconnected) {
        ui.UAV2_mocapFlag->setText("<font color='green'>OptiTrack OK</font>");
    } else {
        ui.UAV2_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    }
    /*-------------------------update perform action button and command state ----------------------------------*/
    if((log.log.Control_Command.Mode == Payload_Stabilization_SingleUAV)
        &&log.isconnected&&log.log.Drone_State.armed){ // action feature is available when detected, armed, and in payload mode
        ui.action_command_state->setText("<font color='green'>In payload mode!</font>");
        ui.action_single->setEnabled(true);
    } else{
        ui.action_command_state->setText("<font color='red'>Not in payload mode!</font>");
        ui.action_single->setEnabled(false);
    }
}

QString MainWindow::GenerateBatteryInfo(qt_ground_station::uav_log& log, float Voltage_High, float Voltage_Low)
{
    QString TX;

    float percent = (log.log.Drone_State.battery_voltage - Voltage_Low)/(Voltage_High - Voltage_Low);

    if (percent > 0.7) {
        TX = "<font color='green'> Voltage: "  + QString::number(log.log.Drone_State.battery_voltage,'f', 1) + "V </font>";
    } else if (percent > 0.6) {
        TX = "<font color='brown'> Voltage: " + QString::number(log.log.Drone_State.battery_voltage,'f', 1) + "V </font>";
    } else {
        TX = "<font color='red'> Voltage: "    + QString::number(log.log.Drone_State.battery_voltage,'f', 1) + "V </font>";
    }

    return TX;
}

void MainWindow::updateMode(){
    if(IsOutDoor){
        ui.displaymode->setText("Outdoor Mode");
        ui.Toggledisplaymode_Button->setText(" Switch To Indoor Mode");
    } else {
        ui.displaymode->setText("Indoor Mode");
        ui.Toggledisplaymode_Button->setText("Switch To Outdoor Mode");
    }
    if(IsMulti){
        ui.controlmode->setText("MultiDrone Mode");
        ui.Togglecontrolmode_Button->setText("Switch To Single-Drone Mode");
    } else {
        ui.controlmode->setText("SingleDrone Mode");
        ui.Togglecontrolmode_Button->setText("Switch To Multi-Drone Mode");
    }
}

Eigen::Vector3d MainWindow::quaternion_to_euler_w(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void MainWindow::DisplayENUErrorMsg(qt_ground_station::ENUCommandError error_msg)
{
    QMessageBox msgBox;
    QString msgbody;
    switch (error_msg) 
    {
        case qt_ground_station::DRONE_COMMAND_TOOCLOSETOOTHER:
        {
            msgbody = "The command position is too close to the other drones! ";
            break;
        }
        case qt_ground_station::DRONE_COMMAND_TOOCLOSETOOTHERCOMMAND:
        {
            msgbody = "The command position is too close to the command position to the other drones!";
            break;
        } 
        case qt_ground_station::DRONE_COMMAND_OUTOFBOUND:
        {
            msgbody = "The command position is out of the safty bound ";
            break;
        } 
    }
    msgBox.setText(msgbody);
    msgBox.exec();
}

 void MainWindow::UpdateUAVPos(Eigen::Vector3f& Position, Eigen::Vector3f& Velocity, int ID){
    if(IsOutDoor){
        qt_ground_station::Topic_for_log templog = qnode.GetLog(ID);
        Position(0) = templog.Drone_State.position[0];
        Position(1) = templog.Drone_State.position[1];
        Position(2) = templog.Drone_State.position[2];

        Velocity(0) = templog.Drone_State.velocity[0];
        Velocity(1) = templog.Drone_State.velocity[1];
        Velocity(2) = templog.Drone_State.velocity[2];
    } else {
        qt_ground_station::Mocap temp_mocap = qnode.GetMocap(ID);
        Position(0) = temp_mocap.position[0];
        Position(1) = temp_mocap.position[1];
        Position(2) = temp_mocap.position[2];

        Velocity(0) = temp_mocap.velocity[0];
        Velocity(1) = temp_mocap.velocity[1];
        Velocity(2) = temp_mocap.velocity[2];
    }
 }

void MainWindow::UpdateListViewENU(int drone_ID,float target_state[4])
{
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "moveENU sent to #" + QString::number(drone_ID) + " drone.";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::blue);
    QString position = " X: " + QString::number(target_state[0]) 
                     + " m , Y: " + QString::number(target_state[1])
                     + " m , Z: " + QString::number(target_state[2]) + " m. ";
    ui.logger1->addItem(position);
    ui.logger1->scrollToBottom();
}
void MainWindow::UpdateListViewMultiPayload(float pose_target[6]){

    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "payload control.";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::magenta);
    QString position = " X: " + QString::number(pose_target[0]) 
                     + " m , Y: " + QString::number(pose_target[1])
                     + " m , Z: " + QString::number(pose_target[2]) + " m. ";
    ui.logger1->addItem(position);
    QString attitude = " R: " + QString::number(pose_target[3]) 
                     + " DG , P: " + QString::number(pose_target[4])
                     + " DG , Y: " + QString::number(pose_target[5]) + " DG. ";
    ui.logger1->addItem(attitude);
    ui.logger1->scrollToBottom();
}



void MainWindow::UpdateListViewMoveToHoverPoint(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "move to hover point sent:";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::darkGreen);
    float height = ui.Payload_Hovering_Height->text().toFloat();
    Eigen::Vector3f temp_position;
    temp_position = qnode.UpdateHoverPosition(0,  height);    
    QString position_0 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_0);
    temp_position = qnode.UpdateHoverPosition(1,  height);    
    QString position_1 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_1);
    temp_position = qnode.UpdateHoverPosition(2,  height);    
    QString position_2 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_2);
    ui.logger1->scrollToBottom();                 
             
}

void MainWindow::UpdateListViewPrelift(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "prelift sent:";
    ui.logger1->addItem(msgdrone);
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::darkGreen);
    float height = ui.Payload_Prelift_Height ->text().toFloat();
    Eigen::Vector3f temp_position;
    temp_position = qnode.UpdateHoverPosition(0,  height);    
    QString position_0 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_0);
    temp_position = qnode.UpdateHoverPosition(1,  height);    
    QString position_1 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_1);
    temp_position = qnode.UpdateHoverPosition(2,  height);    
    QString position_2 = " X: " + QString::number(temp_position [0]) 
                     + " m , Y: " + QString::number(temp_position [1])
                     + " m , Z: " + QString::number(temp_position [2]) + " m. ";
    ui.logger1->addItem(position_2);
    ui.logger1->scrollToBottom();
}

void MainWindow::UpdataListViewLand(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "payload land sent.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::red);
    ui.logger1->scrollToBottom();    
}

void MainWindow::UpdateListViewSingleAction() {
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "single action sent.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::darkMagenta);
    ui.logger1->scrollToBottom(); 
}

void  MainWindow::UpdataListViewMultiAction(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + " multi-drone action sent.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::darkMagenta);
    ui.logger1->scrollToBottom();     
}

void MainWindow::UpdateListViewMultiStopAction(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "stop multi-drone action.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::red);
    ui.logger1->scrollToBottom(); 
}

void MainWindow::UpdateListViewSingleStopAction(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "stop single action.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::red);
    ui.logger1->scrollToBottom(); 
}
void MainWindow::UpdateBacktoENU(){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "back to ENU.";
    ui.logger1->addItem(msgdrone);                  
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::darkYellow);
    ui.logger1->scrollToBottom(); 
}

void MainWindow::UpdateSwitchToSinglePayloadMode(float pose_target[4]){
    QString msgdrone = "@ " + QTime::currentTime().toString() 
                        + " : " + "Single Drone Payload.";
    ui.logger1->addItem(msgdrone);      
    int item_index = ui.logger1->count()- 1;
    ui.logger1->item(item_index)->setForeground(Qt::magenta);
    QString position = " X: " + QString::number(pose_target[0]) 
                     + " m , Y: " + QString::number(pose_target[1])
                     + " m , Z: " + QString::number(pose_target[2]) + " m. ";
    ui.logger1->addItem(position);
    ui.logger1->scrollToBottom(); 
}


bool MainWindow::IsPayloadModeCorrect(){
    // logic: if multi_flag is true, then use para of drone #0 to determine whether the num_of_drones and mode are correct
    // if multi_flag is false, then use para of drone #2 and check check its mode, disable prelift and move to hover point 
    bool isCorrect = true;
    if(IsMulti) {
        bool isNumofDronesCorrect = qnode.isNumberofDronesConsistent();
        bool isCooperativeModeCorrecrt = qnode.isCooperativeModeConsistent();
        isCorrect = isNumofDronesCorrect&&isCooperativeModeCorrecrt;
    }else{
        isCorrect = false;
    }
    return isCorrect;
}

}
// namespace qt_ground_station

