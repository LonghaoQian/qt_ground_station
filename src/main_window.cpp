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

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV0mocap()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV1mocap()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV2mocap()));

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV0attReference()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV1attReference()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateUAV2attReference()));

    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updatePayloadmocap()));

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
    bool input_is_valid = true;

    if(target_state[0]<-1.5 || target_state[0]> 1.3) {
        input_is_valid = false;
    }

    if(target_state[1]< -1 || target_state[1]> 1) {
        input_is_valid = false;
    }

    if(target_state[2]< 0|| target_state[2]> 1.8) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.UAV0_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV0_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV0_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/

        qnode.move_ENU(0,target_state);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
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
    bool input_is_valid = true;

    if(target_state[0]<-1.5 || target_state[0]> 1.3) {
        input_is_valid = false;
    }

    if(target_state[1]< -1 || target_state[1]> 1) {
        input_is_valid = false;
    }

    if(target_state[2]< 0|| target_state[2]> 1.8) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.UAV1_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV1_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV1_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/

        qnode.move_ENU(1,target_state);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
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
    bool input_is_valid = true;

    if(target_state[0]<-1.5 || target_state[0]> 1.3) {
        input_is_valid = false;
    }

    if(target_state[1]< -1 || target_state[1]> 1) {
        input_is_valid = false;
    }

    if(target_state[2]< 0|| target_state[2]> 1.8) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.UAV2_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
        ui.UAV2_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
        ui.UAV2_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
        /*-------------------- set move ENU to node --------------------------------*/

        qnode.move_ENU(2,target_state);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
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

void MainWindow::on_Payload_Activate_Button_clicked(bool check) {
    // this turns on the button for payload stabilization command and turn off individual command

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
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Payload_Land_Button_clicked(bool check) {
      qnode.payload_land();
}

void MainWindow::on_Flush_MoveENU_Button_clicked(bool check) {
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


    }
}

void MainWindow::on_Payload_Prelift_clicked(bool check){
    // move to lift payload
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
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
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
void MainWindow::updateUAV0mocap() {
    qt_ground_station::Mocap temp_mocap = qnode.GetMocap(0);
    ui.UAV0_x->setText(QString::number(temp_mocap.position[0], 'f', 2));
    ui.UAV0_y->setText(QString::number(temp_mocap.position[1], 'f', 2));
    ui.UAV0_z->setText(QString::number(temp_mocap.position[2], 'f', 2));

    ui.UAV0_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 2));
    ui.UAV0_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 2));
    ui.UAV0_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 2));

}

void MainWindow::updateUAV1mocap() {
    qt_ground_station::Mocap temp_mocap = qnode.GetMocap(1);
    ui.UAV1_x->setText(QString::number(temp_mocap.position[0], 'f', 2));
    ui.UAV1_y->setText(QString::number(temp_mocap.position[1], 'f', 2));
    ui.UAV1_z->setText(QString::number(temp_mocap.position[2], 'f', 2));

    ui.UAV1_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 2));
    ui.UAV1_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 2));
    ui.UAV1_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 2));

}

void MainWindow::updateUAV2mocap() {
    qt_ground_station::Mocap temp_mocap = qnode.GetMocap(2);
    ui.UAV2_x->setText(QString::number(temp_mocap.position[0], 'f', 2));
    ui.UAV2_y->setText(QString::number(temp_mocap.position[1], 'f', 2));
    ui.UAV2_z->setText(QString::number(temp_mocap.position[2], 'f', 2));

    ui.UAV2_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 2));
    ui.UAV2_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 2));
    ui.UAV2_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 2));

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
    ui.UAV0_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " + QString::number(param.kv_xy, 'f', 2) + ", " +  QString::number(param.kv_z, 'f', 2 ));
    ui.UAV0_kvi->setText("kvi: " + QString::number(param.kvi_xy ,'f', 2) + ", " + QString::number(param.kvi_xy, 'f', 2) + ", " +  QString::number(param.kvi_z, 'f', 2 ));
    ui.UAV0_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " + QString::number(param.kR_xy, 'f', 2) + ", " +  QString::number(param.kR_z, 'f', 2 ));
    ui.UAV0_kL ->setText("kL : " + QString::number(param.kL, 'f', 2));
    ui.UAV0_kphi->setText("kphi: " + QString::number(param.Kphi_xy, 'f', 2) + ", " + QString::number(param.Kphi_xy, 'f', 2) + ", " +  QString::number(param.Kphi_z, 'f', 2 ));
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
    ui.UAV1_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " + QString::number(param.kv_xy, 'f', 2) + ", " +  QString::number(param.kv_z, 'f', 2 ));
    ui.UAV1_kvi->setText("kvi: " + QString::number(param.kvi_xy ,'f', 2) + ", " + QString::number(param.kvi_xy, 'f', 2) + ", " +  QString::number(param.kvi_z, 'f', 2 ));
    ui.UAV1_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " + QString::number(param.kR_xy, 'f', 2) + ", " +  QString::number(param.kR_z, 'f', 2 ));
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
    ui.UAV2_kv ->setText("kv: " + QString::number(param.kv_xy, 'f', 2) + ", " + QString::number(param.kv_xy, 'f', 2) + ", " +  QString::number(param.kv_z, 'f', 2 ));
    ui.UAV2_kvi->setText("kvi: " + QString::number(param.kvi_xy ,'f', 2) + ", " + QString::number(param.kvi_xy, 'f', 2) + ", " +  QString::number(param.kvi_z, 'f', 2 ));
    ui.UAV2_kR ->setText("kR: " + QString::number(param.kR_xy, 'f', 2) + ", " + QString::number(param.kR_xy, 'f', 2) + ", " +  QString::number(param.kR_z, 'f', 2 ));
    ui.UAV2_kL ->setText("kL : " + QString::number(param.kL, 'f', 2));
    ui.UAV2_kphi->setText("kphi: " + QString::number(param.Kphi_xy, 'f', 2) + ", " + QString::number(param.Kphi_xy, 'f', 2) + ", " +  QString::number(param.Kphi_z, 'f', 2 ));
    ui.UAV2_p_error_max->setText("p_error_max: " + QString::number(param.pxy_error_max, 'f', 2) + ", " + QString::number(param.pxy_error_max, 'f', 2) + ", " +  QString::number(param.pz_error_max, 'f', 2 ));
    ui.UAV2_p_int_max->setText("p_int_max: " + QString::number(param.pxy_int_max, 'f', 2) + ", " + QString::number(param.pxy_int_max, 'f', 2) + ", " +  QString::number(param.pz_int_max, 'f', 2 ));
    ui.UAV2_tilt_max->setText("tilt_max : " + QString::number(param.tilt_max, 'f', 2));
    ui.UAV2_fp_max->setText("fp_max: " + QString::number(param.fp_max_x, 'f', 2) + ", " + QString::number(param.fp_max_y, 'f', 2) + ", " +  QString::number(param.fp_max_z, 'f', 2 ));
    ui.UAV2_int_start_error->setText("int_start_error : " + QString::number(param.int_start_error, 'f', 2));
}

void MainWindow::updatePayloadmocap() {

    bool ispayloaddetected = qnode.IsPayloadDetected();

    if(ispayloaddetected) {
        ui.Payload_detection->setText("<font color='green'>Payload Mocap Detected!</font>");
        qt_ground_station::Mocap temp_mocap = qnode.GetMocap(-1);
        ui.Payload_x->setText(QString::number(temp_mocap.position[0], 'f', 2));
        ui.Payload_y->setText(QString::number(temp_mocap.position[1], 'f', 2));
        ui.Payload_z->setText(QString::number(temp_mocap.position[2], 'f', 2));

        ui.Payload_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 2));
        ui.Payload_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 2));
        ui.Payload_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 2));

        ui.Payload_omega_x->setText(QString::number(temp_mocap.angular_velocity[0]*57.3, 'f', 2));
        ui.Payload_omega_y->setText(QString::number(temp_mocap.angular_velocity[1]*57.3, 'f', 2));
        ui.Payload_omega_z->setText(QString::number(temp_mocap.angular_velocity[2]*57.3, 'f', 2));

        Eigen::Quaterniond quaternion_temp;
        quaternion_temp.w() = temp_mocap.quaternion[0];
        quaternion_temp.x() = temp_mocap.quaternion[1];
        quaternion_temp.y() = temp_mocap.quaternion[2];
        quaternion_temp.z() = temp_mocap.quaternion[3];

        Eigen::Vector3d euler_temp =  quaternion_to_euler_w(quaternion_temp);
        ui.Payload_roll->setText(QString::number(euler_temp(0)*57.3, 'f', 2));
        ui.Payload_pitch->setText(QString::number(euler_temp(1)*57.3, 'f', 2));
        ui.Payload_yaw->setText(QString::number(euler_temp(2)*57.3, 'f', 2));

        /*----turn on button----------------*/
        ui.Payload_Prelift->setEnabled(true);
        ui.Payload_Pose_Button->setEnabled(true);
        ui.Payload_Move_to_Start->setEnabled(true);
        // update the hovering place

        qt_ground_station::uav_para param = qnode.GetUAVPARA(0);
        int num_of_drones = param.num_drone;

        float height = ui.Payload_Hovering_Height->text().toFloat();

        Eigen::Vector3f pos_temp = qnode.UpdateHoverPosition(0,  height);

        if( height>=1.2 ) {
            ui.UAV0_payload_hovering->setText("<font color='red'>UAV0 : " + QString::number(pos_temp(0), 'f', 2)
                                                 + ", " + QString::number(pos_temp(1), 'f', 2)
                                                 + ", " + QString::number(pos_temp(2), 'f', 2) + "</font>");
            if(num_of_drones>=2){
                pos_temp = qnode.UpdateHoverPosition(1, 0.7);
                ui.UAV1_payload_hovering->setText("<font color='red'>UAV1 : " + QString::number(pos_temp(0), 'f', 2)
                                                     + ", " + QString::number(pos_temp(1), 'f', 2)
                                                     + ", " + QString::number(pos_temp(2), 'f', 2)+ "</font>");
            }
            if(num_of_drones>=3) {
                pos_temp = qnode.UpdateHoverPosition(2, 0.7);
                ui.UAV2_payload_hovering->setText("<font color='red'>UAV2 : " + QString::number(pos_temp(0), 'f', 2)
                                                     + ", " + QString::number(pos_temp(1), 'f', 2)
                                                     + ", " + QString::number(pos_temp(2), 'f', 2)+ "</font>");
            }

        } else {

            ui.UAV0_payload_hovering->setText("UAV0 : " + QString::number(pos_temp(0), 'f', 2)
                                                 + ", " + QString::number(pos_temp(1), 'f', 2)
                                                 + ", " + QString::number(pos_temp(2), 'f', 2));
            if(num_of_drones>=2){
                pos_temp = qnode.UpdateHoverPosition(1, 0.7);
                ui.UAV1_payload_hovering->setText("UAV1 : " + QString::number(pos_temp(0), 'f', 2)
                                                     + ", " + QString::number(pos_temp(1), 'f', 2)
                                                     + ", " + QString::number(pos_temp(2), 'f', 2));
            }
            if(num_of_drones>=3) {
                pos_temp = qnode.UpdateHoverPosition(2, 0.7);
                ui.UAV2_payload_hovering->setText("UAV2 : " + QString::number(pos_temp(0), 'f', 2)
                                                     + ", " + QString::number(pos_temp(1), 'f', 2)
                                                     + ", " + QString::number(pos_temp(2), 'f', 2));
            }

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

    ui.UAV0_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 2));
    ui.UAV0_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 2));
    ui.UAV0_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 2));
    ui.UAV0_thrust->setText(QString::number(log.Thrust_target, 'f', 2));
}

void MainWindow::updateUAV1attReference() {
    qt_ground_station::uav_log log = qnode.GetUAVLOG(1);

    ui.UAV1_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 2));
    ui.UAV1_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 2));
    ui.UAV1_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 2));
    ui.UAV1_thrust->setText(QString::number(log.Thrust_target, 'f', 2));
}

void MainWindow::updateUAV2attReference() {
    qt_ground_station::uav_log log = qnode.GetUAVLOG(2);

    ui.UAV2_att_roll->setText(QString::number(log.euler_fcu_target(0)*57.3, 'f', 2));
    ui.UAV2_att_pitch->setText(QString::number(log.euler_fcu_target(1)*57.3, 'f', 2));
    ui.UAV2_att_yaw->setText(QString::number(log.euler_fcu_target(2)*57.3, 'f', 2));
    ui.UAV2_thrust->setText(QString::number(log.Thrust_target, 'f', 2));
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
        ui.UAV0_Button_Takeoff->setEnabled(true);
        ui.UAV0_Button_Land->setEnabled(true);
        ui.UAV0_Button_moveENU->setEnabled(true);

    } else {
        ui.UAV0_connection->setText("<font color='red'>UNCONNECTED</font>");
        /*------------disable all the buttons -------------------*/
        ui.UAV0_Button_Disarm->setEnabled(false);
        ui.UAV0_Button_Takeoff->setEnabled(false);
        ui.UAV0_Button_Land->setEnabled(false);
        ui.UAV0_Button_moveENU->setEnabled(false);
    }

    if (log.log.Drone_State.armed) {
        ui.UAV0_arm->setText("<font color='green'>ARMED</font>");    
    } else {
        ui.UAV0_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV0_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV0_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 2));
    ui.UAV0_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 2));
    ui.UAV0_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 2));
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
        ui.UAV1_Button_Takeoff->setEnabled(true);
        ui.UAV1_Button_Land->setEnabled(true);
        ui.UAV1_Button_moveENU->setEnabled(true);

    } else {
        ui.UAV1_connection->setText("<font color='red'>UNCONNECTED</font>");
        ui.UAV1_Button_Disarm->setEnabled(false);
        ui.UAV1_Button_Takeoff->setEnabled(false);
        ui.UAV1_Button_Land->setEnabled(false);
        ui.UAV1_Button_moveENU->setEnabled(false);
    }

    if (log.log.Drone_State.armed) {
        ui.UAV1_arm->setText("<font color='green'>ARMED</font>");
    } else {
        ui.UAV1_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV1_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV1_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 2));
    ui.UAV1_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 2));
    ui.UAV1_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 2));
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
        ui.UAV2_Button_Takeoff->setEnabled(true);
        ui.UAV2_Button_Land->setEnabled(true);
        ui.UAV2_Button_moveENU->setEnabled(true);
    } else {
        ui.UAV2_connection->setText("<font color='red'>UNCONNECTED</font>");
        ui.UAV2_Button_Disarm->setEnabled(false);
        ui.UAV2_Button_Takeoff->setEnabled(false);
        ui.UAV2_Button_Land->setEnabled(false);
        ui.UAV2_Button_moveENU->setEnabled(false);
    }

    if (log.log.Drone_State.armed) {
        ui.UAV2_arm->setText("<font color='green'>ARMED</font>");
    } else {
        ui.UAV2_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV2_mode->setText(QString::fromStdString(log.log.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV2_Tx->setText(QString::number(log.log.Control_Output.Throttle[0], 'f', 2));
    ui.UAV2_Ty->setText(QString::number(log.log.Control_Output.Throttle[1], 'f', 2));
    ui.UAV2_Tz->setText(QString::number(log.log.Control_Output.Throttle[2], 'f', 2));
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

}  // namespace qt_ground_station

