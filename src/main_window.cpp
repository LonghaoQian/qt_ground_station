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
    QObject::connect(&qnode, SIGNAL(UAV0_LogFromDrone_label()), this, SLOT(updateUAV0log()));
    QObject::connect(&qnode, SIGNAL(mocapUAV0_label()), this, SLOT(updateUAV0mocap()));
    QObject::connect(&qnode, SIGNAL(attReferenceUAV0_lable()), this, SLOT(updateUAV0attReference()));
    /* -----------------------------update labels --------------------------------*/
    ui.UAV0_connection->setText("<font color='red'>UNCONNECTED</font>");
    ui.UAV0_arm->setText("<font color='red'>DISARMED</font>");
    ui.UAV0_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
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

void MainWindow::on_Button_Takeoff_clicked(bool check ) {

}

void MainWindow::on_Button_moveENU_clicked(bool check){
    /* read values from line edit */
    float target_state[4];

    target_state[0] =  ui.UAV0_Target_x->text().toFloat();
    target_state[1] =  ui.UAV0_Target_y->text().toFloat();
    target_state[2] =  ui.UAV0_Target_z->text().toFloat();
    target_state[3] = 0;
    /*  update the ENU target label */
    ui.UAV0_Target_x_label->setText(QString::number(target_state[0], 'f', 2));
    ui.UAV0_Target_y_label->setText(QString::number(target_state[1], 'f', 2));
    ui.UAV0_Target_z_label->setText(QString::number(target_state[2], 'f', 2));
    /*-------------------- set move ENU to node --------------------------------*/
    qnode.move_ENU_UAV0(target_state);
}

void MainWindow::on_Button_Land_clicked(bool check) {


}
//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
        //bool enabled;
        //if ( state == 0 ) {
        //	enabled = true;
        //} else {
        //	enabled = false;
        //}
        //ui.line_edit_master->setEnabled(enabled);
        //ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
//}

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

/******************************SLOT************************************/
void MainWindow::updateUAV0mocap() {
    qt_ground_station::Mocap temp_mocap = qnode.GetMocapUAV0();
    ui.UAV0_x->setText(QString::number(temp_mocap.position[0], 'f', 2));
    ui.UAV0_y->setText(QString::number(temp_mocap.position[1], 'f', 2));
    ui.UAV0_z->setText(QString::number(temp_mocap.position[2], 'f', 2));

    ui.UAV0_vx->setText(QString::number(temp_mocap.velocity[0], 'f', 2));
    ui.UAV0_vy->setText(QString::number(temp_mocap.velocity[1], 'f', 2));
    ui.UAV0_vz->setText(QString::number(temp_mocap.velocity[2], 'f', 2));

}

void MainWindow::updateUAV0attReference() {
    Eigen::Vector4d command = qnode.GetAttThrustCommandUAV0();

    ui.UAV0_att_roll->setText(QString::number(command(0)*57.3, 'f', 2));
    ui.UAV0_att_pitch->setText(QString::number(command(1)*57.3, 'f', 2));
    ui.UAV0_att_yaw->setText(QString::number(command(2)*57.3, 'f', 2));
    ui.UAV0_thrust->setText(QString::number(command(3), 'f', 2));
}

void MainWindow::updateUAV0log() {

    qt_ground_station::Topic_for_log topic = qnode.GetDroneStateUAV0();

    if (topic.Drone_State.connected) {
        ui.UAV0_connection->setText("<font color='green'>CONNECTED</font>");
    } else {
        ui.UAV0_connection->setText("<font color='red'>UNCONNECTED</font>");
    }

    if (topic.Drone_State.armed) {
        ui.UAV0_arm->setText("<font color='green'>ARMED</font>");
    } else {
        ui.UAV0_arm->setText("<font color='red'>DISARMED</font>");
    }
    ui.UAV0_mode->setText(QString::fromStdString(topic.Drone_State.mode));
    /*-------------------------- update command thrust --------------------------*/
    ui.UAV0_Tx->setText(QString::number(topic.Control_Output.Throttle[0], 'f', 2));
    ui.UAV0_Ty->setText(QString::number(topic.Control_Output.Throttle[1], 'f', 2));
    ui.UAV0_Tz->setText(QString::number(topic.Control_Output.Throttle[2], 'f', 2));
    /*----------------------------update command mode ---------------------------*/
    switch(topic.Control_Command.Mode) {
    case QNode::Idle:
        ui.UAV0_commandmode->setText("Idle");
        break;

    case QNode::Takeoff:
        ui.UAV0_commandmode->setText("Take Off");
        break;

    case QNode::Move_ENU:
        ui.UAV0_commandmode->setText("Move ENU");
        break;

    case QNode::Move_Body:
        ui.UAV0_commandmode->setText("Move Body");
        break;

    case QNode::Hold:
        ui.UAV0_commandmode->setText("Hold");
        break;

    case QNode::Land:
        ui.UAV0_commandmode->setText("Land");
        break;

    case QNode::Disarm:
        ui.UAV0_commandmode->setText("Disarm");
        break;
    default:
        ui.UAV0_commandmode->setText("UNDEFINED MODE");
        break;

    }
    /*-------------------------update mocap feedback flag ----------------------------------*/
    if(topic.Drone_State.mocapOK) {
        ui.UAV0_mocapFlag->setText("<font color='green'>OptiTrack OK</font>");
    } else {
        ui.UAV0_mocapFlag->setText("<font color='red'>No OptiTrack Feedback!!</font>");
    }
}

}  // namespace qt_ground_station

