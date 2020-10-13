/**
 * @file /include/qt_ground_station/main_window.hpp
 *
 * @brief Qt based gui for qt_ground_station.
 *
 * @date November 2010
 **/
#ifndef qt_ground_station_MAIN_WINDOW_H
#define qt_ground_station_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_ground_station {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

        void on_Button_DisarmALL_clicked(bool check);

        void on_UAV0_Button_Takeoff_clicked(bool check);
        void on_UAV0_Button_Land_clicked(bool check);
        void on_UAV0_Button_moveENU_clicked(bool check);
        void on_UAV0_Button_Disarm_clicked(bool check);
        void on_UAV0_Copypos_clicked(bool check);

        void on_UAV1_Button_Takeoff_clicked(bool check);
        void on_UAV1_Button_Land_clicked(bool check);
        void on_UAV1_Button_moveENU_clicked(bool check);
        void on_UAV1_Button_Disarm_clicked(bool check);
        void on_UAV1_Copypos_clicked(bool check);

        void on_UAV2_Button_Takeoff_clicked(bool check);
        void on_UAV2_Button_Land_clicked(bool check);
        void on_UAV2_Button_moveENU_clicked(bool check);
        void on_UAV2_Button_Disarm_clicked(bool check);
        void on_UAV2_Copypos_clicked(bool check);
        void on_UAV2_Move_with_payload_clicked(bool check);
        void on_UAV2_Back_to_ENU_clicked(bool check);

        void on_Flush_MoveENU_Button_clicked(bool check);

        void on_Payload_Pose_Button_clicked(bool check);
        void on_Payload_Land_Button_clicked(bool check);
        void on_Payload_Move_to_Start_clicked(bool check);
        void on_Payload_Prelift_clicked(bool check);

        void on_Toggledisplaymode_Button_clicked(bool check);
        void on_Togglecontrolmode_Button_clicked(bool check);
        void on_ClearLog_Button_clicked(bool check);
        // action panel:
        void on_action_single_clicked(bool check);
        void on_stop_action_single_clicked(bool check);
        void on_UAV2_Back_to_ENU_1_clicked(bool check);
        // multi-drone action
        void on_Multi_action_clicked(bool check);
        void on_Multi_stopaction_clicked(bool check);
    /******************************************
    ** Manual connections
    *******************************************/

    void updateUAV0pos();// action perfomed when recieving mocap data
    void updateUAV1pos();
    void updateUAV2pos();
    void updatePayloadpos();

    void updateUAV0attReference(); // // action perfomed when recieving attitude reference
    void updateUAV1attReference();
    void updateUAV2attReference();

    void updateUAV0log();
    void updateUAV1log();
    void updateUAV2log(); // action perfomed when recieving log data from drones

    void updateUAV0Param(); // action performed when recieving parameter info from drones
    void updateUAV1Param();
    void updateUAV2Param();
    Eigen::Vector3d  quaternion_to_euler_w(const Eigen::Quaterniond &q);
    QString GenerateBatteryInfo(qt_ground_station::uav_log& log, float Voltage_High, float Voltage_Low);
    void updateMode();
Q_SIGNALS:
    void send_TakeoffUAV0();
    void send_TakeoffUAV1();
    void send_TakeoffUAV2();
    void send_LandUAV0();
    void send_LandUAV1();
    void send_LandUAV2();
    void send_MoveENUUAV0();
    void send_MoveENUUAV1();
    void send_MoveENUUAV2();
    void send_DisarmUAV0();
    void send_DisarmUAV1();
    void send_DisarmUAV2();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    bool IsOutDoor{false}; // display mode
    bool IsMulti{true};
    qt_ground_station::CommandGeoFence DroneFence;
    qt_ground_station::CommandGeoFence PayloadFence;
    void UpdateUAVPos(Eigen::Vector3f& Position, Eigen::Vector3f& Velocity, int ID);
    void DisplayENUErrorMsg(qt_ground_station::ENUCommandError error_msg);
    void UpdateListViewENU(int rone_ID,float target_state[4]);
    void UpdateListViewMultiPayload(float pose_target[6]);
    void UpdateListViewMoveToHoverPoint();
    void UpdateListViewPrelift();
    void UpdataListViewLand();
    void UpdateListViewSingleAction();
    void UpdataListViewMultiAction();
    void UpdateListViewSingleStopAction();
    void UpdateListViewMultiStopAction();
    void UpdateBacktoENU();
    void UpdateSwitchToSinglePayloadMode(float pose_target[4]);
    // TO DO: std::map<qt_ground_station::ENUCommandError, QString> error_msg_list = 
    bool IsPayloadModeCorrect();
};

}  // namespace qt_ground_station

#endif // qt_ground_station_MAIN_WINDOW_H
