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

        void on_UAV1_Button_Takeoff_clicked(bool check);
        void on_UAV1_Button_Land_clicked(bool check);
        void on_UAV1_Button_moveENU_clicked(bool check);
        void on_UAV1_Button_Disarm_clicked(bool check);

        void on_UAV2_Button_Takeoff_clicked(bool check);
        void on_UAV2_Button_Land_clicked(bool check);
        void on_UAV2_Button_moveENU_clicked(bool check);
        void on_UAV2_Button_Disarm_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void updateUAV0mocap();
    void updateUAV1mocap();
    void updateUAV2mocap();
    void updatePayloadmocap();

    void updateUAV0attReference();
    void updateUAV1attReference();
    void updateUAV2attReference();

    void updateUAV0log();
    void updateUAV1log();
    void updateUAV2log();

    Eigen::Vector3d  quaternion_to_euler_w(const Eigen::Quaterniond &q);

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
};

}  // namespace qt_ground_station

#endif // qt_ground_station_MAIN_WINDOW_H
