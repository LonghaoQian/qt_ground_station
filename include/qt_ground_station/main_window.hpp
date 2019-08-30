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
        void on_Button_Takeoff_clicked(bool check);
        void on_Button_Land_clicked(bool check);
        void on_Button_moveENU_clicked(bool check);
        //void on_checkbox_use_environment_stateChanged(int state);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateUAV0mocap();
    void updateUAV0attReference();
    void updateUAV0log();


Q_SIGNALS:
    void send_TakeoffUAV0();
    void send_TakeoffUAV1();
    void send_TakeoffUAV2();
    void send_TakeoffUAV3();
    void send_LandUAV0();
    void send_LandUAV1();
    void send_LandUAV2();
    void send_LandUAV3();
    void send_MoveENUUAV0();
    void send_MoveENUUAV1();
    void send_MoveENUUAV2();
    void send_MoveENUUAV3();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace qt_ground_station

#endif // qt_ground_station_MAIN_WINDOW_H
