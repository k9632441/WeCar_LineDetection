/**
 * @file /include/lane_detection/main_window.hpp
 *
 * @brief Qt based gui for lane_detection.
 *
 * @date November 2010
 **/
#ifndef lane_detection_MAIN_WINDOW_H
#define lane_detection_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace lane_detection {

typedef struct ColorRange_ {
  enum {
    h,
    s,
    v
  };
  enum {
    e,
    d
  };

  int low_range[3];
  int high_range[3];
  int morph[2];

  ColorRange_() {
    memset(low_range, 0, sizeof(int)*3);
    memset(high_range, 0, sizeof(int)*3);
    memset(morph, 0, sizeof(int)*2);
  }

} ColorRange;

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

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:  
  /******************************************
  ** Manual connections
  *******************************************/
  void imageUpdate(const cv::Mat &img, const cv::Mat &info);

  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_tabWidget_currentChanged(int index);

  void on_sliderHmax_valueChanged(int value);
  void on_sliderHmin_valueChanged(int value);
  void on_sliderSmax_valueChanged(int value);
  void on_sliderSmin_valueChanged(int value);
  void on_sliderVmax_valueChanged(int value);
  void on_sliderVmin_valueChanged(int value);
  void on_sliderErode_valueChanged(int value);
  void on_sliderDilate_valueChanged(int value);

  void on_buttonSave_clicked();

private:
  void updateUI();

  void loadColorInfo();
  void saveColorInfo();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

  ColorRange lanecolor, linecolor;
};

}  // namespace lane_detection

#endif // lane_detection_MAIN_WINDOW_H
