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
#include <fstream>
#include <vector>
#include <cmath>
#include <std_msgs/Float64.h>
#include "../include/lane_detection/main_window.hpp"
#include "../include/lane_detection/lanedetection.hpp"
#include "../include/lane_detection/linedetection.hpp"
#include "../include/lane_detection/robit_ransac.h"
#include "../include/lane_detection/robit_master_vision.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace lane_detection {

using namespace Qt;
using namespace cv;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType<cv::Mat>("cv::Mat");

  QObject::connect(&qnode, SIGNAL(CameraRecved(const cv::Mat&, const cv::Mat&)),
                   this,   SLOT(imageUpdate(const cv::Mat&, const cv::Mat&)));

  loadColorInfo();
  ui.tabWidget->setCurrentIndex(1);

  cout << "init" << endl;

  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void MainWindow::imageUpdate(const cv::Mat &img, const cv::Mat &info)
{
  if(qnode.isRecv)
  {
    Mat imgRgb = img.clone();
    cv::resize(imgRgb, imgRgb, Size(0, 0), 0.5, 0.5);

    Mat imgLine = imgRgb.clone();
    GaussianBlur(imgLine, imgLine, Size(5, 5), -1);

    Mat imgHsv;
    cvtColor(imgLine, imgHsv, COLOR_BGR2HSV);

    Mat mask;

    int index = ui.tabWidget->currentIndex();

    // lane detection
    if(index == 0)
    {
      inRange(imgHsv, Scalar(lanecolor.low_range[0], lanecolor.low_range[1], lanecolor.low_range[2]),
          Scalar(lanecolor.high_range[0], lanecolor.high_range[1], lanecolor.high_range[2]), mask);
      erode(mask, mask,
            getStructuringElement(MORPH_RECT, Size(2*lanecolor.morph[0]+1, 2*lanecolor.morph[0]+1), Point(lanecolor.morph[0], lanecolor.morph[0])));
      dilate(mask, mask,
             getStructuringElement(MORPH_RECT, Size(2*lanecolor.morph[1]+1, 2*lanecolor.morph[1]+1), Point(lanecolor.morph[1], lanecolor.morph[1])));

      vector<Point> roi;
      roi.push_back(Point(0, mask.rows));
      roi.push_back(Point(0, mask.rows/15));
      roi.push_back(Point(mask.cols, mask.rows/15));
      roi.push_back(Point(mask.cols, mask.rows));

      Mat temp = Mat::zeros(mask.size(), mask.type());
      const Point *pts = (const cv::Point*) Mat(roi).data;
      int npts = Mat(roi).rows;

      fillPoly(temp, &pts, &npts, 1, Scalar(255));
      bitwise_and(mask, temp, mask);

      LaneDetection lane(mask, 40, 30, 50); // 30, 50
      lane.run();

      vector<Vec4i> houghlines = lane.getHoughLines();

      for( size_t i = 0; i < houghlines.size(); i++ )
      {
        Vec4i l = houghlines[i];
        line( imgRgb, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
      }

      vector<Vec4i> lines = lane.getLanes();

      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        line( imgRgb, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, LINE_AA);
      }

      Vec4i baseLine = lane.getBaseLine();
      line( imgRgb, Point(baseLine[0], baseLine[1]), Point(baseLine[2], baseLine[3]), Scalar(0,255,255), 3, LINE_AA);

      int steer = lane.getSteer();

      stringstream s;
      s << "Steer: " << steer;
      putText(imgRgb, s.str(), Point(baseLine[2], baseLine[3]), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

      std_msgs::Float64 position_value, speed_value;

      if(steer != -99)
      {
        position_value.data = ((steer*M_PI)/180.) + 0.6;
        speed_value.data = 2000 - abs(steer*5);
      }
      else
      {
        position_value.data = 0.6;
        speed_value.data = 1000;
      }

      qnode.positionPub.publish(position_value);
      //speed_value.data = 0;
      qnode.speedPub.publish(speed_value);
	

      QImage qiRgb((const unsigned char*)imgRgb.data, imgRgb.cols, imgRgb.rows, QImage::Format_RGB888);
      ui.qimage1_1->setPixmap(QPixmap::fromImage(qiRgb.rgbSwapped()));

      QImage qiMask((const unsigned char*)mask.data, mask.cols, mask.rows, QImage::Format_Indexed8);
      ui.qimage1_2->setPixmap(QPixmap::fromImage(qiMask));
    }
    // single line detection
    else if(index == 1)
    {
      inRange(imgHsv, Scalar(linecolor.low_range[0], linecolor.low_range[1], linecolor.low_range[2]),
          Scalar(linecolor.high_range[0], linecolor.high_range[1], linecolor.high_range[2]), mask);
      erode(mask, mask,
            getStructuringElement(MORPH_RECT, Size(2*linecolor.morph[0]+1, 2*linecolor.morph[0]+1), Point(linecolor.morph[0], linecolor.morph[0])));
      dilate(mask, mask,
             getStructuringElement(MORPH_RECT, Size(2*linecolor.morph[1]+1, 2*linecolor.morph[1]+1), Point(linecolor.morph[1], linecolor.morph[1])));

      vector<Point> roi;
      roi.push_back(Point(0/*mask.cols/4*/, mask.rows));
      roi.push_back(Point(0/*mask.cols/4*/, mask.rows/2));
      roi.push_back(Point(mask.cols/**3/4*/, mask.rows/2));
      roi.push_back(Point(mask.cols/**3/4*/, mask.rows));

      rectangle(imgRgb, roi[1], roi[3], Scalar(0, 0, 255));

      Mat temp = Mat::zeros(mask.size(), mask.type());
      const Point *pts = (const cv::Point*) Mat(roi).data;
      int npts = Mat(roi).rows;

      fillPoly(temp, &pts, &npts, 1, Scalar(255));
      bitwise_and(mask, temp, mask);

      Rect roi_upper(roi[1].x, roi[1].y, roi[2].x - roi[1].x, (roi[3].y - roi[2].y)*2/3);
      Rect roi_lower(roi[1].x, roi[1].y + (roi[3].y - roi[2].y)*1/3, roi[2].x - roi[1].x, (roi[3].y - roi[2].y)*2/3);

      rectangle(imgRgb, roi_upper, Scalar(0, 255, 0), 2);
      rectangle(imgRgb, roi_lower, Scalar(255, 0, 0), 2);

      Mat upper = mask(roi_upper).clone();
      Mat lower = mask(roi_lower).clone();

      std_msgs::Float64 position_value, speed_value;
      static double past_servo = 0., past_speed = 0.;

      // upper
      LineDetection lineUP(upper, 40, 20, 50, 10.); // 30, 50
      lineUP.run();

      vector<Vec4i> linesUP = lineUP.getLines();

      int upperSX = roi_upper.x, upperSY = roi_upper.y;
      Point upperSt(upperSX, upperSY);

//      for(auto line : linesUP)
//        cv::line(imgRgb, Point(line[0], line[1])+upperSt, Point(line[2], line[3])+upperSt, Scalar(0, 255, 255), 2);

      Vec4i vlineUP = lineUP.getLine();

      if(vlineUP[0] != 0 && vlineUP[2] != 0)
        cv::line(imgRgb, Point(vlineUP[0], vlineUP[1])+upperSt, Point(vlineUP[2], vlineUP[3])+upperSt, Scalar(0, 255, 255), 2);

      RobitLabling labelUP(upper, 800);
      labelUP.doLabeling();

      int max_upper_i = -1;
      int max_upper_area = 0;

      for(int i = 0; i < labelUP.getNumOfBlobs(); i++)
      {
        if(vlineUP[0] != 0 && vlineUP[2] != 0)
        {
          int lineX = (vlineUP[0]+vlineUP[2]) / 2 + upperSt.x;
          int lineY = (vlineUP[1]+vlineUP[3]) / 2 + upperSt.y;
          circle(imgRgb, Point(lineX, lineY), 4, Scalar(255, 0, 0), 2);

          Rect blob = labelUP.getRecBlobs()[i];
          int sX = blob.x + upperSt.x, sY = blob.y + upperSt.y;
          int eX = sX + blob.width;
          int eY = sY + blob.height;

          if(lineX <= sX || lineX > eX) continue;
          if(lineY <= sY || lineY > eY) continue;
        }
        else continue;

        int area = labelUP.getRecBlobs()[i].area();
        if(area > max_upper_area)
        {
          max_upper_area = area;
          max_upper_i = i;
        }
      }

      cout << "====================================" << endl;
      cout << "upper index = " << max_upper_i << endl;
      if(max_upper_i != -1)
      {
        Rect blob = labelUP.getRecBlobs()[max_upper_i];
        int sX = blob.x, sY = blob.y;
        int eX = blob.x + blob.width;
        int eY = blob.y + blob.height;

        Point upperCenter(0, 0);
        int nCount = 0;

        for(int y = sY, idxY = sY*upper.cols; y < eY; y++, idxY += upper.cols)
          for(int x = sX; x < eX; x++)
          {
            if(labelUP.m_Image.data[idxY+x] == max_upper_i+1)
            {
              upperCenter.x += x;
              upperCenter.y += y;
              nCount++;
            }
          }

        if(nCount != 0)
        {
          upperCenter /= nCount;
        }

        int upperSX = roi_upper.x, upperSY = roi_upper.y;
        Point upperSt(upperSX, upperSY);

        circle(imgRgb, upperSt + upperCenter, 4, Scalar(0, 0, 255), 2);
      }
      else
      {
          ;
      }

      // lower
      LineDetection lineLO(lower, 40, 30, 50, 10.); // 30, 50
      lineLO.run();

      vector<Vec4i> linesLO = lineLO.getLines();

      int lowerSX = roi_lower.x, lowerSY = roi_lower.y;
      Point lowerSt(lowerSX, lowerSY);

//      for(auto line : linesLO)
//        cv::line(imgRgb, Point(line[0], line[1])+lowerSt, Point(line[2], line[3])+lowerSt, Scalar(0, 255, 255), 2);

      Vec4i vlineLO = lineLO.getLine();

      if(vlineLO[0] != 0 && vlineLO[2] != 0)
        cv::line(imgRgb, Point(vlineLO[0], vlineLO[1])+lowerSt, Point(vlineLO[2], vlineLO[3])+lowerSt, Scalar(0, 255, 255), 2);

      RobitLabling labelLO(lower, 800);
      labelLO.doLabeling();

      int max_lower_i = -1;
      int max_lower_area = 0;

      for(int i = 0; i < labelLO.getNumOfBlobs(); i++)
      {
        if(vlineLO[0] != 0 && vlineLO[2] != 0)
        {
          int lineX = (vlineLO[0]+vlineLO[2]) / 2 + lowerSt.x;
          int lineY = (vlineLO[1]+vlineLO[3]) / 2 + lowerSt.y;
          circle(imgRgb, Point(lineX, lineY), 4, Scalar(255, 0, 0), 2);

          Rect blob = labelLO.getRecBlobs()[i];
          int sX = blob.x + lowerSt.x, sY = blob.y + lowerSt.y;
          int eX = sX + blob.width;
          int eY = sY + blob.height;

          if(lineX <= sX || lineX > eX) continue;
          if(lineY <= sY || lineY > eY) continue;
        }
        else continue;

        int area = labelLO.getRecBlobs()[i].area();
        if(area > max_lower_area)
        {
          max_lower_area = area;
          max_lower_i = i;
        }
      }

      cout << "lower index = " << max_lower_i << endl;

      if(max_lower_i != -1)
      {
        Rect blob = labelLO.getRecBlobs()[max_lower_i];
        int sX = blob.x, sY = blob.y;
        int eX = blob.x + blob.width;
        int eY = blob.y + blob.height;

        Point lowerCenter(0, 0);
        int nCount = 0;

        for(int y = sY, idxY = sY*lower.cols; y < eY; y++, idxY += lower.cols)
          for(int x = sX; x < eX; x++)
          {
            if(labelLO.m_Image.data[idxY+x] == max_lower_i+1)
            {
              lowerCenter.x += x;
              lowerCenter.y += y;
              nCount++;
            }
          }

        if(nCount != 0)
        {
          lowerCenter /= nCount;
        }

        int lowerSX = roi_lower.x, lowerSY = roi_lower.y;
        Point lowerSt(lowerSX, lowerSY);

        circle(imgRgb, lowerSt + lowerCenter, 4, Scalar(0, 0, 255), 2);

        // P control
        int centerX = mask.cols/2;
        double e = (lowerSt.x + lowerCenter.x) - centerX;
        static double e_past = 0.;
        double e_diff = (e - e_past)/0.03;
        e_past = e;
        ui.printLog->appendPlainText("e_diff = " + QString::number(e_diff, 'g', 3));

        double kP = 0.2; //7
        double kD = 0.0;
        double steer = kP*e + kD*e_diff;
        if(steer > 30.0) // 30
            steer = 30.0;
        else if(steer < -30.0)
            steer = -30.0;
        ui.printLog->appendPlainText("steer from lower = " + QString::number(steer, 'g', 3));

        position_value.data = ((steer*M_PI)/180.) + 0.4501;
        speed_value.data = 3200 - abs(steer*15); //3200
        past_servo = position_value.data;
        past_speed = speed_value.data;
      }
      else
      {
        position_value.data = past_servo;
        speed_value.data = past_speed;
      }
	

      qnode.positionPub.publish(position_value);
	//speed_value.data = 0;
      qnode.speedPub.publish(speed_value);

      // RANSAC
//      Mat canny;
//      Canny(mask, canny, 75, 150);

//      JYJ_RansacLine line(canny);

//      line.runRansac();

//      Point Pst = line.getLineStart(), Ped = line.getLineEnd();

//      cv::line(imgRgb, Pst, Ped, Scalar(0, 255, 255), 2);

      QImage qiRgb((const unsigned char*)imgRgb.data, imgRgb.cols, imgRgb.rows, QImage::Format_RGB888);
      ui.qimage2_1->setPixmap(QPixmap::fromImage(qiRgb.rgbSwapped()));

      QImage qiMask((const unsigned char*)mask.data, mask.cols, mask.rows, QImage::Format_Indexed8);
      ui.qimage2_2->setPixmap(QPixmap::fromImage(qiMask));
    }

    qnode.isRecv = false;
  }
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
  ui.printLog->appendPlainText("index = " + QString::number(index));
  switch (index)
  {
  case 0:
    break;

  case 1:
    break;

  default:
    break;
  }

  updateUI();
}

void MainWindow::on_sliderHmax_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.high_range[ColorRange::h] = value;

    ui.label_hmax->setText(QString::number(lanecolor.high_range[ColorRange::h]));
    break;

  case 1:
    linecolor.high_range[ColorRange::h] = value;

    ui.label_hmax->setText(QString::number(linecolor.high_range[ColorRange::h]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderHmin_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.low_range[ColorRange::h] = value;

    ui.label_hmin->setText(QString::number(lanecolor.low_range[ColorRange::h]));
    break;

  case 1:
    linecolor.low_range[ColorRange::h] = value;

    ui.label_hmin->setText(QString::number(linecolor.low_range[ColorRange::h]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderSmax_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.high_range[ColorRange::s] = value;

    ui.label_smax->setText(QString::number(lanecolor.high_range[ColorRange::s]));
    break;

  case 1:
    linecolor.high_range[ColorRange::s] = value;

    ui.label_smax->setText(QString::number(linecolor.high_range[ColorRange::s]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderSmin_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.low_range[ColorRange::s] = value;

    ui.label_smin->setText(QString::number(lanecolor.low_range[ColorRange::s]));
    break;

  case 1:
    linecolor.low_range[ColorRange::s] = value;

    ui.label_smin->setText(QString::number(linecolor.low_range[ColorRange::s]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderVmax_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.high_range[ColorRange::v] = value;

    ui.label_vmax->setText(QString::number(lanecolor.high_range[ColorRange::v]));
    break;

  case 1:
    linecolor.high_range[ColorRange::v] = value;

    ui.label_vmax->setText(QString::number(linecolor.high_range[ColorRange::v]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderVmin_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.low_range[ColorRange::v] = value;

    ui.label_vmin->setText(QString::number(lanecolor.low_range[ColorRange::v]));
    break;

  case 1:
    linecolor.low_range[ColorRange::v] = value;

    ui.label_vmin->setText(QString::number(linecolor.low_range[ColorRange::v]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderErode_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.morph[ColorRange::e] = value;

    ui.label_erode->setText(QString::number(lanecolor.morph[ColorRange::e]));
    break;

  case 1:
    linecolor.morph[ColorRange::e] = value;

    ui.label_erode->setText(QString::number(linecolor.morph[ColorRange::e]));
    break;

  default:
    break;
  }
}

void MainWindow::on_sliderDilate_valueChanged(int value)
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    lanecolor.morph[ColorRange::d] = value;

    ui.label_dilate->setText(QString::number(lanecolor.morph[ColorRange::d]));
    break;

  case 1:
    linecolor.morph[ColorRange::d] = value;

    ui.label_dilate->setText(QString::number(linecolor.morph[ColorRange::d]));
    break;

  default:
    break;
  }
}

void MainWindow::on_buttonSave_clicked()
{
  saveColorInfo();
}

void MainWindow::updateUI()
{
  int index = ui.tabWidget->currentIndex();

  switch (index)
  {
  case 0:
    ui.sliderHmax->setValue(lanecolor.high_range[ColorRange::h]);
    ui.sliderHmin->setValue(lanecolor.low_range[ColorRange::h]);
    ui.sliderSmax->setValue(lanecolor.high_range[ColorRange::s]);
    ui.sliderSmin->setValue(lanecolor.low_range[ColorRange::s]);
    ui.sliderVmax->setValue(lanecolor.high_range[ColorRange::v]);
    ui.sliderVmin->setValue(lanecolor.low_range[ColorRange::v]);
    ui.sliderErode->setValue(lanecolor.morph[ColorRange::e]);
    ui.sliderDilate->setValue(lanecolor.morph[ColorRange::d]);
    break;

  case 1:
    ui.sliderHmax->setValue(linecolor.high_range[ColorRange::h]);
    ui.sliderHmin->setValue(linecolor.low_range[ColorRange::h]);
    ui.sliderSmax->setValue(linecolor.high_range[ColorRange::s]);
    ui.sliderSmin->setValue(linecolor.low_range[ColorRange::s]);
    ui.sliderVmax->setValue(linecolor.high_range[ColorRange::v]);
    ui.sliderVmin->setValue(linecolor.low_range[ColorRange::v]);
    ui.sliderErode->setValue(linecolor.morph[ColorRange::e]);
    ui.sliderDilate->setValue(linecolor.morph[ColorRange::d]);
    break;

  default:
    break;
  }
}

void MainWindow::loadColorInfo()
{
  ifstream fin;
  fin.open("/home/nvidia/wecar_ws/src/lane_detection/data/lanecolor.txt");
  if(fin.is_open())
  {
    for(int i = 0; i < 3; i++)
    {
      fin >> lanecolor.low_range[i]
          >> lanecolor.high_range[i];
    }
    for(int i = 0; i < 2; i++)
      fin >> lanecolor.morph[i];

    fin.close();
  }

  fin.open("/home/nvidia/wecar_ws/src/lane_detection/data/linecolor.txt");
  if(fin.is_open())
  {
    for(int i = 0; i < 3; i++)
    {
      fin >> linecolor.low_range[i]
          >> linecolor.high_range[i];
    }
    for(int i = 0; i < 2; i++)
      fin >> linecolor.morph[i];

    fin.close();
  }

  updateUI();
}

void MainWindow::saveColorInfo()
{
  int index = ui.tabWidget->currentIndex();
  ofstream fout;

  switch (index)
  {
  case 0:
  {
    fout.open("/home/nvidia/wecar_ws/src/lane_detection/data/lanecolor.txt");
    if(fout.is_open())
    {
      for(int i = 0; i < 3; i++)
      {
        fout << lanecolor.low_range[i] << endl
             << lanecolor.high_range[i] << endl;
      }
      for(int i = 0; i < 2; i++)
        fout << lanecolor.morph[i] << endl;

      fout.close();
    }
    break;
  }

  case 1:
  {
    fout.open("/home/nvidia/wecar_ws/src/lane_detection/data/linecolor.txt");
    if(fout.is_open())
    {
      for(int i = 0; i < 3; i++)
      {
        fout << linecolor.low_range[i] << endl
             << linecolor.high_range[i] << endl;
      }
      for(int i = 0; i < 2; i++)
        fout << linecolor.morph[i] << endl;

      fout.close();
    }
    break;
  }

  default:
  {
    break;
  }
  }
}

}  // namespace lane_detection
