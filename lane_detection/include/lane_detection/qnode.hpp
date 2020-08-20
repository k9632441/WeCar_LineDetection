/**
 * @file /include/lane_detection/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef lane_detection_QNODE_HPP_
#define lane_detection_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace lane_detection {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
  bool init();
  void run();

private:
  void CameraCallback(const sensor_msgs::ImageConstPtr &img,
                      const sensor_msgs::CameraInfoConstPtr &info);

Q_SIGNALS:
    void rosShutdown();

    void CameraRecved(const cv::Mat&, const cv::Mat&);

public:
    bool isRecv;
    ros::Publisher positionPub, speedPub;

private:
	int init_argc;
  char** init_argv;

  image_transport::CameraSubscriber camSub;
};

}  // namespace lane_detection

#endif /* lane_detection_QNODE_HPP_ */
