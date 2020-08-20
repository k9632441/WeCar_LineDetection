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
#include <std_msgs/Float64.h>
#include <sstream>
#include "../include/lane_detection/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace lane_detection {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"lane_detection");
        ros::Time::init();
        while( ! ros::master::check() )
        {
            std::cout << "wating for master" << std::endl;
            ros::Duration(0.5).sleep();
        }
	if ( ! ros::master::check() ) {
            std::cout << "ros master check failed" << std::endl;
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  // Add your ros communications here.
  isRecv = false;
  image_transport::ImageTransport it(n);
  camSub = it.subscribeCamera("/usb_cam/image_raw", 10, &QNode::CameraCallback, this);

  positionPub = n.advertise<std_msgs::Float64>("/commands/servo/position", 1);
  speedPub    = n.advertise<std_msgs::Float64>("/commands/motor/speed", 1);

	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(33);
        while ( ros::ok() ) {
		ros::spinOnce();
    loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::CameraCallback(const sensor_msgs::ImageConstPtr &img,
                    const sensor_msgs::CameraInfoConstPtr &info)
{
  if(!isRecv)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat *K;
    try {
      K = new cv::Mat(3, 3, CV_64FC1, (void*)info->K.data());
    } catch (cv::Exception &e) {
      ROS_ERROR("cv exception: %s", e.what());
      return;
    }

    isRecv = true;
    Q_EMIT CameraRecved(cv_ptr->image, *K);
  }
}

}  // namespace lane_detection
