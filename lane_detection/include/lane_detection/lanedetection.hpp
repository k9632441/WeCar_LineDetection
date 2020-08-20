#ifndef LANEDETECTION_HPP
#define LANEDETECTION_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>


class LaneDetection
{
public:
  LaneDetection();
  LaneDetection(const cv::Mat &img, const int &threshold, const int &minLineLength, const int &maxLimeGap);

  ~LaneDetection();

  void setImage(const cv::Mat &img);
  void setHoughLineParam(const int &threshold, const int &minLineLength, const int &maxLimeGap);

  const std::vector<cv::Vec4i>& getLanes() const { return mLanes; }
  const std::vector<cv::Vec4i>& getHoughLines() const { return mHoughLines; }
  const cv::Vec4i& getBaseLine() const { return mBaseLine; }
  const int& getSteer() const { return mSteer; }

  void run();

private:
  const std::vector<cv::Vec4i> recognition();
  const std::vector<cv::Vec4i> judgement(const std::vector<cv::Vec4i> &lines);

  const cv::Vec4i drawBaseLine(const int &_num, const int &_x, const int &_y);
  void stabilizeDeg(const int &_num, const int &_calc_deg);

  const cv::Vec4i makePoints(const cv::Vec2d &line);

private:
  cv::Mat mImg;

  int _threshold;
  int _minLineLength;
  int _maxLineGap;

  std::vector<cv::Vec4i> mHoughLines;
  std::vector<cv::Vec4i> mLanes;
  cv::Vec4i mBaseLine;
  int mSteer;
};

/**
 * @brief C++ implementation of numpy polyfit with Eigen lib
 * @ref   https://github.com/patLoeber/Polyfit
 */
template<typename T>
std::vector<T> polyfit(const std::vector<T> &xValues, const std::vector<T> &yValues, const int &degree,
                       const std::vector<T> &weights = std::vector<T>(), bool useJacobi = false);

#endif // LANEDETECTION_HPP
