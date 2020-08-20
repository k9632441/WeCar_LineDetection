#ifndef LINEDETECTION_HPP
#define LINEDETECTION_HPP

#include <vector>
#include <opencv2/opencv.hpp>


class LineDetection
{
public:
  LineDetection();
  LineDetection(const cv::Mat &img, const int &threshold, const int &minLineLength, const int &maxLineGap, const int &tolerance);

  ~LineDetection();

  void setImage(const cv::Mat &img);
  void setHoughLineParam(const int &threshold, const int &minLineLength, const int &maxLimeGap);

  const cv::Vec4i &getLine() const { return mLine; }
  const std::vector<cv::Vec4i> &getLines() const { return _lines; }

  void run();

private:
  const cv::Vec4i makePoints(const cv::Vec2d &line);

private:
  cv::Mat mImg;

  int _threshold;
  int _minLineLength;
  int _maxLineGap;
  double _tolerance;

  cv::Vec4i mLine;
  std::vector<cv::Vec4i> _lines;
  int mSteer;
};

#endif // LINEDETECTION_HPP
