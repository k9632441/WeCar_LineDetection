#include "../include/lane_detection/linedetection.hpp"
#include "../include/lane_detection/lanedetection.hpp"

#include <cmath>
#include <numeric>

using namespace cv;
using namespace std;

LineDetection::LineDetection()
{
  if(!mImg.empty()) mImg.release();

  _threshold      = 0;
  _minLineLength  = 0;
  _maxLineGap     = 0;
  _tolerance      = 0.;
}

LineDetection::LineDetection(const cv::Mat &img, const int &threshold, const int &minLineLength, const int &maxLineGap, const int &tolerance)
{
  setImage(img);
  setHoughLineParam(threshold, minLineLength, maxLineGap);
  _tolerance = tolerance;
}

LineDetection::~LineDetection()
{
  if(!mImg.empty()) mImg.release();
}

void LineDetection::setImage(const cv::Mat &img)
{
  CV_Assert(img.type() == CV_8UC1);

  mImg = img.clone();
}

void LineDetection::setHoughLineParam(const int &threshold, const int &minLineLength, const int &maxLimeGap)
{
  _threshold = threshold;
  _minLineLength = minLineLength;
  _maxLineGap = maxLimeGap;
}

void LineDetection::run()
{
  CV_Assert(!mImg.empty());

  Mat canny;
  Canny(mImg, canny, 75, 150);

  vector<Vec4i> lines, temp;
  HoughLinesP(canny, lines, 1, CV_PI/180, _threshold, _minLineLength, _maxLineGap);

  vector<Point> p1, p2;
  vector<Vec2d> line_fit;
  vector<double> a_pos, a_neg;

  cout << "============================" << endl;

  for(int i = 0; i < lines.size(); i++)
  {
    Vec4i line = lines[i];
    int x1 = line[0], y1 = line[1];
    int x2 = line[2], y2 = line[3];
//    cout << "p1 = (" << x1 << ", " << y1 << ") p2 = (" << x2 << ", " << y2 << ")" << endl;

    if(x1 == x2)  continue;
    if(y1 == y2)  continue;

    double deg = atan2(y2-y1, x2-x1) * 180 / M_PI;
    if(deg > 180.) deg -= 180.;

    if(deg >= -_tolerance && deg <= _tolerance)
      continue;

    temp.push_back(line);

    double dist1 = norm(Point(x1, y1));
    double dist2 = norm(Point(x2, y2));
    p1.push_back((dist1 > dist2 ? Point(x1, y1) : Point(x2, y2)));
    p2.push_back((dist1 > dist2 ? Point(x2, y2) : Point(x1, y1)));

    vector<double> x;
    x.push_back((double)x1);
    x.push_back((double)x2);

    vector<double> y;
    y.push_back((double)y1);
    y.push_back((double)y2);

    // y = ax + b, 1st order: line equation
    vector<double> _fit = polyfit(x, y, 1);
    double a = _fit[1]; // slope
    double b = _fit[0]; // y-axis intercept
    if(a == 0.) a = 0.0001;
    double b_ = -b / a; // x-axis intercept

    line_fit.push_back(Vec2d(a, b_));
//    cout << "slope = " << a << ", x inter = " << b_ << ", y inter = " << b << endl;
  }

  if(temp.size() > 0)
  {
    Vec2d line_fit_average = std::accumulate(line_fit.begin(), line_fit.end(), Vec2d(0., 0.)) / (double)line_fit.size();

    double avg_a  = line_fit_average[0];
    double avg_b_ = line_fit_average[1];
    double avg_b  = -avg_a*avg_b_;
    line_fit_average[1] = avg_b;

//    cout << "avg slope = " << avg_a << ", avg x inter = " << avg_b_ << ", avg y inter = " << avg_b << endl;

//    vector<double> x;
//    x.push_back((double)line_fit_average[0]);
//    x.push_back((double)line_fit_average[2]);

//    vector<double> y;
//    y.push_back((double)line_fit_average[1]);
//    y.push_back((double)line_fit_average[3]);

//    vector<double> line_fit = polyfit(x, y, 1);

//    mLine = makePoints(Vec2d(line_fit[1], line_fit[0]));
    mLine = makePoints(line_fit_average);

//    Point p1_average = std::accumulate(p1.begin(), p1.end(), Point(0, 0)) / (double)p1.size();
//    Point p2_average = std::accumulate(p2.begin(), p2.end(), Point(0, 0)) / (double)p2.size();

//    vector<double> x;
//    x.push_back((double)p1_average.x);
//    x.push_back((double)p2_average.x);

//    vector<double> y;
//    y.push_back((double)p1_average.y);
//    y.push_back((double)p2_average.y);

//    vector<double> line_fit = polyfit(x, y, 1);
//    mLine = makePoints(Vec2d(line_fit[1], line_fit[0]));
  }
  else
    mLine = Vec4i(0, 0, 0, 0);

  if(!_lines.empty()) _lines.clear();
  _lines = temp;
}

const cv::Vec4i LineDetection::makePoints(const cv::Vec2d &line)
{
  int height = mImg.rows;
  int width  = mImg.cols;
  double slope = line[0];
  double intercept = line[1];
  int y1 = height;
  int y2 = y1/8;

  if(slope == 0.)
    slope = 0.0000001;
  int x1 = max( - width, min(2*width, (int)((y1 - intercept)/slope)));
  int x2 = max( - width, min(2*width, (int)((y2 - intercept)/slope)));

  return Vec4i(x1, y1, x2, y2);
}
