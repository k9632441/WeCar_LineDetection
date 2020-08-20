#include "../include/lane_detection/lanedetection.hpp"

#include <cmath>
#include <numeric>


using namespace cv;
using namespace std;

LaneDetection::LaneDetection() :
    mSteer(0)
{
  if(!mImg.empty()) mImg.release();

  _threshold      = 0;
  _minLineLength  = 0;
  _maxLineGap     = 0;
}

LaneDetection::LaneDetection(const Mat &img, const int &threshold, const int &minLineLength, const int &maxLimeGap) :
    mSteer(0)
{
  setImage(img);
  setHoughLineParam(threshold, minLineLength, maxLimeGap);
}

LaneDetection::~LaneDetection()
{
  if(!mImg.empty()) mImg.release();
}

void LaneDetection::setImage(const Mat &img)
{
  CV_Assert(img.type() == CV_8UC1);

  mImg = img.clone();
}

void LaneDetection::setHoughLineParam(const int &threshold, const int &minLineLength, const int &maxLimeGap)
{
  _threshold = threshold;
  _minLineLength = minLineLength;
  _maxLineGap = maxLimeGap;
}

void LaneDetection::run()
{
  if(!mLanes.empty()) mLanes.clear();
  vector<Vec4i> lines = recognition();
  mLanes = judgement(lines);

  int height = mImg.rows;
  int width  = mImg.cols;

  int base_x, base_y;
  Vec4i base;

  cout << "num of lanes = " << mLanes.size() << endl;

  if(mLanes.size() == 2)
  {
    int left_x2 = mLanes[0][2];
    int right_x2 = mLanes[1][2];

    int mid = width / 2.f;
    if(left_x2 > width*2/3 || right_x2 < width/3)
    {
      if(left_x2 > width*2/3)
        base_x = left_x2 - (width/2);
      else if(right_x2 < width/3)
        base_x = width/2 - right_x2;
      base_y = height / 2;
      base = drawBaseLine(1, base_x, base_y);
    }
    else
    {
      base_x = (left_x2 + right_x2) / 2 - mid;
      base_y = (int)(height / 2.);

      base = drawBaseLine(2, base_x, base_y);
    }
  }
  else if(mLanes.size() == 1)
  {
    int x1 = mLanes[0][0], x2 = mLanes[0][2];

    base_x = x2 - x1;
    base_y = (int)(height / 2.);

    base = drawBaseLine(1, base_x, base_y);
  }
  else
  {
    base_x = width / 2.;
    base = drawBaseLine(0, base_x, height);
  }

  mBaseLine = base;
}

const vector<Vec4i> LaneDetection::recognition()
{
  CV_Assert(!mImg.empty());

  Mat canny;
  Canny(mImg, canny, 75, 150);

  vector<Vec4i> lines;
  HoughLinesP(canny, lines, 1, CV_PI/180, _threshold, _minLineLength, _maxLineGap);

  // temp
  mHoughLines = lines;
  return lines;
}

const std::vector<Vec4i> LaneDetection::judgement(const vector<Vec4i> &lines)
{
  if(!lines.empty())
  {
    int width = mImg.cols;

    float boundary = 2. / 3.;
    int left_region_boundary = width * (1 - (float)boundary);
    int right_region_boundary = width * (float)boundary;

    vector<Vec4i> lane_lines;

    vector<Vec2d> left_fit;
    vector<Vec2d> right_fit;

    cout << "============================" << endl;

    for(int i = 0; i < lines.size(); i++)
    {
      Vec4i line = lines[i];
      int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

      if(x1 == x2)  continue;
      if(y1 == y2)  continue;

      vector<double> x;
      x.push_back((double)x1);
      x.push_back((double)x2);

      vector<double> y;
      y.push_back((double)y1);
      y.push_back((double)y2);

      vector<double> fit = polyfit(x, y, 1);

      double slope = fit[1];
      double intercept = fit[0];

      /* / shaped lines */
      if(slope < 0)
      {
        if(x1 < left_region_boundary)
          left_fit.push_back(Vec2d(slope, intercept));
      }
      /* \ shaped lines */
      else if(slope > 0)
      {
        if(x1 > right_region_boundary)
          right_fit.push_back(Vec2d(slope, intercept));
      }
      else  continue;
    }

    if(left_fit.size() > 0)
    {
      Vec2d left_fit_average = std::accumulate(left_fit.begin(), left_fit.end(), Vec2d(0., 0.)) / (double)left_fit.size();
      lane_lines.push_back(makePoints(left_fit_average));
    }
    if(right_fit.size() > 0)
    {
      Vec2d right_fit_average = std::accumulate(right_fit.begin(), right_fit.end(), Vec2d(0., 0.)) / (double)right_fit.size();
      lane_lines.push_back(makePoints(right_fit_average));
    }

    return lane_lines;
  }
  else
  {
    mSteer = -99;
    return vector<Vec4i>();
  }
}

const Vec4i LaneDetection::drawBaseLine(const int &_num, const int &_x, const int &_y)
{
  int height = mImg.rows;
  int width  = mImg.cols;

  cout << "base_x = " << _x << ", base_y = " << _y << endl;

  double mid_rad = atan((double)_x/(double)_y);

  int mid_deg = (int)(mid_rad*180/M_PI);
  cout << "mid_rad = " << mid_rad << ", mid_deg = " << mid_deg << endl;
  stabilizeDeg(_num, mid_deg);

  int temp = mSteer + 90;
  double ofc_rad = temp / 180. * M_PI;

  int x1 = width/2.;
  int y1 = height;
  int x2;

  cout << "ofc_rad = " << ofc_rad << endl;

  if(ofc_rad == 0.)
    x2 = width/2.;
  else
    x2 = x1 - height / 2 / tan(ofc_rad);

  int y2 = height / 2.;

  return Vec4i(x1, y1, x2, y2);
}

void LaneDetection::stabilizeDeg(const int &_num, const int &_calc_deg)
{
  int max_angle = 0;

  cout << "steer before = " << mSteer << endl;

  //if(mSteer == -99)
    //mSteer = 0;

  if(_num > 0)
    max_angle = 4;
  else
    return;

  if(mSteer != -99)
  {
    int deg_offset = _calc_deg - mSteer;
    if(abs(deg_offset) > max_angle){	
        mSteer = (_calc_deg + max_angle*deg_offset/abs(deg_offset));
	//printf("%d %d %d\n", _calc_deg, deg_offset, mSteer);
	
}
	
int dgr_th = 15;
    if(mSteer > dgr_th)
        mSteer = dgr_th;
    else if(mSteer < -dgr_th)
        mSteer = -dgr_th;
  }
}

const Vec4i LaneDetection::makePoints(const Vec2d &line)
{
  int height = mImg.rows;
  int width  = mImg.cols;
  double slope = line[0];
  double intercept = line[1];
  int y1 = height;
  int y2 = y1/2;

  if(slope == 0.)
    slope = 0.0000001;
  int x1 = max( - width, min(2*width, (int)((y1 - intercept)/slope)));
  int x2 = max( - width, min(2*width, (int)((y2 - intercept)/slope)));

  return Vec4i(x1, y1, x2, y2);
}

template<typename T>
std::vector<T> polyfit(const std::vector<T> &xValues, const std::vector<T> &yValues, const int &degree,
                       const std::vector<T> &weights, bool useJacobi)
{
  using namespace Eigen;

  bool useWeights = weights.size() > 0 && weights.size() == xValues.size();

  int numCoefficients = degree + 1;
  size_t nCount = xValues.size();

  MatrixXf X(nCount, numCoefficients);
  MatrixXf Y(nCount, 1);

  // fill Y matrix
  for (size_t i = 0; i < nCount; i++)
  {
      if (useWeights)
          Y(i, 0) = yValues[i] * weights[i];
      else
          Y(i, 0) = yValues[i];
  }

  // fill X matrix (Vandermonde matrix)
  for (size_t nRow = 0; nRow < nCount; nRow++)
  {
      T nVal = 1.0f;
      for (int nCol = 0; nCol < numCoefficients; nCol++)
      {
          if (useWeights)
              X(nRow, nCol) = nVal * weights[nRow];
          else
              X(nRow, nCol) = nVal;
          nVal *= xValues[nRow];
      }
  }

  VectorXf coefficients;
  if (useJacobi)
      coefficients = X.jacobiSvd(ComputeThinU | ComputeThinV).solve(Y);
  else
      coefficients = X.colPivHouseholderQr().solve(Y);

  return std::vector<T>(coefficients.data(), coefficients.data() + numCoefficients);
}
