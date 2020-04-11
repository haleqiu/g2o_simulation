#ifndef POINT_H
#define POINT_H

#include <Eigen/Core>
#include <Eigen/Dense>

// #include "Point.h"
#include <opencv2/core/core.hpp>
#include "opencv2/core/eigen.hpp"
#include "matrix_utils.h"


class Point{
  public:
  Eigen::Vector3d truepos;//
  Eigen::Vector3d simulatedpos;
  Eigen::Vector3d optimizedpos;
  std::vector<int> connectedBy;

  Point(int id);
  Point(cv::Mat &Pos, int id);
  Point(Eigen::Vector3d &Pos, int id);
  cv::Mat mpos;
  int idx;
  bool isdyn;

  std::vector<int> seenBy;
};


#endif
