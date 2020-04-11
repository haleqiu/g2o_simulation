#include "Point.h"

Point::Point(cv::Mat &Pos, int id):
  idx(id)
{
  Pos.copyTo(mpos);//Do we need deep copy or not?
  cv2eigen(mpos,truepos);
}

Point::Point(int id):idx(id){}

Point::Point(Eigen::Vector3d &Pos, int id):
  idx(id),truepos(Pos)
{
  eigen2cv(Pos,mpos);
  truepos=Pos;//deep copy or not
}
