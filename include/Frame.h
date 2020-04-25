#ifndef FRAME_H
#define FRAME_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include "opencv2/core/eigen.hpp"
#include "matrix_utils.h"
#include "Point.h"
typedef std::vector<Point*> PointPtrVec;

struct rigidbody{//means one constraint one line
  Point* first_point;
  Point* second_point;
  int distance_id;
};

struct PointMeasurement{//means one constraint one line
  Eigen::Vector3d observation;
  int point_id;
};

typedef std::vector<PointMeasurement> PointMeasurementVec;
typedef std::vector<rigidbody> RigidPairVec;

struct DyObject{
  int id;
  PointPtrVec dylandmarks;
  Eigen::Affine3d motion_transform;
  int frameid;
  RigidPairVec rigidbodypairs;
};

typedef std::vector<DyObject> ObjectVec;


class Frame{
  public:
    //define the points observed
    PointPtrVec landmarks;
    Eigen::Matrix4d truepose;
    Eigen::Matrix4d simulatedpose;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();//for pcl visualisation
    Eigen::Affine3d simulatedtransform = Eigen::Affine3d::Identity();//for pcl visualisation
    Eigen::Affine3d optimizedtransform = Eigen::Affine3d::Identity();//for pcl visualisation

    //Frame(cv::Mat _R, cv::Mat _t, int id);
    Frame(Eigen::Vector3d eularangle, Eigen::Vector3d xyz, int id);
    Frame(Eigen::Matrix4d TrueMotion, int id);
    ObjectVec seenedobjs;
    PointMeasurementVec vPointMeasurements;

    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mTcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc which is the pose of the camera
    cv::Mat mTwc;

    int time_step;
    int idx;

    void addmotion();
    void SetPose(cv::Mat Tcw);
    void UpdatePoseMatrices();

    //adding noise to the symulated pose
    void sampleNoiseTransform(Eigen::Vector3d& transNoise, Eigen::Vector3d& rotNoise);};

#endif // ORBVOCABULARY_H
