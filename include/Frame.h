
#ifndef FRAME_H
#define FRAME_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Frame.h"
#include<opencv2/core/core.hpp>
#include "opencv2/core/eigen.hpp"
#include "matrix_utils.h"
// Eigen


class Frame{
  public:
    //Frame(cv::Mat _R, cv::Mat _t, int id);
    Frame(Eigen::Vector3f eularangle, Eigen::Vector3f xyz, int id);
    // Rotation, translation and camera center
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
};

Frame::Frame(Eigen::Vector3f eularangle, Eigen::Vector3f xyz, int id){
  float roll = eularangle(0);
  float pitch = eularangle(1);
  float yaw = eularangle(2);

  Eigen::Matrix3f _R = euler_zyx_to_rot<float>(roll, pitch, yaw);
  cv::eigen2cv(_R,mRwc);
};

// tothink why is the Tcw word view current // this is the pattern of the ORB_SLAM2
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
};

void Frame::UpdatePoseMatrices()// from Tcw to Twc?
{   //Tcw means convert a world point to a cemara
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);//;the current frame the view of original pose
    mOw = -mRcw.t()*mtcw;//twc in world view of current so is the position of the camera pose

    mTwc = cv::Mat::eye(4,4,CV_32F);
    mRwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
    mOw.copyTo(mTwc.rowRange(0,3).col(3));
};



#endif // ORBVOCABULARY_H
