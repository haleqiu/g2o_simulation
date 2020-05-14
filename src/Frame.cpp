#include "Frame.h"

Frame::Frame(Eigen::Vector3d eularangle, Eigen::Vector3d xyz, int id):idx(id){
  // initiate with the xyz movement and the eular ange
  double roll = eularangle(0);
  double pitch = eularangle(1);
  double yaw = eularangle(2);
  //from matrix_utils
  Eigen::Matrix3d _R = euler_zyx_to_rot<double>(roll, pitch, yaw);
  transform.translation()=xyz;//Twc
  transform.rotate(Eigen::AngleAxisd(_R));
  truepose = transform.matrix();
  cv::eigen2cv(_R,mRwc);

  mTwc = cv::Mat::eye(4,4,CV_32F);
  cv::eigen2cv(xyz,mOw);
  mRwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
  mOw.copyTo(mTwc.rowRange(0,3).col(3));

  //From Twc to Tcw
  mRcw = mRwc.t();
  mtcw = -mRcw * mOw;
  mTcw = cv::Mat::eye(4,4,CV_32F);
  mRcw.copyTo(mTcw.rowRange(0,3).colRange(0,3));
  mtcw.copyTo(mTcw.rowRange(0,3).col(3));
};

Frame::Frame(Eigen::Matrix4d TrueMotion, int id):idx(id){
  truepose = TrueMotion;
  transform.matrix() = TrueMotion;
};

Frame::Frame(Eigen::Affine3d Truetransform, int id):idx(id){
  transform = Truetransform;
  truepose = Truetransform.matrix();
}


// tothink why is the Tcw word view current // this is the pattern of the ORB_SLAM2
void Frame::SetPose(cv::Mat Tcw)//the pose of the camera pose
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
};

//What KITTI store is twc
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
