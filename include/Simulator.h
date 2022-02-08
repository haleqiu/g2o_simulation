#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Point.h"
#include "Frame.h"
#include "rand.h"

typedef std::vector<Frame> FrameVec;
typedef std::map<int, std::map<int, PointPtrVec> > LandmarkGrid;

struct RigidEdge{//means one constraint one line
  double gtdistance;
  int vertex_id;
  int obj_id;
};


class Simulator{
  public:
    Simulator();
    Simulator(const std::string &strSettingsFile);

    int iGlobalId=0;
    LandmarkGrid grid;
    FrameVec gridposes;
    PointPtrVec Landmarks;
    PointPtrVec vDynamicLandmarks;
    PointPtrVec vDynamicShape;
    std::vector<RigidEdge> vRigidEdges;

    int numNodes = 10;
    int steps = 5;//TODO
    double stepLen = 1.0;
    int boundArea = 50;//TODO
    int landmarksRange = 2;
    double maxSensorRangeLandmarks = 2.5 * stepLen;//it decide how many
    int landMarksPerSquareMeter = 1;
    double observationprob = 0.9;//TODO miss observation

    // FALG
    int mbDistanceGroundTruthInit = 0;
    int mbDebug = 0;
    int mbStraightMotion = 1;
    int mbCameraPoseNoise = 1;

    // Mode
    int mbStatic = 1;
    int mbRigidity = 1;
    int mbMotion = 1;

    //Optimizer
    int Dynum = 10;
    int DyVertexNum = 2;
    double rigidbodyweight = 1;
    double pointmeasurementweight = 1;
    double MotionSigma = 1;
    int iterationsteps = 10;
    //expected identifier before numeric constant
    Eigen::Vector3d TransNoise;
    Eigen::Vector3d RotNoise;
    Eigen::Vector3d LandmarkNoise;

    void InitializeFrames();//adding gridposes
    void InitDynamicShape();//triangular now
    void AddingStaticLandmarks();//adding Landmarks
    void AddingDynamicCircle();
    void AddingDynamicShape();

    const double dCircleAngle = 18.0;//if using circle
    double kGtDistance = 0;

  private:
    Frame GenerateNewPose(const Frame& prev, Eigen::Affine3d Motion,
                      Eigen::Vector3d transNoise,  Eigen::Vector3d& rotNoise);
    Eigen::Affine3d sampleNoiseTransform(Eigen::Affine3d Motion, Eigen::Vector3d& transNoise, Eigen::Vector3d& rotNoise);

    void AddingRigidEdges();
    void InitRigidEdgesFully(PointPtrVec& dynamic_points, int obj_id);
    void AddingRigidEdgeFully(DyObject & dyobj);
    void InitRigidEdgesTwo(PointPtrVec& dynamic_points, int obj_id);
    void AddingRigidEdgeTwo(DyObject& dyobj);
    void InitRigidEdgesThree(PointPtrVec& dynamic_points, int obj_id);


};

#endif // G2O_TYPES_SLAM3D_API_H
