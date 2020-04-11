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
    int iGlobalId;
    LandmarkGrid grid;
    FrameVec gridposes;
    PointPtrVec Landmarks;
    PointPtrVec vDynamicLandmarks;
    std::vector<RigidEdge> vRigidEdges;

    //TODO:FileStorage
    int numNodes = 20;
    int steps = 5;//TODO
    double stepLen = 1.0;
    int boundArea = 50;//TODO
    int landmarksRange = 2;
    double maxSensorRangeLandmarks = 2.5 * stepLen;//it decide how many
    int landMarksPerSquareMeter = 1;

    //expected identifier before numeric constant
    Eigen::Vector3d TransNoise;
    Eigen::Vector3d RotNoise;
    Eigen::Vector3d LandmarkNoise;

    void InitializeFrames();//adding gridposes
    void AddingStaticLandmarks();//adding Landmarks
    void AddingDynamicCircle();


    const double dCircleAngle = 18.0;
    double kGtDistance = 0;

  private:
    Frame GenerateNewPose(const Frame& prev, const Eigen::Matrix4d& Motion,
                      Eigen::Vector3d transNoise,  Eigen::Vector3d& rotNoise);
    void AddingRigidEdges();
    void InitRigidEdgesFully(PointPtrVec& dynamic_points, int obj_id);
    void AddingRigidEdgeFully(DyObject & dyobj);
    void InitRigidEdgesTwo(PointPtrVec& dynamic_points, int obj_id);
    void AddingRigidEdgeTwo(DyObject& dyobj);

};

#endif // G2O_TYPES_SLAM3D_API_H
