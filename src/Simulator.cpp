#include "Simulator.h"


Simulator::Simulator(){
  TransNoise << 0,0,0;
  RotNoise << 0,0,0;
  LandmarkNoise << 0.05,0.05,0.05;
  std::cerr << "LandmarkNoise: "<<LandmarkNoise << '\n';
};

void Simulator::InitializeFrames()
{
  //first pose initialization
  Frame firstpose(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0),1);
  gridposes.push_back(firstpose);
  //TODO: grid motion pattern
  std::cerr << "Adding Frames" << '\n';
  Eigen::Affine3d TrueMotion = Eigen::Affine3d::Identity();
  TrueMotion.translation() << 0,1,0;

  while ((int)gridposes.size() < numNodes) {//TODO steps
    if ((int)gridposes.size() == numNodes)
      break;
    //TODO
    Frame nextpose = GenerateNewPose(gridposes.back(),
    TrueMotion.matrix(), TransNoise, RotNoise);//add noise
    gridposes.push_back(nextpose);
  }
};

void Simulator::AddingStaticLandmarks()
{
  //add landmarks static observation around the cameara poses
  LandmarkGrid grid;
  for (FrameVec::const_iterator it = gridposes.begin(); it != gridposes.end(); ++it)
  {
    int ccx = (int)round(it->transform.translation().x());
    int ccy = (int)round(it->transform.translation().y());
    for (int a=-landmarksRange; a<=landmarksRange; a++)
      for (int b=-landmarksRange; b<=landmarksRange; b++){
        int cx=ccx+a;
        int cy=ccy+b;
        PointPtrVec& landmarksForCell = grid[cx][cy];
        if (landmarksForCell.size()==0){
          for (int i = 0; i < landMarksPerSquareMeter; ++i) {
            Point* l = new Point(0);
            double offx, offy, offz;
            do {
              offx = Rand::gauss_rand(-0.5*stepLen, 0.5*stepLen);
              offy = Rand::gauss_rand(-0.5*stepLen, 0.5*stepLen);
              offz = Rand::gauss_rand(-0.5*stepLen, 0.5*stepLen);
            } while ((offx*offx+ offy*offy) < 0.25*0.25);//lower bound of distance
            l->truepos[0] = cx + offx;
            l->truepos[1] = cy + offy;
            l->truepos[2] = 0 + offz;
            landmarksForCell.push_back(l);
            Landmarks.push_back(l);
        }
      }
    }
  }

  double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it)
  {
    int cx = (int)round(it->transform.translation().x());
    int cy = (int)round(it->transform.translation().y());
    int numGridCells = (int)(maxSensorRangeLandmarks) + 1;
    Eigen::Affine3d trueInv = it->transform.inverse();

    for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
      for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
        PointPtrVec& landmarksForCell = grid[xx][yy];
        if (landmarksForCell.size() == 0)
          continue;
        for (size_t i = 0; i < landmarksForCell.size(); ++i) {
          Point* l = landmarksForCell[i];
          //check for observation range
          double dSqr = sqrt(it->transform.translation().x() - l->truepos[0]) +
                        sqrt(it->transform.translation().y() - l->truepos[1]);
          if (dSqr > maxSensorSqr)
            continue;
          if (l->idx == 0)
            l->idx = iGlobalId++;
          if (l->seenBy.size() == 0) {
            //consider the error of the VO
            Eigen::Vector3d trueObservation = trueInv * l->truepos;//get the refference pose the error just accumulate with distance
            Eigen::Vector3d observation = trueObservation;
            //observation noise for sensor
            observation[0] += Rand::gauss_rand(0., LandmarkNoise[0]);
            observation[1] += Rand::gauss_rand(0., LandmarkNoise[1]);
            observation[2] += Rand::gauss_rand(0., LandmarkNoise[2]);
            l->simulatedpos = it->simulatedtransform * observation;//TODO: add VO noise
          }
          l->seenBy.push_back(it->idx);
          it->landmarks.push_back(l);
        }
      }
  }
};

void Simulator::InitRigidEdgesFully(PointPtrVec& dynamic_points, int obj_id){
  for (PointPtrVec::iterator pt=dynamic_points.begin(); pt!=dynamic_points.end(); ++pt){
    PointPtrVec::iterator ptt = std::next(pt);
    while (ptt!=dynamic_points.end()){
      std::cerr << "pt: "<<(*pt)->idx<<"ptt: "<<(*ptt)->idx << '\n';
      RigidEdge edge;
      edge.obj_id = obj_id;
      double distance = ((*pt)->truepos - (*ptt)->truepos).norm();
      edge.vertex_id = iGlobalId++;
      edge.gtdistance = distance;
      vRigidEdges.push_back(edge);
      ptt++;
    }
  }
};

void Simulator::InitRigidEdgesTwo(PointPtrVec& dynamic_points, int obj_id){
  for (PointPtrVec::iterator pt=dynamic_points.begin(); pt!=dynamic_points.end(); ++pt){
    if (*pt != dynamic_points.back()){//a pointer while back is not
      RigidEdge edge;
      edge.obj_id = obj_id;
      double distance = ((*pt)->truepos - (*std::next(pt))->truepos).norm();
      edge.vertex_id = iGlobalId++;
      edge.gtdistance = distance;
      vRigidEdges.push_back(edge);
    }
    else{
      RigidEdge edge;
      edge.obj_id = obj_id;
      double distance = ((*pt)->truepos - (*dynamic_points.begin())->truepos).norm();
      edge.vertex_id = iGlobalId++;
      edge.gtdistance = distance;
      vRigidEdges.push_back(edge);
    }
  }
};

void Simulator::AddingRigidEdgeTwo(DyObject& dyobj){
  std::vector<RigidEdge>::iterator vedge = vRigidEdges.begin();
  for (PointPtrVec::iterator pt = dyobj.dylandmarks.begin(); pt != dyobj.dylandmarks.end(); ++pt){
    if (*pt != dyobj.dylandmarks.back()){
      rigidbody p; p.first_point = *pt; p.second_point = *std::next(pt);
      p.distance_id = vedge->vertex_id;
      dyobj.rigidbodypairs.push_back(p);
      vedge++;
    }
    else{
      rigidbody p; p.first_point = *pt; p.second_point = *(dyobj.dylandmarks.begin());
      p.distance_id = vedge->vertex_id;
      dyobj.rigidbodypairs.push_back(p);
      vedge++;
    }
  // add the rigid body rigidbodypairs
  }
}

void Simulator::AddingRigidEdgeFully(DyObject& dyobj){
  std::vector<RigidEdge>::iterator vedge = vRigidEdges.begin();
  for (PointPtrVec::iterator pt = dyobj.dylandmarks.begin(); pt != dyobj.dylandmarks.end(); ++pt){
    PointPtrVec::iterator ptt = std::next(pt);
    while (ptt!=dyobj.dylandmarks.end()){
      rigidbody p; p.first_point = *pt; p.second_point = *ptt;
      p.distance_id = vedge->vertex_id;
      dyobj.rigidbodypairs.push_back(p);
      ptt++;
      vedge++;
    }
  // add the rigid body rigidbodypairs
  }
}

void Simulator::AddingDynamicCircle()
{
  std::cerr << "Adding Dynamic points in circle" << '\n';
  std::vector<int> rigidids;
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    Eigen::Affine3d objmotion;
    Eigen::Matrix3d _R = euler_zyx_to_rot<double>(Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.3));
    objmotion.rotate(Eigen::AngleAxisd(_R));
    objmotion.translation()<<0+Rand::uniform_rand(-0.5, 0.5),1+Rand::uniform_rand(-0.5, 0.5),0+Rand::uniform_rand(-0.5, 0.5);
    // initiate the dynamic object vector
    DyObject dyobj;
    Point* temppoint;
    for (float angle(0.0); angle < 360.0; angle += dCircleAngle)
    {
      Point* l = new Point(0);
      l->truepos[0] = it->transform.translation().x() + std::cos (pcl::deg2rad(angle));
      l->truepos[1] = it->transform.translation().y() + sin (pcl::deg2rad(angle));
      l->truepos[2] = it->transform.translation().z();
      // l->truepos = objmotion * l->truepos;
      l->simulatedpos[0] =l->truepos[0] + Rand::gauss_rand(0., LandmarkNoise[0]);
      l->simulatedpos[1] =l->truepos[1] + Rand::gauss_rand(0., LandmarkNoise[1]);
      l->simulatedpos[2] =l->truepos[2] + Rand::gauss_rand(0., LandmarkNoise[2]);

      l->idx = iGlobalId++;
      l->seenBy.push_back(it->idx);
      dyobj.dylandmarks.push_back(l);
      dyobj.id = 1;
      vDynamicLandmarks.push_back(l);
    }
    //initialize the number of distance vertex
    if (it == gridposes.begin()){
      InitRigidEdgesFully(dyobj.dylandmarks,1);
      // InitRigidEdgesTwo(dyobj.dylandmarks,1);

      std::cerr << "debug" << '\n';

    }
    //TODO:add association

    AddingRigidEdgeFully(dyobj);
    // AddingRigidEdgeTwo(dyobj);
    it->seenedobjs.push_back(dyobj);
  }
}

Frame Simulator::GenerateNewPose(const Frame& prev, const Eigen::Matrix4d& Motion,
   Eigen::Vector3d transNoise,  Eigen::Vector3d& rotNoise)
{
  Eigen::Matrix4d newpose = Motion * prev.truepose;
  Frame nextPose(newpose, prev.idx + 1);
  nextPose.sampleNoiseTransform(TransNoise, RotNoise);
  return nextPose;
};