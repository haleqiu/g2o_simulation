#include "Simulator.h"


Simulator::Simulator(){
  TransNoise << 0,0,0;
  RotNoise << 0,0,0;
  LandmarkNoise << 0.05,0.05,0.05;
  std::cerr << "LandmarkNoise: "<<LandmarkNoise << '\n';
};

Simulator::Simulator(const std::string &strSettingsFile){
  TransNoise << 0,0,0;
  RotNoise << 0,0,0;
  LandmarkNoise << 0.05,0.05,0.05;
  std::cerr << "LandmarkNoise: "<<LandmarkNoise << '\n';

  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
     cerr << "Failed to open settings file at: " << strSettingsFile << endl;
     exit(-1);
  }

  numNodes = fsSettings["Simulator.numNodes"];
  steps = fsSettings["Simulator.steps"];//TODO
  stepLen = fsSettings["Simulator.stepLen"];
  boundArea = fsSettings["Simulator.boundArea"];//TODO
  landmarksRange = fsSettings["Simulator.landmarksRange"];
  maxSensorRangeLandmarks = 2.5 * stepLen;//it decide how many
  landMarksPerSquareMeter = fsSettings["Simulator.landMarksPerSquareMeter"];

  // FALG
  mbDistanceGroundTruthInit = fsSettings["Optimizer.DistanceGroundTruthInit"];
  mbDebug = fsSettings["Optimizer.Debug"];
  mbStraightMotion = fsSettings["Simulator.StraightMotion"];

  //Optimizer
  Dynum = fsSettings["Optimizer.Dynum"];
  DyVertexNum = fsSettings["Optimizer.vertexnum"];

};

void Simulator::InitializeFrames()
{
  //first pose initialization
  Frame firstpose(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0),iGlobalId++);
  gridposes.push_back(firstpose);
  //TODO: grid motion pattern
  std::cerr << "Adding Frames" << '\n';
  //adding straight motion
  Eigen::Affine3d TrueMotion = Eigen::Affine3d::Identity();

  if (mbStraightMotion){
    TrueMotion.translation() << 0,stepLen,0;
  }


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
          double dSqr = pow(it->transform.translation().x() - l->truepos[0], 2) +
                        pow(it->transform.translation().y() - l->truepos[1], 2);
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
  //adding observatio
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    for (PointPtrVec::iterator pt = it->landmarks.begin(); pt != it->landmarks.end(); ++pt){
      Eigen::Vector3d trueObservation = it->transform.inverse() * (*pt)->truepos;
      Eigen::Vector3d Observation;
      Observation = trueObservation;
      if ((*pt)->seenBy[0] == it->idx){//first estimation
        Observation = it->simulatedtransform.inverse() * (*pt)->simulatedpos;//vector based measurement
      }
      else{
        Observation[0] += Rand::gauss_rand(0., LandmarkNoise[0]);
        Observation[1] += Rand::gauss_rand(0., LandmarkNoise[1]);
        Observation[2] += Rand::gauss_rand(0., LandmarkNoise[2]);
      }
      PointMeasurement Pm = {Observation,(*pt)->idx};
      it->vPointMeasurements.push_back(Pm);
    }
  }

};

void Simulator::InitRigidEdgesFully(PointPtrVec& dynamic_points, int obj_id){
  for (PointPtrVec::iterator pt=dynamic_points.begin(); pt!=dynamic_points.end(); ++pt){
    PointPtrVec::iterator ptt = std::next(pt);
    while (ptt!=dynamic_points.end()){
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
void Simulator::InitRigidEdgesThree(PointPtrVec& dynamic_points, int obj_id){
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

void Simulator::InitDynamicShape(){
  //init the triangular
  std::cerr << "Init the shape of triangular" << '\n';
  double trianglelength = 2;
  int lengthnumber = Dynum/3;
  double d = trianglelength / lengthnumber;

  for (int len=0; len < lengthnumber; len++){
    Point* l = new Point(-1);
    l->truepos[0] = std::cos(pcl::deg2rad(60.0))*(len*d);
    l->truepos[1] = std::sin(pcl::deg2rad(60.0))*(trianglelength - len*d);
    l->truepos[2] = 0;
    vDynamicShape.push_back(l);
  }
  for (int len=0; len < lengthnumber; len++){
    Point* l = new Point(-1);
    l->truepos[0] = std::cos(pcl::deg2rad(60.0))*trianglelength-len*d;
    l->truepos[1] = 0;
    l->truepos[2] = 0;
    vDynamicShape.push_back(l);
  }
  for (int len=0; len < lengthnumber; len++){
    Point* l = new Point(-1);
    l->truepos[0] = std::cos(pcl::deg2rad(60.0))*(len*d-trianglelength);
    l->truepos[1] = std::sin(pcl::deg2rad(60.0))*(len*d);
    l->truepos[2] = 0;
    vDynamicShape.push_back(l);
  }
}

void Simulator::AddingDynamicShape(){
  InitDynamicShape();
  std::cerr << "Adding Dynamic shape" << '\n';
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    Eigen::Affine3d objmotion;
    Eigen::Matrix3d _R = euler_zyx_to_rot<double>(Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.3));
    objmotion.rotate(Eigen::AngleAxisd(_R));
    objmotion.translation()<<0+Rand::uniform_rand(-0.5, 0.5),1+Rand::uniform_rand(-0.5, 0.5),0+Rand::uniform_rand(-0.5, 0.5);
    // initiate the dynamic object vector
    DyObject dyobj;
    for (PointPtrVec::iterator pt = vDynamicShape.begin(); pt != vDynamicShape.end();++pt){
      Point* l = new Point(0);
      l->truepos[0] = it->transform.translation().x() + (*pt)->truepos[0];
      l->truepos[1] = it->transform.translation().y() + (*pt)->truepos[1];
      l->truepos[2] = it->transform.translation().z() + (*pt)->truepos[2];
      // l->truepos = objmotion * l->truepos;
      l->simulatedpos[0] =l->truepos[0] + Rand::gauss_rand(0., LandmarkNoise[0]);
      l->simulatedpos[1] =l->truepos[1] + Rand::gauss_rand(0., LandmarkNoise[1]);
      l->simulatedpos[2] =l->truepos[2] + Rand::gauss_rand(0., LandmarkNoise[2]);

      l->idx = iGlobalId++;
      l->seenBy.push_back(it->idx);
      dyobj.dylandmarks.push_back(l);
      dyobj.id = 2;
      vDynamicLandmarks.push_back(l);
    }
    //initialize the number of distance vertex
    if (it == gridposes.begin()){
      if(DyVertexNum > 4) InitRigidEdgesFully(dyobj.dylandmarks,1);
      else InitRigidEdgesTwo(dyobj.dylandmarks,1);
    }
    //TODO:add association
    if(DyVertexNum > 4){
      AddingRigidEdgeFully(dyobj);
    }
    else{
      AddingRigidEdgeTwo(dyobj);
    }
    it->seenedobjs.push_back(dyobj);
  }
}

void Simulator::AddingDynamicCircle()
{
  std::cerr << "Adding Dynamic points in circle" << '\n';
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    Eigen::Affine3d objmotion;
    Eigen::Matrix3d _R = euler_zyx_to_rot<double>(Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.1), Rand::uniform_rand(-0.1, 0.3));
    objmotion.rotate(Eigen::AngleAxisd(_R));
    objmotion.translation()<<0+Rand::uniform_rand(-0.5, 0.5),1+Rand::uniform_rand(-0.5, 0.5),0+Rand::uniform_rand(-0.5, 0.5);
    // initiate the dynamic object vector
    DyObject dyobj;
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
  Frame nextPose(newpose, iGlobalId++);
  nextPose.sampleNoiseTransform(TransNoise, RotNoise);
  return nextPose;
};
