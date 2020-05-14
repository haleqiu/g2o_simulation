#include "matrix_utils.h"
#include "Frame.h"
#include "Point.h"
#include "g2o_vertex_distance.h"
#include "g2o_vertex_se3.h"
#include "g2o_edge_rigidbody.h"
#include "rand.h"
#include "Simulator.h"
#include "viewer.h"
#include "g2o_edge_se3.h"
#include "optimizer.h"

using namespace std::chrono_literals;

std::vector<EdgeSE3PointXYZ*> vpEdgesSE3Point;

int main (int argc, char** argv){
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), simulated(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false);

  if (pcl::console::find_argument (argc, argv, "-t") > 0)
  {
    std::cout << "output the ground truth\n";
  }

  else if (pcl::console::find_argument (argc, argv, "-s") > 0)
  {
    simulated = true;
    std::cout << "output the simulated pose\n";
  }

  else
  {
    printUsage (argv[0]);
    return 0;
  }

  Rand::seed_rand(1);
  std::string strSettingsFile = "../rigid_distance.yaml";
  // Simulator grid_simulation();
  Simulator grid_simulation(strSettingsFile);
  grid_simulation.InitializeFrames();
  grid_simulation.AddingStaticLandmarks();
  grid_simulation.AddingDynamicShape();

  //setup solver
  g2o::SparseOptimizer optimizer;
  std::cerr << "Setting up solver and optimizer" << '\n';
  g2o::BlockSolverX::LinearSolverType* linearSolver; // BlockSolverX instead of BlockSolver63
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);


  for (PointPtrVec::iterator pt = grid_simulation.Landmarks.begin(); pt != grid_simulation.Landmarks.end(); ++pt){
      g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
      Vector3 obs((*pt)->simulatedpos);
      vPoint1->setEstimate(obs);
      vPoint1->setId((*pt)->idx);
      if (grid_simulation.mbDebug){std::cerr << "adding static: " <<(*pt)->idx <<'\n';}
      optimizer.addVertex(vPoint1);
    }

    // adding vDynamicLandmarks to your graph
  for (PointPtrVec::iterator pt = grid_simulation.vDynamicLandmarks.begin(); pt != grid_simulation.vDynamicLandmarks.end(); ++pt){
      g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
      vPoint1->setEstimate((*pt)->simulatedpos);
      vPoint1->setId((*pt)->idx);
      if (grid_simulation.mbDebug){std::cerr << "adding dynamic: " <<(*pt)->idx <<'\n';}
      optimizer.addVertex(vPoint1);
    }
  //adding Frame vector

  for (FrameVec::iterator ft = grid_simulation.gridposes.begin(); ft != grid_simulation.gridposes.end(); ++ft){
      VertexSE3* vSE3 = new VertexSE3();
      Isometry3 estimation(ft->simulatedtransform.matrix());
      vSE3->setEstimate(estimation);
      vSE3->setId(ft->idx);
      if (grid_simulation.mbDebug){std::cerr << "adding frames: " <<ft->idx <<'\n';}
      if (ft == grid_simulation.gridposes.begin()){
        vSE3->setFixed(true);
      }
      optimizer.addVertex(vSE3);
  }
//ADDING RIGID body constraint
  for (FrameVec::iterator it = grid_simulation.gridposes.begin(); it != grid_simulation.gridposes.end(); ++it){
      for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
        for (RigidPairVec::iterator p = obj->rigidbodypairs.begin(); p != obj->rigidbodypairs.end(); ++p){
          Eigen::Vector3d distance_init;
          if (grid_simulation.mbDebug){std::cerr << "first_point: " <<p->first_point->idx<< " second_point: " <<p->second_point->idx<<" distance_id: "<<p->distance_id<< '\n';}
          if (grid_simulation.mbDistanceGroundTruthInit){
            distance_init = (p->first_point->truepos - p->second_point->truepos);
          }
          else{
            distance_init = (p->first_point->simulatedpos - p->second_point->simulatedpos);
          }
          if (it == grid_simulation.gridposes.begin()){
            VertexDistanceDouble* vDistance = new VertexDistanceDouble();
            vDistance->setEstimate(distance_init.norm());
            vDistance->setId(p->distance_id);
            optimizer.addVertex(vDistance);
            if (grid_simulation.mbDebug){std::cerr << "adding distance: " <<p->distance_id <<'\n';}
          }
          EdgeRigidBodyDouble* e = new EdgeRigidBodyDouble();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->first_point->idx) ));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->second_point->idx) ));
          e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->distance_id) ));

          // Eigen::Vector3d landmark_sigma = grid_simulation.LandmarkNoise;
          // Eigen::Matrix3d rigidbody_sigma =rigidbodyweight* landmark_sigma.cwiseProduct(landmark_sigma).asDiagonal();
          // e->setInformation(rigidbody_sigma);

          Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(1,1);
          Info(0,0) = grid_simulation.rigidbodyweight * grid_simulation.LandmarkNoise(0);
          e->setInformation(Info);
          optimizer.addEdge(e);
      }
    }
  }

  // //adding dynamic vertex and se3 edge
  // for (FrameVec::iterator ft = grid_simulation.gridposes.begin(); ft != grid_simulation.gridposes.end(); ++ft){
  //   for (ObjectVec::iterator obj = ft->seenedobjs.begin(); obj != ft->seenedobjs.end(); ++obj){
  //     for (PointMeasurementVec::iterator ptp = obj->pointmeasurements.begin();ptp != obj->pointmeasurements.end();++ptp){
  //       EdgeSE3PointXYZ* e = new EdgeSE3PointXYZ();
  //       // Eigen::Vector3 estimation = (*pt)->simulatedpos;
  //       Eigen::Vector3d estimation = ptp->observation;
  //       e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(ft->idx)));
  //       e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(ptp->point_id)));
  //       Vector3 obs(estimation);
  //       e->setMeasurement(obs);
  //       if (grid_simulation.mbDebug){std::cerr << "adding se3 edge: " <<ft->idx<<"point: " <<ptp->point_id <<'\n';}
  //       Eigen::Vector3d landmark_sigma = grid_simulation.LandmarkNoise;
  //       Eigen::Matrix3d rigidbody_sigma = landmark_sigma.cwiseProduct(landmark_sigma).asDiagonal();
  //       e->setInformation(rigidbody_sigma);
  //       optimizer.addEdge(e);
  //       vpEdgesSE3Point.push_back(e);
  //       }
  //     }
  //   }

  //adding static vertex and se3 edge
  for (FrameVec::iterator ft = grid_simulation.gridposes.begin(); ft != grid_simulation.gridposes.end(); ++ft){
    for (PointMeasurementVec::iterator ptp = ft->vPointMeasurements.begin();ptp != ft->vPointMeasurements.end();++ptp){
      EdgeSE3PointXYZ* e = new EdgeSE3PointXYZ();
      // Eigen::Vector3 estimation = (*pt)->simulatedpos;
      Eigen::Vector3d estimation = ptp->observation;
      e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(ft->idx)));
      e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(ptp->point_id)));
      Vector3 obs(estimation);
      e->setMeasurement(obs);
      if (grid_simulation.mbDebug){std::cerr << "adding se3 edge: " <<ft->idx<<"point: " <<ptp->point_id <<'\n';}
      Eigen::Vector3d landmark_sigma = grid_simulation.LandmarkNoise;
      Eigen::Matrix3d rigidbody_sigma = grid_simulation.pointmeasurementweight * landmark_sigma.cwiseProduct(landmark_sigma).asDiagonal();
      e->setInformation(rigidbody_sigma);
      optimizer.addEdge(e);
      vpEdgesSE3Point.push_back(e);
      }
    }
  std::cerr << "static landmars"<< grid_simulation.Landmarks.size()<< '\n';
  std::cerr << "dynamic landmars"<< grid_simulation.vDynamicLandmarks.size()<< '\n';

  EvaluatePointError(optimizer, grid_simulation.Landmarks, "static before");
  EvaluatePointError(optimizer, grid_simulation.vDynamicLandmarks, "dynamic before");
  EvaluateCaemraPoseError(optimizer,grid_simulation.gridposes, "before");
  EvaluateDistanceError(optimizer,grid_simulation.vRigidEdges, "before");

  optimizer.initializeOptimization();
  optimizer.optimize(grid_simulation.iterationsteps);

  EvaluatePointError(optimizer, grid_simulation.Landmarks, "static");
  EvaluatePointError(optimizer, grid_simulation.vDynamicLandmarks, "dynamic");
  EvaluateCaemraPoseError(optimizer,grid_simulation.gridposes);
  EvaluateDistanceError(optimizer,grid_simulation.vRigidEdges);

  for (FrameVec::iterator it = grid_simulation.gridposes.begin(); it != grid_simulation.gridposes.end(); ++it){
    VertexSE3* SE3 = static_cast<VertexSE3*>(optimizer.vertex(it->idx));
    Eigen::Affine3d optimizedtransform;
    optimizedtransform.matrix() = SE3->estimate().matrix();
    it->optimizedtransform = optimizedtransform;
  }

  //dumping the static point into the visualization
  for (PointPtrVec::iterator pt = grid_simulation.Landmarks.begin(); pt!=grid_simulation.Landmarks.end();++pt){
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
    (*pt)->optimizedpos = vPoint->estimate();
  }

  //dumping the optimized data into the visualization
  for (FrameVec::iterator it = grid_simulation.gridposes.begin(); it != grid_simulation.gridposes.end(); ++it){
      for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
        for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          g2o::VertexSBAPointXYZ* vetp = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
          (*pt)->optimizedpos = vetp->estimate();
        }
      }
  }

  // visualisation
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (simulated)
  {
    viewer = Initiateviewer();

    pointMeasurementVis(grid_simulation.gridposes, viewer);
    simulatedCircleVis(grid_simulation.gridposes,viewer);
    optimizedCircleVis(grid_simulation.gridposes,viewer);
    // simulatedCircleVis(grid_simulation.gridposes,viewer);
    // optimizedCircleVis(grid_simulation.gridposes,viewer);
    gtCircleVis(grid_simulation.gridposes,viewer);
    // rigidBodyEdgeVis(grid_simulation.gridposes,viewer);
    visulizeFrames(grid_simulation.gridposes,viewer);
    pointMeasuremenOptimizedtVis(grid_simulation.gridposes,viewer);
    // VisulizeOptimized(gridposes,viewer);
  }
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

}
