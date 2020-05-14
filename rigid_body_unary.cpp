#include "matrix_utils.h"
#include "Frame.h"
#include "Point.h"
#include "g2o_vertex_distance.h"
#include "g2o_edge_rigidbody.h"
#include "rand.h"
#include "Simulator.h"
#include "viewer.h"
#include "g2o_unaryedge_point.h"
#include "optimizer.h"

using namespace std::chrono_literals;

double rigidbodyweight = 0.01;
std::vector<EdgeUnaryPointXYZ*> vpEdgesUnaryPoint;

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


  std::string strSettingsFile = "../rigid_distance.yaml";
  // Simulator grid_simulation();
  Simulator grid_simulation(strSettingsFile);
  grid_simulation.InitializeFrames();
  grid_simulation.AddingDynamicShape();

  //setup solver
  g2o::SparseOptimizer optimizer;
  std::cerr << "Setting up solver and optimizer" << '\n';
  g2o::BlockSolverX::LinearSolverType* linearSolver; // BlockSolverX instead of BlockSolver63
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // adding vDynamicLandmarks to your graph
  for (PointPtrVec::iterator pt = grid_simulation.vDynamicLandmarks.begin(); pt != grid_simulation.vDynamicLandmarks.end(); ++pt){
      g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
      vPoint1->setEstimate((*pt)->simulatedpos);
      vPoint1->setId((*pt)->idx);
      if (grid_simulation.mbDebug){std::cerr << "adding dynamic: " <<(*pt)->idx <<'\n';}
      optimizer.addVertex(vPoint1);
    }
  //adding unary edge to the point vertex
  //known VO only
  for (PointPtrVec::iterator pt = grid_simulation.vDynamicLandmarks.begin(); pt != grid_simulation.vDynamicLandmarks.end(); ++pt){
      EdgeUnaryPointXYZ* e = new EdgeUnaryPointXYZ();
      Eigen::Vector3d estimation = (*pt)->simulatedpos;
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex((*pt)->idx)));
      e->setMeasurement(estimation);
      if (grid_simulation.mbDebug){std::cerr << "adding dynamic: " <<(*pt)->idx <<'\n';}
      optimizer.addEdge(e);
      vpEdgesUnaryPoint.push_back(e);

      Eigen::Vector3d landmark_sigma = grid_simulation.LandmarkNoise;
      Eigen::Matrix3d rigidbody_sigma =rigidbodyweight* landmark_sigma.cwiseProduct(landmark_sigma).asDiagonal();
      e->setInformation(rigidbody_sigma);
    }
  //adding distance vertex
  //adding rigidbody constraint
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
          Info(0,0) = 0.05;//LandmarkNoise(0);

          e->setInformation(Info);
          optimizer.addEdge(e);
      }
    }
  }

  EvaluateDistanceError(optimizer, grid_simulation.vRigidEdges);
  EvaluatePointError(optimizer, grid_simulation.vDynamicLandmarks);

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  EvaluateDistanceError(optimizer, grid_simulation.vRigidEdges);
  EvaluatePointError(optimizer, grid_simulation.vDynamicLandmarks);


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
    simulatedCircleVis(grid_simulation.gridposes,viewer);
    optimizedCircleVis(grid_simulation.gridposes,viewer);
    gtCircleVis(grid_simulation.gridposes,viewer);
    // rigidBodyEdgeVis(grid_simulation.gridposes,viewer);
    visulizeFrames(grid_simulation.gridposes,viewer);
    // VisulizeOptimized(gridposes,viewer);
  }
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

}
