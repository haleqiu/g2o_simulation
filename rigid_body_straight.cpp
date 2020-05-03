#include "matrix_utils.h"
#include "Frame.h"
#include "Point.h"
#include "g2o_vertex_distance.h"
#include "g2o_edge_rigidbody.h"
#include "rand.h"
#include "Simulator.h"
#include "viewer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
// #include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std::chrono_literals;

double rigidbodyweight = 1;

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

  // Simulator grid_simulation();
  Simulator grid_simulation;
  grid_simulation.InitializeFrames();
  grid_simulation.AddingDynamicCircle();

  //setup solver
  //block size is 3
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
      std::cerr << "point vertex id: " <<(*pt)->idx<< '\n';
      optimizer.addVertex(vPoint1);
    }
  //adding distance vertex
  //adding rigidbody constraint
  for (FrameVec::iterator it = grid_simulation.gridposes.begin(); it != grid_simulation.gridposes.end(); ++it){
      for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
        for (RigidPairVec::iterator p = obj->rigidbodypairs.begin(); p != obj->rigidbodypairs.end(); ++p){
          Eigen::Vector3d distance_init = (p->first_point->truepos - p->second_point->truepos);
          Eigen::Vector3d distance_init_debug = (p->first_point->simulatedpos - p->second_point->simulatedpos);
          if (it == grid_simulation.gridposes.begin()){
            // Eigen::Vector3d distance_init = (p->first_point->simulatedpos - p->second_point->simulatedpos);

            VertexDistance* vDistance = new VertexDistance();
            vDistance->setEstimate(distance_init_debug);
            vDistance->setId(p->distance_id);
            optimizer.addVertex(vDistance);
          }
          EdgeRigidBody* e = new EdgeRigidBody();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->first_point->idx) ));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->second_point->idx) ));
    		  e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( optimizer.vertex(p->distance_id) ));

          Eigen::Vector3d landmark_sigma = grid_simulation.LandmarkNoise;
          Eigen::Matrix3d rigidbody_sigma =rigidbodyweight* landmark_sigma.cwiseProduct(landmark_sigma).asDiagonal();
          e->setInformation(rigidbody_sigma);
          optimizer.addEdge(e);
          e->computeError_debug();
          std::cerr << "e->compute_error_norm" << e->compute_error_norm()<<'\n';
      }
    }
  }

  std::cout<<"optimizers vertexes and edges  "<<optimizer.vertices().size()<<"  "<<optimizer.edges().size()<<std::endl;
  //distance_error before //TODO: by point
  double dMeanDistanceErrorBefore = 0;
  for (std::vector<RigidEdge>::iterator et = grid_simulation.vRigidEdges.begin(); et != grid_simulation.vRigidEdges.end();++et){
    VertexDistance* vetd = static_cast<VertexDistance*>(optimizer.vertex(et->vertex_id));
    double distance_error = abs(vetd->estimate().norm() - et->gtdistance);
    dMeanDistanceErrorBefore += distance_error;
  }
  double average = dMeanDistanceErrorBefore/grid_simulation.vRigidEdges.size();
  std::cout << "The average distance error before: " << average << endl;
  //point error
  double dMeanPointErrorBefore = 0;
  for (PointPtrVec::iterator pt = grid_simulation.vDynamicLandmarks.begin(); pt != grid_simulation.vDynamicLandmarks.end(); ++pt){
    g2o::VertexSBAPointXYZ* vetp = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
    double point_error = ((*pt)->truepos - vetp->estimate()).norm();
    dMeanPointErrorBefore += point_error;
  }
  double average_point_error_before = dMeanPointErrorBefore/grid_simulation.vDynamicLandmarks.size();
  std::cout << "The average point error before: " << average_point_error_before << endl;
  //rigid body constraint error
  double average_edge_error_before = 0;
  for (g2o::SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      EdgeRigidBody* e = dynamic_cast<EdgeRigidBody*>(*it);
      average_edge_error_before += e->compute_error_norm();
    }
  average_edge_error_before = average_edge_error_before/optimizer.edges().size();
  std::cout << "The average edge error before: " << average_edge_error_before << endl;

  optimizer.initializeOptimization();
  optimizer.optimize(1);

  double average_edge_error_after = 0;
  for (g2o::SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      EdgeRigidBody* e = dynamic_cast<EdgeRigidBody*>(*it);
      average_edge_error_after += e->compute_error_norm();
    }
  average_edge_error_after = average_edge_error_after/optimizer.edges().size();
  std::cout << "The average edge error after: " << average_edge_error_after << endl;

  g2o::VertexSBAPointXYZ* vet2 = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(2));
  //edge error

  double dMeanDistanceErrorAfter = 0;
  for (std::vector<RigidEdge>::iterator et = grid_simulation.vRigidEdges.begin(); et != grid_simulation.vRigidEdges.end();++et){
    VertexDistance* vetd = static_cast<VertexDistance*>(optimizer.vertex(et->vertex_id));
    double distance_error = abs(vetd->estimate().norm() - et->gtdistance);
    dMeanDistanceErrorAfter+=distance_error;
  }
  double averageafter = dMeanDistanceErrorAfter/grid_simulation.vRigidEdges.size();
  std::cout << "The average distance error after:" << averageafter << endl;

  double dMeanPointErrorAfter = 0;
  for (PointPtrVec::iterator pt = grid_simulation.vDynamicLandmarks.begin(); pt != grid_simulation.vDynamicLandmarks.end(); ++pt){
    g2o::VertexSBAPointXYZ* vetp = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
    double point_error = ((*pt)->truepos - vetp->estimate()).norm();
    dMeanPointErrorAfter += point_error;
  }
  double average_point_error_after = dMeanPointErrorAfter/grid_simulation.vDynamicLandmarks.size();
  std::cout << "The average point error after:" << average_point_error_after << endl;

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
    // visulizeFrames(gridposes,viewer);
    // VisulizeOptimized(gridposes,viewer);
  }
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

}
