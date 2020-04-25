#include "optimizer.h"

void EvaluateDistanceError(g2o::SparseOptimizer& optimizer, std::vector<RigidEdge>& vRigidEdges){
  std::cout<<"optimizers vertexes and edges  "<<optimizer.vertices().size()<<"  "<<optimizer.edges().size()<<std::endl;
  //distance_error before //TODO: by point
  double dMeanDistanceErrorBefore = 0;
  for (std::vector<RigidEdge>::iterator et = vRigidEdges.begin(); et != vRigidEdges.end();++et){
    VertexDistanceDouble* vetd = static_cast<VertexDistanceDouble*>(optimizer.vertex(et->vertex_id));
    double distance_error = abs(vetd->estimate() - et->gtdistance);
    dMeanDistanceErrorBefore += distance_error;
  }
  double average = dMeanDistanceErrorBefore/vRigidEdges.size();
  std::cout << "The average distance error before: " << average << endl;
}

void EvaluatePointError(g2o::SparseOptimizer& optimizer, PointPtrVec& vLandmarks){
  double dMeanPointErrorBefore = 0;
  for (PointPtrVec::iterator pt = vLandmarks.begin(); pt != vLandmarks.end(); ++pt){
    g2o::VertexSBAPointXYZ* vetp = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
    double point_error = ((*pt)->truepos - vetp->estimate()).norm();
    dMeanPointErrorBefore += point_error;
  }
  double average_point_error_before = dMeanPointErrorBefore/vLandmarks.size();
  std::cout << "The average point error before: " << average_point_error_before << endl;
}

void EvaluateEdgeError(g2o::SparseOptimizer& optimizer){
//Simple form
  double average_edge_error_after = 0;
  for (g2o::SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      EdgeRigidBodyDouble* e = static_cast<EdgeRigidBodyDouble*>(*it);
      average_edge_error_after += e->compute_error_norm();
    }
  average_edge_error_after = average_edge_error_after/optimizer.edges().size();
  std::cout << "The average edge error after: " << average_edge_error_after << endl;
}
