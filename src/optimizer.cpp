#include "optimizer.h"

void EvaluateDistanceError(g2o::SparseOptimizer& optimizer, std::vector<RigidEdge>& vRigidEdges, const std::string words ){
  std::cout<<"optimizers vertexes and edges  "<<optimizer.vertices().size()<<"  "<<optimizer.edges().size()<<std::endl;
  //distance_error before //TODO: by point
  double dMeanDistanceErrorBefore = 0;
  for (std::vector<RigidEdge>::iterator et = vRigidEdges.begin(); et != vRigidEdges.end();++et){
    VertexDistanceDouble* vetd = static_cast<VertexDistanceDouble*>(optimizer.vertex(et->vertex_id));
    double distance_error = abs(vetd->estimate() - et->gtdistance);
    dMeanDistanceErrorBefore += distance_error;
  }
  double average = dMeanDistanceErrorBefore/vRigidEdges.size();
  std::cout << "Distance error "<<words<< ": " << average << endl;
}

void EvaluatePointError(g2o::SparseOptimizer& optimizer, PointPtrVec& vLandmarks,  const std::string words ){
  double dMeanPointErrorBefore = 0;
  for (PointPtrVec::iterator pt = vLandmarks.begin(); pt != vLandmarks.end(); ++pt){
    g2o::VertexSBAPointXYZ* vetp = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*pt)->idx));
    double point_error = ((*pt)->truepos - vetp->estimate()).norm();
    dMeanPointErrorBefore += point_error;
  }
  double average_point_error_before = dMeanPointErrorBefore/vLandmarks.size();
  std::cout << "Point error "<<words<< ": " << average_point_error_before << endl;
}

void EvaluateCaemraPoseError(g2o::SparseOptimizer& optimizer,FrameVec& vFrame, const std::string words ){
  double average_translation_error = 0;
  for (FrameVec::iterator pt = vFrame.begin(); pt!=vFrame.end();++pt){
    VertexSE3* vSE3 = static_cast<VertexSE3*>(optimizer.vertex(pt->idx));
    double translation_error = (pt->transform.translation() - vSE3->estimate().translation()).norm();
    average_translation_error+=translation_error;
  }
  average_translation_error = average_translation_error/vFrame.size();
  std::cout << "Translation error "<<words<< ": " << average_translation_error << endl;
}
