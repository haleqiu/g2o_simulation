#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Point.h"
#include "Frame.h"
#include "Simulator.h"
#include "g2o_vertex_distance.h"
#include "g2o_edge_rigidbody.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

void EvaluateDistanceError(g2o::SparseOptimizer& optimizer, std::vector<RigidEdge>& vRigidEdges);
void EvaluatePointError(g2o::SparseOptimizer& optimizer,  PointPtrVec& vDynamicLandmarks);
void EvaluateEdgeError(g2o::SparseOptimizer& optimizer);


#endif
