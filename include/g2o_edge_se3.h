#ifndef G2O_EDGE_SE3_H
#define G2O_EDGE_SE3_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"

#include "g2o_vertex_se3.h"

class EdgeSE3PointXYZ : public g2o::BaseBinaryEdge<3, Vector3, VertexSE3, g2o::VertexSBAPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3PointXYZ(){
    information().setIdentity();
    J.fill(0);
    J.block<3,3>(0,0) = - Eigen::Matrix3d::Identity();
    // resizeParameters(1);
    };
    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    // return the error estimate as a 3-vector
    void computeError();
    void computeErrordebug();

    // virtual void linearizeOplus();

    // jacobian

    virtual void setMeasurement(const Vector3& m){//Matrix may be?
      _measurement = m;
    }
  private:
    Eigen::Matrix<double,3,9,Eigen::ColMajor> J;//number_t

  };

  // void EdgeSE3PointXYZ::linearizeOplus() {
  //     g2o::VertexSE3Expmap *cam = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  //     g2o::VertexSBAPointXYZ *vp = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[1]);
  //
  //     Vector3 Zcam = cam->estimate().inverse() * vp->estimate();//se3.inverse
  //
  //     //  J(0,3) = -0.0;
  //     J(0,4) = -2*Zcam(2);
  //     J(0,5) = 2*Zcam(1);
  //
  //     J(1,3) = 2*Zcam(2);
  //     //  J(1,4) = -0.0;
  //     J(1,5) = -2*Zcam(0);
  //
  //     J(2,3) = -2*Zcam(1);
  //     J(2,4) = 2*Zcam(0);
  //     //  J(2,5) = -0.0;
  //
  //     J.block<3,3>(0,6) = cam->estimate().inverse().rotation().toRotationMatrix();;
  //
  //     Eigen::Matrix<double,3,9,Eigen::ColMajor> Jhom = J;
  //
  //     _jacobianOplusXi = Jhom.block<3,6>(0,0);
  //     _jacobianOplusXj = Jhom.block<3,3>(0,6);
  //
  //     // std::cerr << "just linearized." << std::endl;
  //     // std::cerr << "_jacobianOplusXi:" << std::endl << _jacobianOplusXi << std::endl;
  //     // std::cerr << "_jacobianOplusXj:" << std::endl << _jacobianOplusXj << std::endl;
  //   }

void EdgeSE3PointXYZ::computeError() {
  //I don't need offset
  VertexSE3 *se3 = static_cast<VertexSE3*>(_vertices[0]);
  g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[1]);

  Vector3 perr = se3->estimate().inverse()*point->estimate();
  Vector3 obs(_measurement);
  _error = perr - obs;
}

void EdgeSE3PointXYZ::computeErrordebug() {
  //I don't need offset
  VertexSE3 *se3 = static_cast<VertexSE3*>(_vertices[0]);
  g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[1]);

  Vector3 perr = se3->estimate().inverse() * point->estimate();
  std::cerr << "inverse estimazation: "<<perr-_measurement << '\n';
  Vector3 obs(_measurement);
  _error = perr - obs;
}



#endif
