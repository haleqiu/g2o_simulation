#ifndef G2O_UNARYEDGE_POINT_H
#define G2O_UNARYEDGE_POINT_H


#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

  class EdgeUnaryPointXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ> {
    public:
      // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeUnaryPointXYZ(){};
      virtual bool read(std::istream& is){ return true; };
      virtual bool write(std::ostream& os) const {return os.good(); };

      void computeError();
      virtual void setMeasurement(const Eigen::Vector3d& m){
        //The measurement shall be the 3D position of the vertex point
        _measurement = m;
      }

      virtual int measurementDimension() const {return 3;}

    private:
    };

void EdgeUnaryPointXYZ::computeError() {
  g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);

  _error = point->estimate() - _measurement;
}


#endif
