#ifndef G2O_EDGE_RIGIDBODY_H
#define G2O_EDGE_RIGIDBODY_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "g2o_vertex_distance.h"

// class EdgeRigidBody : public g2o::BaseMultiEdge<3, double>
class EdgeRigidBody: public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRigidBody(){resize(3);};

    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    void computeError();
    void computeError_debug();

    double delta_t; //to_time - from_time   positive
    double compute_error_norm();

};

void EdgeRigidBody::computeError_debug()
{
  const g2o::VertexSBAPointXYZ* pointVertexfrom = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);	   	// first point Vector 3d
  const g2o::VertexSBAPointXYZ* pointVertexto = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);	   	// next point
  const VertexDistance* distanceVertex = dynamic_cast<const VertexDistance*>(_vertices[2]); // object to world pose

  if (pointVertexfrom==nullptr||pointVertexto==nullptr||distanceVertex==nullptr)
      std::cout<<"bad casting!!!!!!!!!!!!!"<<std::endl;
  std::cerr << "pointVertexfrom->estimate()"<<pointVertexfrom->estimate() << '\n';
  std::cerr << "pointVertexto->estimate()"<<pointVertexto->estimate() << '\n';

  Eigen::Vector3d subtract = pointVertexfrom->estimate()-pointVertexto->estimate();

  double _distance = subtract.norm();
  std::cerr << "_distance" <<_distance<< '\n';
  std::cerr << "eroror" << distanceVertex->estimate().norm() - _distance<<'\n';;
};

double EdgeRigidBody::compute_error_norm(){
  computeError();
  Eigen::Vector3d err = _error;
  return err.norm();
}

void EdgeRigidBody::computeError(){
  const g2o::VertexSBAPointXYZ* pointVertexfrom = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);	   	// first point Vector 3d
  const g2o::VertexSBAPointXYZ* pointVertexto = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);	   	// next point
  const VertexDistance* distanceVertex = dynamic_cast<const VertexDistance*>(_vertices[2]); // object to world pose

  if (pointVertexfrom==nullptr||pointVertexto==nullptr||distanceVertex==nullptr)
      std::cout<<"bad casting!!!!!!!!!!!!!"<<std::endl;

  Eigen::Vector3d subtract = pointVertexfrom->estimate()-pointVertexto->estimate();

  double _distance = subtract.norm();

  // _error[0] =  _distance-distanceVertex->estimate().norm();
  _error = distanceVertex->estimate() - subtract;

};

class EdgeRigidBodyDouble: public g2o::BaseMultiEdge<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRigidBodyDouble(){resize(3);};

    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    void computeError();
    void computeError_debug();

    double delta_t; //to_time - from_time   positive
    double compute_error_norm();

};

void EdgeRigidBodyDouble::computeError_debug()
{
  const g2o::VertexSBAPointXYZ* pointVertexfrom = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);	   	// first point Vector 3d
  const g2o::VertexSBAPointXYZ* pointVertexto = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);	   	// next point
  const VertexDistanceDouble* distanceVertex = dynamic_cast<const VertexDistanceDouble*>(_vertices[2]); // object to world pose

  if (pointVertexfrom==nullptr||pointVertexto==nullptr||distanceVertex==nullptr)
      std::cout<<"bad casting!!!!!!!!!!!!!"<<std::endl;
  std::cerr << "pointVertexfrom->estimate()"<<"\n"<<pointVertexfrom->estimate() << '\n';
  std::cerr << "pointVertexto->estimate()"<<"\n"<<pointVertexto->estimate() << '\n';

  Eigen::Vector3d subtract = pointVertexfrom->estimate()-pointVertexto->estimate();

  double _distance = subtract.norm();
  std::cerr << "_distance: " <<_distance<< '\n';
  std::cerr << "error: " << distanceVertex->estimate() - _distance<<'\n';;
};

double EdgeRigidBodyDouble::compute_error_norm(){
  computeError();
  double err = _error[0];
  return err;
}

void EdgeRigidBodyDouble::computeError(){
  const g2o::VertexSBAPointXYZ* pointVertexfrom = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);	   	// first point Vector 3d
  const g2o::VertexSBAPointXYZ* pointVertexto = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);	   	// next point
  const VertexDistanceDouble* distanceVertex = dynamic_cast<const VertexDistanceDouble*>(_vertices[2]); // object to world pose

  if (pointVertexfrom==nullptr||pointVertexto==nullptr||distanceVertex==nullptr)
      std::cout<<"bad casting!!!!!!!!!!!!!"<<std::endl;

  Eigen::Vector3d subtract = pointVertexfrom->estimate()-pointVertexto->estimate();

  double _distance = subtract.norm();

  _error[0] =  _distance-distanceVertex->estimate();
};

#endif
