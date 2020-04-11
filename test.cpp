#include <iostream>
#include <chrono>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/eigen.hpp>

#include "matrix_utils.h"
#include "Frame.h"

int main()
{
  float theta = M_PI/4;
  Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
  //
  // Eigen::Vector3d eularangle(1.6,0,0);
  // Eigen::Vector3d xyz(1,0,0);
  // Frame Initframe(eularangle,xyz,1);

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;
  Eigen::Vector3d test; test << 0,0,0;
  std::cerr << test.squaredNorm() << '\n';

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

  // Eigen::Affine3f transform_2 << Initframe.mTwc
  // cv2eigen(Initframe.mTwc, Initframe);

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.translation() << std::endl;
  std::vector<int> v{10,20,30,40};
  std::vector<int>::iterator it;
  for (std::vector<int>::iterator itt=v.begin();itt != v.end();++itt){
    std::cerr << "itt"<<*itt << '\n';
    it = std::next(itt);
    while (it != v.end()){
      std::cerr << "id" <<*it<< '\n';
      it++;
    }
  }

  // std::cout << Initframe.mTwc << std::endl;

}
