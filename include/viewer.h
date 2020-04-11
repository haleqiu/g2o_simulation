#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>

int iViewerIdx = 0;

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-r           RGB colour visualisation example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
};

pcl::visualization::PCLVisualizer::Ptr Initiateviewer()
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  return (viewer);
};

void rigidBodyEdgeVis(FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer){
  // visualize the rigid body constrain
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    pcl::PointXYZ camera_pose(it->transform.translation().x(),it->transform.translation().y(),it->transform.translation().z());
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (RigidPairVec::iterator p = obj->rigidbodypairs.begin(); p != obj->rigidbodypairs.end(); ++p){
        pcl::PointXYZ first_point(p->first_point->simulatedpos[0],p->first_point->simulatedpos[1],p->first_point->simulatedpos[2]);
        pcl::PointXYZ second_point(p->second_point->simulatedpos[0],p->second_point->simulatedpos[1],p->second_point->simulatedpos[2]);
        std::string idstring = std::to_string(iViewerIdx++);
        viewer->addLine<pcl::PointXYZ> (first_point,second_point, 1,0,0,idstring,0);
      }
    }
  }
}

void simulatedCircleVis (FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Dynamic shape visualziation.\n";

  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    pcl::PointXYZ camera_pose(it->transform.translation().x(),it->transform.translation().y(),it->transform.translation().z());
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          pcl::PointXYZ basic_point((*pt)->simulatedpos[0],(*pt)->simulatedpos[1],(*pt)->simulatedpos[2]);
          basic_cloud_ptr->points.push_back(basic_point);
          std::string idstring = std::to_string(iViewerIdx++);
          viewer->addLine<pcl::PointXYZ> (basic_point,camera_pose, idstring);
          //plot the shape
          if (*pt != obj->dylandmarks.back()){
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*std::next(pt))->simulatedpos[0],(*std::next(pt))->simulatedpos[1],(*std::next(pt))->simulatedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 0,1,0,idstring,0);
          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->simulatedpos[0],(*obj->dylandmarks.begin())->simulatedpos[1],(*obj->dylandmarks.begin())->simulatedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 0,1,0,idstring,0);
          }
        }
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "Dynamic shape cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Dynamic shape cloud");
  //If you want to update a point cloud that is already displayed, you must first call removePointCloud() and provide the ID of the cloud that is to be updated.
}

void optimizedCircleVis (FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Dynamic optimzed shape visualziation.\n";
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          pcl::PointXYZ basic_point((*pt)->optimizedpos[0],(*pt)->optimizedpos[1],(*pt)->optimizedpos[2]);
          basic_cloud_ptr->points.push_back(basic_point);
          std::string idstring = std::to_string(iViewerIdx++);
          //plot the shape
          if (*pt != obj->dylandmarks.back()){
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*std::next(pt))->optimizedpos[0],(*std::next(pt))->optimizedpos[1],(*std::next(pt))->optimizedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,1,0,idstring,0);
          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->optimizedpos[0],(*obj->dylandmarks.begin())->optimizedpos[1],(*obj->dylandmarks.begin())->optimizedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,1,0,idstring,0);
          }
        }
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "Dynamic shape optimized cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Dynamic shape cloud");
  //If you want to update a point cloud that is already displayed, you must first call removePointCloud() and provide the ID of the cloud that is to be updated.
}

void gtCircleVis (FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Dynamic optimzed shape visualziation.\n";
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          pcl::PointXYZ basic_point((*pt)->truepos[0],(*pt)->truepos[1],(*pt)->truepos[2]);
          basic_cloud_ptr->points.push_back(basic_point);
          std::string idstring = std::to_string(iViewerIdx++);
          //plot the shape
          if (*pt != obj->dylandmarks.back()){
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*std::next(pt))->truepos[0],(*std::next(pt))->truepos[1],(*std::next(pt))->truepos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,0);
          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->truepos[0],(*obj->dylandmarks.begin())->truepos[1],(*obj->dylandmarks.begin())->truepos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,0);
          }
        }
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "Dynamic shape optimized cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Dynamic shape cloud");
  //If you want to update a point cloud that is already displayed, you must first call removePointCloud() and provide the ID of the cloud that is to be updated.
}


#endif
