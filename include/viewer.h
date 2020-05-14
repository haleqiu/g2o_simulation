#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>

using namespace std::chrono_literals;

int iViewerIdx = 0;
std::string idstring;
int v1(0);
int v2(0);

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
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr simu_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Dynamic shape visualziation.\n";

  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    pcl::PointXYZ camera_pose(it->transform.translation().x(),it->transform.translation().y(),it->transform.translation().z());
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          pcl::PointXYZ simu_point((*pt)->simulatedpos[0],(*pt)->simulatedpos[1],(*pt)->simulatedpos[2]);
          simu_cloud_ptr->points.push_back(simu_point);
          std::string idstring = std::to_string(iViewerIdx++);
          viewer->addLine<pcl::PointXYZ> (simu_point,camera_pose, idstring);
          //plot the shape
          if (*pt != obj->dylandmarks.back()){
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*std::next(pt))->simulatedpos[0],(*std::next(pt))->simulatedpos[1],(*std::next(pt))->simulatedpos[2]);
            viewer->addLine<pcl::PointXYZ> (simu_point,second_point, 0,1,0,idstring,v1);
          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->simulatedpos[0],(*obj->dylandmarks.begin())->simulatedpos[1],(*obj->dylandmarks.begin())->simulatedpos[2]);
            viewer->addLine<pcl::PointXYZ> (simu_point,second_point, 0,1,0,idstring,v1);
          }
        }
    }
  }
  simu_cloud_ptr->width = (int) simu_cloud_ptr->points.size ();
  simu_cloud_ptr->height = 1;
  viewer->addPointCloud<pcl::PointXYZ> (simu_cloud_ptr, "Dynamic shape cloud",v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Dynamic shape cloud");
  //If you want to update a point cloud that is already displayed, you must first call removePointCloud() and provide the ID of the cloud that is to be updated.
}

void optimizedCircleVis (FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Dynamic optimzed shape visualziation.\n";
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    for (ObjectVec::iterator obj = it->seenedobjs.begin(); obj != it->seenedobjs.end(); ++obj){
      for (PointPtrVec::iterator pt = obj->dylandmarks.begin(); pt != obj->dylandmarks.end(); ++pt){
          pcl::PointXYZ basic_point((*pt)->optimizedpos[0],(*pt)->optimizedpos[1],(*pt)->optimizedpos[2]);
          optimized_cloud_ptr->points.push_back(basic_point);
          std::string idstring = std::to_string(iViewerIdx++);
          //plot the shape
          if (*pt != obj->dylandmarks.back()){
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*std::next(pt))->optimizedpos[0],(*std::next(pt))->optimizedpos[1],(*std::next(pt))->optimizedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,1,0,idstring,v2);
          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->optimizedpos[0],(*obj->dylandmarks.begin())->optimizedpos[1],(*obj->dylandmarks.begin())->optimizedpos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,1,0,idstring,v2);
          }
        }
    }
  }
  optimized_cloud_ptr->width = (int) optimized_cloud_ptr->points.size ();
  optimized_cloud_ptr->height = 1;
  viewer->addPointCloud<pcl::PointXYZ> (optimized_cloud_ptr, "Dynamic shape optimized cloud",v2);
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
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,v1);
            idstring = std::to_string(iViewerIdx++);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,v2);

          }
          else{
            std::string idstring = std::to_string(iViewerIdx++);
            pcl::PointXYZ second_point((*obj->dylandmarks.begin())->truepos[0],(*obj->dylandmarks.begin())->truepos[1],(*obj->dylandmarks.begin())->truepos[2]);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,v1);
            idstring = std::to_string(iViewerIdx++);
            viewer->addLine<pcl::PointXYZ> (basic_point,second_point, 1,0,0,idstring,v2);
          }
        }
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  // viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "v1 Dynamic shape optimized cloud",v1);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "v1 Dynamic shape cloud");
  // viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "v2 Dynamic shape optimized cloud",v2);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "v2 Dynamic shape cloud");
  //If you want to update a point cloud that is already displayed, you must first call removePointCloud() and provide the ID of the cloud that is to be updated.
}


void pointMeasuremenOptimizedtVis(FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer){
  pcl::PointCloud<pcl::PointXYZ>::Ptr gt_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "point Measurement visualziation.\n";

  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    pcl::PointXYZ camera_pose(it->transform.translation().x(),it->transform.translation().y(),it->transform.translation().z());
    pcl::PointXYZ opti_camera_pose(it->optimizedtransform.translation().x(),it->optimizedtransform.translation().y(),it->optimizedtransform.translation().z());

    for (PointPtrVec::iterator pt = it->landmarks.begin(); pt != it->landmarks.end(); ++pt){
        pcl::PointXYZ gt_point((*pt)->truepos[0],(*pt)->truepos[1],(*pt)->truepos[2]);
        pcl::PointXYZ optimized_point((*pt)->optimizedpos[0],(*pt)->optimizedpos[1],(*pt)->optimizedpos[2]);
        gt_cloud_ptr->points.push_back(gt_point);
        optimized_cloud_ptr->points.push_back(optimized_point);

        // idstring = std::to_string(iViewerIdx++);
        // viewer->addLine<pcl::PointXYZ> (gt_point,camera_pose, idstring, v2);
        idstring = std::to_string(iViewerIdx++);
        viewer->addLine<pcl::PointXYZ> (optimized_point,opti_camera_pose, idstring, v2);
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> optimized_color(optimized_cloud_ptr, 255, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gt_color(gt_cloud_ptr, 255, 0, 0);

  viewer->addPointCloud<pcl::PointXYZ> (gt_cloud_ptr, gt_color, "gtv2 point measurement cloud",v2);
  viewer->addPointCloud<pcl::PointXYZ> (optimized_cloud_ptr, optimized_color, "opti point measurement cloud",v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gtv2 point measurement cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "opti point measurement cloud");

}


void pointMeasurementVis(FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer){
  pcl::PointCloud<pcl::PointXYZ>::Ptr gt_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr simu_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "point Measurement visualziation.\n";

  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    pcl::PointXYZ camera_pose(it->transform.translation().x(),it->transform.translation().y(),it->transform.translation().z());
    pcl::PointXYZ simu_camera_pose(it->simulatedtransform.translation().x(),it->simulatedtransform.translation().y(),it->simulatedtransform.translation().z());

    for (PointPtrVec::iterator pt = it->landmarks.begin(); pt != it->landmarks.end(); ++pt){
        pcl::PointXYZ gt_point((*pt)->truepos[0],(*pt)->truepos[1],(*pt)->truepos[2]);
        pcl::PointXYZ simu_point((*pt)->simulatedpos[0],(*pt)->simulatedpos[1],(*pt)->simulatedpos[2]);
        gt_cloud_ptr->points.push_back(gt_point);
        simu_cloud_ptr->points.push_back(simu_point);

        // idstring = std::to_string(iViewerIdx++);
        // viewer->addLine<pcl::PointXYZ> (gt_point,camera_pose, idstring, v1);
        idstring = std::to_string(iViewerIdx++);
        viewer->addLine<pcl::PointXYZ> (simu_point,simu_camera_pose, idstring, v1);
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> simulated_color(simu_cloud_ptr, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gt_color(gt_cloud_ptr, 255, 0, 0);

  viewer->addPointCloud<pcl::PointXYZ> (gt_cloud_ptr, gt_color, "gt point measurement cloud",v1);
  viewer->addPointCloud<pcl::PointXYZ> (simu_cloud_ptr, simulated_color, "simu point measurement cloud",v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gt point measurement cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "simu point measurement cloud");

}

void visulizeFrames(FrameVec gridposes,pcl::visualization::PCLVisualizer::Ptr viewer){
  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    Eigen::Affine3f frame = it->simulatedtransform.cast <float>();
    idstring = std::to_string(iViewerIdx++);
    viewer->addCoordinateSystem (0.5,frame, idstring,v1);
  }

  for (FrameVec::iterator it = gridposes.begin(); it != gridposes.end(); ++it){
    Eigen::Affine3f frame = it->optimizedtransform.cast <float>();
    idstring = std::to_string(iViewerIdx++);
    viewer->addCoordinateSystem (0.5,frame, idstring,v2);
  }
  //adding the traj of gt
  FrameVec::iterator it = gridposes.begin();
  pcl::PointXYZ temp_point(it->transform.translation()[0],it->transform.translation()[1],it->transform.translation()[2]);
  while(it != gridposes.end()){
    pcl::PointXYZ next_frame(it->transform.translation()[0],it->transform.translation()[1],it->transform.translation()[2]);
    idstring = std::to_string(iViewerIdx++);
    viewer->addLine<pcl::PointXYZ> (temp_point,next_frame,1,0,0, idstring,v1);
    idstring = std::to_string(iViewerIdx++);
    viewer->addLine<pcl::PointXYZ> (temp_point,next_frame,1,0,0, idstring,v2);

    temp_point = next_frame;
    ++it;
  }
}
#endif
