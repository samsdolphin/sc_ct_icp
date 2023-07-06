#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <math.h>
#include <ceres/ceres.h>

#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

string filePath;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_pcd");
  ros::NodeHandle nh("~");
  
  nh.getParam("filePath", filePath);

  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100);
  ros::Publisher pub_pose = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

  vector<mypcl::pose> pose_vec = mypcl::loadKittiPose(filePath + "poses/07.txt");
  size_t pose_size = pose_vec.size();

  Eigen::Matrix3d R_;
  R_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  for(size_t i = 0; i < pose_size; i++)
  {
    pose_vec[i].t = R_.inverse() * pose_vec[i].t;
    pose_vec[i].q = R_.inverse() * pose_vec[i].q * R_;
  }

  vector<double> time_vec;
  std::fstream file;
  file.open(filePath + "times/07.txt");
  while(!file.eof())
  {
    double t_;
    file >> t_;
    time_vec.push_back(t_);
  }
  cout<<"time_vec size "<<time_vec.size()<<endl;

  Vector3d cur_t, pre_t(0, 0, 0);

  sensor_msgs::PointCloud2 debugMsg;

  for(size_t i = 0; i < pose_size; i++)
  {
    cur_t = pose_vec[i].t;
    // if((cur_t-pre_t).norm() > 0.1)
    {
      // pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
      // mypcl::loadPCD("/media/sam/CR7/KITTI_ME/00/pcd/", 5, pc_surf, i);
      std::stringstream ss;
      ss << std::setw(6) << std::setfill('0') << i;
      string infile = filePath + "sequences/07/velodyne/" + ss.str() + ".bin";
      ifstream input(infile.c_str(), ios::in | ios::binary);
      if(!input.is_open())
      {
        cerr << "Could not read file: " << infile << endl;
        return -1;
      }
      // pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
      pcl::PointCloud<PointType> points;
      const size_t kMaxNumberOfPoints = 1e6;  // From the Readme of raw files.
      points.clear();
      points.reserve(kMaxNumberOfPoints);

      int j;
      for(j=0; input.is_open() && !input.eof(); j++)
      {
        PointType point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points.push_back(point);
      }
      input.close();

      pcl::toROSMsg(points, debugMsg);
      debugMsg.header.frame_id = "camera_init";
      debugMsg.header.stamp = ros::Time().fromSec(time_vec[i]);
      pub_cloud.publish(debugMsg);

      nav_msgs::Odometry odomAftMapped;
			odomAftMapped.header.frame_id = "camera_init";
			odomAftMapped.child_frame_id = "aft_mapped";
			odomAftMapped.header.stamp = ros::Time().fromSec(time_vec[i]);
			odomAftMapped.pose.pose.orientation.x = pose_vec[i].q.x();
			odomAftMapped.pose.pose.orientation.y = pose_vec[i].q.y();
			odomAftMapped.pose.pose.orientation.z = pose_vec[i].q.z();
			odomAftMapped.pose.pose.orientation.w = pose_vec[i].q.w();
			odomAftMapped.pose.pose.position.x = pose_vec[i].t(0);
			odomAftMapped.pose.pose.position.y = pose_vec[i].t(1);
			odomAftMapped.pose.pose.position.z = pose_vec[i].t(2);
			pub_pose.publish(odomAftMapped);

      cur_t = pre_t;
    }
    ros::Duration(0.1).sleep();
  }

  // ros::Rate loop_rate(1);
  // while(ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
}