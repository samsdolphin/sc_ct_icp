#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "sc_ct_icp/common.h"

std::string logPath;
int msg_cnt = 0;

void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *pc);
  pcl::io::savePCDFileBinary(logPath + "pcd/" + std::to_string(msg_cnt) + ".pcd", *pc);
  msg_cnt++;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  std::ofstream file;
  file.open(logPath + "pose.json", std::ofstream::app);
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;
  Eigen::Matrix3d R(q_wodom_curr);
  Eigen::Matrix3d R_;
  // R_ << 0, -1, 0, 0, 0, -1, 1, 0, 0; // original
  R_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  R = R_ * R * R_.inverse();
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = R_ * t_wodom_curr;
  file << t(0) << " " << t(1) << " " << t(2) << " "
       << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
  // file << laserOdometry->header.stamp.toSec() << " "
  //      << t(0) << " " << t(1) << " " << t(2) << " "
  //      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  file.close();
}

void navPathHandler(const nav_msgs::Path::ConstPtr& msg)
{
  std::ofstream file;
  file.open(logPath + "SC_CT_ICP.txt", std::ofstream::trunc);
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
  for(size_t i = 0; i < msg->poses.size(); i++)
  {
    q_wodom_curr.x() = msg->poses[i].pose.orientation.x;
    q_wodom_curr.y() = msg->poses[i].pose.orientation.y;
    q_wodom_curr.z() = msg->poses[i].pose.orientation.z;
    q_wodom_curr.w() = msg->poses[i].pose.orientation.w;
    t_wodom_curr.x() = msg->poses[i].pose.position.x;
    t_wodom_curr.y() = msg->poses[i].pose.position.y;
    t_wodom_curr.z() = msg->poses[i].pose.position.z;

    Eigen::Matrix3d R(q_wodom_curr);
    Eigen::Matrix3d R_;
    R_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    R = R_ * R * R_.inverse();
    Eigen::Quaterniond q(R);
    Eigen::Vector3d t = R_ * t_wodom_curr;
    file << msg->poses[i].header.stamp.toSec() << " "
         << t(0) << " " << t(1) << " " << t(2) << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "log_sc_cticp");
  ros::NodeHandle n("~");
  
  n.getParam("logPath", logPath);

  // ros::Subscriber subPoint =
  //   n.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, pointCloudHandler);
  // ros::Subscriber subPose = n.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
  ros::Subscriber subPath = n.subscribe<nav_msgs::Path>("/aft_pgo_path", 100, navPathHandler);

  std::ofstream file;
  file.open(logPath + "SC_CT_ICP.txt", std::ofstream::trunc);
  file.close();
  // file.open(logPath+"pose.json", std::ofstream::trunc);
  // file.close();

  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}