#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <filesystem>

using namespace std;

string bag_file;
string lidar_topic;
string pcd_file;
string pcd_dir;
bool is_custom_msg;

string get_pcd_dir(string s)
{
  // get the pcd directory from pcd file
  int start_idx = 0;
  char ch = '/';
  size_t last_idx;
  string dir;

  last_idx = s.find_last_of(ch);
  if (last_idx == string::npos)
  {
    ROS_ERROR_STREAM("Invalid pcd directory!");
  }
  else
  {
    dir = s.substr(start_idx, last_idx-start_idx);
  }
  
  return dir;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("bag_file", bag_file, "");
  nh.param<string>("pcd_file", pcd_file, "");
  nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  pcd_dir = get_pcd_dir(pcd_file);
  nh.param<bool>("is_custom_msg", is_custom_msg, false);
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_) {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return -1;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return -1;
  }
  std::vector<string> lidar_topic_vec;
  lidar_topic_vec.push_back(lidar_topic);
  rosbag::View view(bag, rosbag::TopicQuery(lidar_topic_vec));
  for (const rosbag::MessageInstance &m : view) {
    if (is_custom_msg) {
      livox_ros_driver::CustomMsg livox_cloud_msg =
          *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
      for (uint i = 0; i < livox_cloud_msg.point_num; ++i) {
        pcl::PointXYZI p;
        p.x = livox_cloud_msg.points[i].x;
        p.y = livox_cloud_msg.points[i].y;
        p.z = livox_cloud_msg.points[i].z;
        p.intensity = livox_cloud_msg.points[i].reflectivity;
        output_cloud.points.push_back(p);
      }
    } else {
      sensor_msgs::PointCloud2 livox_cloud;
      livox_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(livox_cloud, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i) {
        output_cloud.points.push_back(cloud.points[i]);
      }
    }
  }
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  std::filesystem::create_directories(pcd_dir);
  pcl::io::savePCDFileASCII(pcd_file, output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + pcd_file;
  ROS_INFO_STREAM(msg.c_str());
  ros::shutdown();
  return 0;
}