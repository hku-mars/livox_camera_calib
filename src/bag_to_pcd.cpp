#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

string bag_file;
string lidar_topic;
string pcd_file;
bool is_custom_msg;

double center_yaw_deg;
double fov_yaw_deg;
double min_elev_deg;
double max_elev_deg;  

inline double degWrap180(double d)
{
  while (d > 180.0) d -= 360.0;
  while (d <= -180.0) d += 360.0;
  return d;
}

inline bool angleInRangeDeg(double ang, double min_deg, double max_deg) {
  if (min_deg <= max_deg) return (ang >= min_deg && ang <= max_deg);
  return (ang >= min_deg || ang <= max_deg);
}

inline bool inFovYawElev(double x, double y, double z,
                         double center_yaw_deg, double fov_yaw_deg,
                         double min_elev_deg, double max_elev_deg) {
  double yaw_deg = std::atan2(y, x) * 180.0 / M_PI;
  yaw_deg = degWrap180(yaw_deg - center_yaw_deg);

  double r_xy = std::sqrt(x * x + y * y);
  double elev_deg = std::atan2(z, r_xy) * 180.0 / M_PI;

  double half_yaw = 0.5 * std::fabs(fov_yaw_deg);
  double yaw_min = -half_yaw;
  double yaw_max = +half_yaw;

  bool yaw_ok = angleInRangeDeg(yaw_deg, yaw_min, yaw_max);
  bool elev_ok = angleInRangeDeg(elev_deg, min_elev_deg, max_elev_deg);
  return (yaw_ok && elev_ok);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("bag_file", bag_file, "");
  nh.param<string>("pcd_file", pcd_file, "");
  nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  nh.param<bool>("is_custom_msg", is_custom_msg, false);

  // fov param
  nh.param<double>("center_yaw_deg", center_yaw_deg, 0.0);
  nh.param<double>("fov_yaw_deg",    fov_yaw_deg,    60.0);
  nh.param<double>("min_elev_deg",   min_elev_deg,  -90.0);
  nh.param<double>("max_elev_deg",   max_elev_deg,   90.0);

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

  size_t kept = 0, skipped = 0;

  for (const rosbag::MessageInstance &m : view) {
    if (is_custom_msg) {
      livox_ros_driver::CustomMsg livox_cloud_msg =
          *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
      for (uint i = 0; i < livox_cloud_msg.point_num; ++i) {
        const auto &pt = livox_cloud_msg.points[i];
        if(inFovYawElev(pt.x, pt.y, pt.z,
                         center_yaw_deg, fov_yaw_deg,
                         min_elev_deg, max_elev_deg))
        {
          pcl::PointXYZI p;
          p.x = pt.x;
          p.y = pt.y;
          p.z = pt.z;
          p.intensity = pt.reflectivity;
          output_cloud.points.push_back(p);
          ++kept;
        }
        else
        {
          ++skipped;
        }
        // pcl::PointXYZI p;
        // p.x = livox_cloud_msg.points[i].x;
        // p.y = livox_cloud_msg.points[i].y;
        // p.z = livox_cloud_msg.points[i].z;
        // p.intensity = livox_cloud_msg.points[i].reflectivity;
        // output_cloud.points.push_back(p);
      }
    } else {
      sensor_msgs::PointCloud2 livox_cloud;
      livox_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(livox_cloud, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i) {
        const auto &q = cloud.points[i];
        if(inFovYawElev(q.x, q.y, q.z,
                         center_yaw_deg, fov_yaw_deg,
                         min_elev_deg, max_elev_deg))
        {
          output_cloud.points.push_back(q);
          ++kept;
        }
        else
        {
          ++skipped;
        }
        // output_cloud.points.push_back(cloud.points[i]);
      }
    }
  }
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileASCII(pcd_file, output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + pcd_file;
  ROS_INFO_STREAM(msg.c_str());
  return 0;
}