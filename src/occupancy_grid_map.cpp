#include <fstream>
#include <boost/algorithm/string.hpp>
#include <string>
#include <math.h>
#include "ros/ros.h"
#include "OccGridMap/LuxData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"

#include "external/Eigen/Eigen"
#include "external/easyloggingpp-9.96.4/src/easylogging++.h"

INITIALIZE_EASYLOGGINGPP

bool lux_data_read(std::string filename, std::vector<OccGridMap::LuxData>& lux_data) {

  // Get file of position measurements:
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file.
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file:
  std::string line_pos;
  getline(in_file_pos, line_pos);

   // Run over each single line:
  while(getline(in_file_pos, line_pos))
  {

    std::vector<std::string> parts;
    boost::split(parts, line_pos, boost::is_any_of("|"));

    // Declare single control measurement:
    OccGridMap::LuxData single_lux_data;

    single_lux_data.timestamp = std::stol(parts[0], nullptr, 10);

    single_lux_data.layer = std::stoi(parts[1], nullptr, 10);
    single_lux_data.echo = std::stoi(parts[2], nullptr, 10);
    single_lux_data.angle = std::stod(parts[3], nullptr);
    CLOG(DEBUG, "default") << "=== TRSF M Homogeneous ===" << std::endl;
    single_lux_data.range = std::stoi(parts[4], nullptr, 10);
    single_lux_data.range /= 10;
    CLOG(DEBUG, "default") <<  single_lux_data.range << std::endl;

    if (single_lux_data.timestamp == 1341907053031 && single_lux_data.angle > -0.5 && single_lux_data.angle < 0.5)
    {
      // Add to list of control measurements:
      lux_data.push_back(single_lux_data);
    }

  }

  return true;
}

Eigen::Vector3d transformPolarToCartesian(double angle, int range, int layer, const Eigen::Matrix3d & R, const Eigen::Vector3d &t)
{
   double openingAngle = 3.2;
   double levels = 4.0;

   double phi = (layer * openingAngle/(levels - 1) - openingAngle/2.0) * M_PI/180.0;
   //CLOG(INFO, "default") << angle << ", " << range << ", " << layer << std::endl;

   double x = range * cos(angle);
   double y = range * sin(angle);
   double z = range * sin(phi);


   Eigen::Vector4d point(x, y, z, 1);

   Eigen::Matrix4d M;
   M << 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0;

   M(0, 0) = R(0, 0);
   M(0, 1) = R(0, 1);
   M(0, 2) = R(0, 2);
   M(1, 0) = R(1, 0);
   M(1, 1) = R(1, 1);
   M(1, 2) = R(1, 2);
   M(2, 0) = R(2, 0);
   M(2, 1) = R(2, 1);
   M(2, 2) = R(2, 2);

   M(0, 3) = t(0, 0);
   M(1, 3) = t(1, 0);
   M(2, 3) = t(2, 0);

   Eigen::Vector4d trans_homogeneous = M * point;

   CLOG(DEBUG, "default") << "=== TRSF M Homogeneous ===" << std::endl;
   CLOG(DEBUG, "default") << trans_homogeneous << std::endl;
   CLOG(DEBUG, "default") << "==========================" << std::endl;

   Eigen::Vector3d result = trans_homogeneous.block<3, 1>(0, 0);

   CLOG(DEBUG, "default") << "=== Transformed point ===" << std::endl;
   CLOG(DEBUG, "default") << result << std::endl;
   CLOG(DEBUG, "default") << "=========================" << std::endl;

   return result;
};

Eigen::Matrix3d getRotationMatrix(double yaw_deg, double pitch_deg, double roll_deg)
{
   double yaw_rad = yaw_deg * M_PI/180.0;
   double pitch_rad = pitch_deg * M_PI/180.0;
   double roll_rad = roll_deg * M_PI/180.0;

   Eigen::Matrix3d roll_RM;
                   roll_RM << 1.0, 0.0, 0.0,
                             0.0, cos(roll_rad), sin(roll_rad),
                             0.0, -sin(roll_rad), cos(roll_rad);
   Eigen::Matrix3d pitch_RM;
                   pitch_RM << cos(pitch_rad), 0.0, -sin(pitch_rad),
                               0.0, 1.0, 0.0,
                               sin(pitch_rad), 0.0, cos(pitch_rad);

   Eigen::Matrix3d yaw_RM;
                   yaw_RM << cos(yaw_rad), sin(yaw_rad), 0.0,
                             -sin(yaw_rad), cos(yaw_rad), 0.0,
                             0.0, 0.0, 1.0;

   return yaw_RM * pitch_RM * roll_RM;
}

int main(int argc, char ** argv)
{
   // Load configuration from file
   el::Configurations conf("/home/parallels/catkin_ws/src/OccGridMap/config/LoggingConfig.conf");

   // Reconfigure single logger
  // el::Loggers::reconfigureLogger("develop", conf);

   // Actually reconfigure all loggers instead
   el::Loggers::reconfigureAllLoggers(conf);
   // Now all the loggers will use configuration from file

  ros::init(argc, argv, "occupancy_grid_map_server");

  ros::NodeHandle n;

 ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
 ros::Publisher occupancy_grid_map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1000);

  ros::Rate loop_rate(10);

  std::vector<OccGridMap::LuxData> lux_data;

  if (!lux_data_read("/home/parallels/catkin_ws/src/OccGridMap/data/LuxData.txt", lux_data)) {
    std::cout << "Error: Could not open lux data measurement file" << std::endl;
    return -1;
  }
  // Lux extrinsic parameters
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;

  double dx = 0.0;
  double dy = 0.0;
  double dz = 1.0;

  Eigen::Matrix3d RM = getRotationMatrix(yaw, pitch, roll);
  Eigen::Vector3d TM(dx, dy, dz);

  CLOG(DEBUG, "default") << "=== RM ===" << std::endl;
  CLOG(DEBUG, "default") << RM <<std::endl;

  CLOG(DEBUG, "default") << "=== TM ===" << std::endl;
  CLOG(DEBUG, "default") << TM << std::endl;
  CLOG(DEBUG, "default") << "==========" << std::endl;

  std::vector<Eigen::Vector3d> points_in_cartesian;
  for (size_t idx = 0; idx < lux_data.size(); idx++)
  {
    Eigen::Vector3d point_in_cartesian;
    point_in_cartesian = transformPolarToCartesian(lux_data[idx].angle, lux_data[idx].range, lux_data[idx].layer, RM, TM);
  //  CLOG(INFO, "default") << point_in_cartesian << std::endl;
    points_in_cartesian.push_back(point_in_cartesian);
  }

  // Init Occ map
  nav_msgs::OccupancyGrid occGridMap;
 // occGridMap.info.map_load_time = 0;

  const uint map_width = 20;
  const uint map_height = 20;
  const double map_resolution = 10;
  const int map_num_cells = map_width * map_height;

  occGridMap.header.frame_id = "map";
  occGridMap.info.resolution = map_resolution;
  occGridMap.info.width = map_width;
  occGridMap.info.height = map_height;

  geometry_msgs::Pose map_orig;
  geometry_msgs::Point map_orig_point;
  map_orig_point.x = -(static_cast<double>(map_width) * map_resolution) * 0.5;
  map_orig_point.y = -(static_cast<double>(map_height) * map_resolution) * 0.5;
  map_orig_point.z = 0;
  map_orig.position = map_orig_point;
  occGridMap.info.origin = map_orig;


  occGridMap.data.resize(map_num_cells);
  CLOG(INFO, "default") << "occGridMap.data.size()" << ": " << occGridMap.data.size() << std::endl;
  for (size_t idx = 0; idx < occGridMap.data.size(); idx++)
  {
     occGridMap.data[idx] = 0.0;
  }

  for (size_t idx = 0; idx < points_in_cartesian.size(); idx++)
  {
    uint x_idx = static_cast<uint>((points_in_cartesian[idx](0) + (static_cast<double>(map_width) * map_resolution) / 2) / occGridMap.info.resolution);
    uint y_idx = static_cast<uint>((points_in_cartesian[idx](1) + (static_cast<double>(map_height) * map_resolution) / 2) / occGridMap.info.resolution);
   // uint z_idx = std::abs(points_in_cartesian[idx](2));
   CLOG(INFO, "default") << "points_in_cartesian: " << points_in_cartesian[idx](0) << ", " << points_in_cartesian[idx](1);
   CLOG(INFO, "default") <<  "x_idx: "<< x_idx << ", y_idx: " << y_idx << std::endl;

   occGridMap.data[x_idx + (y_idx * map_width)] = 100.0;
   //CLOG(INFO, "default") << int(points_in_cartesian[idx](0)/occGridMap.info.resolution) << ", " << int((points_in_cartesian[idx](1)/occGridMap.info.resolution)) << "(" <<int((points_in_cartesian[idx](1)/occGridMap.info.resolution) * occGridMap.info.width) << ")"<< std::endl;

  }


  while(ros::ok())
  {
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "map";
    for (size_t idx = 0; idx < points_in_cartesian.size(); idx++)
    {
      geometry_msgs::Point32 point;
      point.x = static_cast<float>(points_in_cartesian[idx](0));
      point.y = static_cast<float>(points_in_cartesian[idx](1));
      point.z = static_cast<float>(points_in_cartesian[idx](2));
      point_cloud.points.push_back(point);
    }
    occupancy_grid_map_pub.publish(occGridMap);
    point_cloud_pub.publish(point_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
