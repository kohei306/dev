#include <fstream>
#include <boost/algorithm/string.hpp>
#include <string>
#include <math.h>
#include "ros/ros.h"
#include "OccGridMap/LuxData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "Eigen/Eigen"


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
    single_lux_data.range = std::stoi(parts[4], nullptr, 10);


    // Add to list of control measurements:
    lux_data.push_back(single_lux_data);
  }

  return true;
}

Eigen::Vector4f transformPolarToCartesian(double angle, int range, int layer, const Eigen::Matrix3d & R, const Eigen::Vector3d &t)
{
	double openingAngle = 3.2;
	double levels = 4.0;

	double phi = (layer * openingAngle/(levels - 1) - openingAngle/2.0) * M_PI/180.0;

	double x = range * cos(angle);
	double y = range * sin(angle);
	double z = range * sin(phi);

	Eigen::Vector4f point(x, y, z, 0);

	Eigen::Matrix4f M;
	M << 1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
		 0.0, 0.0, 1.0;

	M(0, 0) = R(0, 0);
	M(0, 1) = R(0, 1);
	M(0, 2) = R(0, 2);
	M(1, 0) = R(1, 0);
	M(1, 1) = R(1, 1);
	M(1, 2) = R(1, 2);
	M(2, 0) = R(2, 0);
	M(2, 1) = R(2, 1);
	M(2, 2) = R(2, 2);

	M(0, 3) = t(1, 0);
	M(1, 3) = t(2, 0);
	M(2, 3) = t(2, 0);

	Eigen::Vector4f result = M * point;

	return result;
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "occupancy_grid_map_server");

  ros::NodeHandle n;
  
 //ros::Publisher occpancy_grid_pub = n.advertise<Localization::VehicleState>("vehicle_state", 1000);

  ros::Rate loop_rate(10);


  std::vector<OccGridMap::LuxData> lux_data;

  if (!lux_data_read("/home/parallels/catkin_ws/src/OccGridMap/data/Messung1.txt", lux_data)) {
    std::cout << "Error: Could not open lux data measurement file" << std::endl;
    return -1;
  }
  
  nav_msgs::OccupancyGrid occGridMap;
 // occGridMap.info.map_load_time = 0;
  occGridMap.info.resolution = 0.1;
  occGridMap.info.width = 10.0;
  occGridMap.info.height = 0.0;

  occGridMap.data.resize(10000);

  for (size_t idx = 0; idx < occGridMap.data.size(); idx++)
  {
	  occGridMap.data[idx] = 0.0;
  }



  while(ros::ok())
  {
 //   for (int i = 0; i < vehicle_states.size(); i++)
    {
 //      vehicle_state_pub.publish(vehicle_states[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
