/*
 * Local map builder
 * The local_map node takes as input a LaserScan message and outputs
 * a local map as OccupancyGrid. The local map orientation is the same
 * as the one of the global frame. The position of the map is the same
 * as the one of the LaserScan.
 *
 * Parameters:
 * - map_width, float, 200, map pixel width (x-direction)
 * - map_height, float, 200, map pixel height (y-direction)
 * - map_resolution, float, 0.020, map resolution (m/pixel)
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <local_map/map_builder.h>
#include <local_map/SaveMap.h>

#include <std_msgs/Float32.h>
#include <local_map/robot_map_angle.h>

//Include headers for OpenCV GUI handling
// #include "opencv2/highgui/highgui.hpp"
//Include headers for OpenCV Image processing
// #include "opencv2/imgproc/imgproc.hpp"

// const int map_x = 200;
// const int map_y = 200;

ros::Publisher map_publisher;
ros::Publisher map_angle_publisher;
local_map::MapBuilder* map_builder_ptr;

// void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle)
// {        
//   CV_Assert(angle % 90 == 0 && angle <= 360 && angle >= -360);

//   if(angle == 270 || angle == -90)
//   {
//     // Rotate clockwise 270 degrees
//     cv::transpose(src, dst);
//     cv::flip(dst, dst, 0);
//   }
//   else if(angle == 180 || angle == -180)
//   {
//     // Rotate clockwise 180 degrees
//     cv::flip(src, dst, -1);
//   }
//   else if(angle == 90 || angle == -270)
//   {
//     // Rotate clockwise 90 degrees
//     cv::transpose(src, dst);
//     cv::flip(dst, dst, 1);
//   }
//   else if(angle == 360 || angle == 0 || angle == -360)
//   {
//     if(src.data != dst.data)
//     {
//       src.copyTo(dst);
//     }
//   }
// }

// void AlignImageToWorldCoord(nav_msgs::OccupancyGrid& map_msg)
// {
//   // cv::Mat mat_map = cv::Mat::zeros( map_x, map_y, CV_8SC1 );

//   cv::Mat mat_map = cv::Mat( map_msg.data ).reshape( 0, map_x );

//   mat_map.convertTo( mat_map, CV_8SC1 );

//   map_msg.data.clear();

//   cv::Mat mat_map_rotated;
//   cv::Mat mat_map_rotated_flip;

//   rotate_90n( mat_map, mat_map_rotated, -90 );
//   cv::flip( mat_map_rotated, mat_map_rotated_flip, 1 );     // because you can't flip in-place (leads to segfault)

//   if ( mat_map_rotated_flip.isContinuous() )
//   {
//     map_msg.data.assign( mat_map_rotated_flip.datastart, mat_map_rotated_flip.dataend );
//   }
//   else
//   {
//     for ( int i = 0; i < mat_map_rotated_flip.rows; ++i )
//     {
//       map_msg.data.insert( map_msg.data.end(), mat_map_rotated_flip.ptr<int8_t>( i ), mat_map_rotated_flip.ptr<int8_t>( i ) + mat_map_rotated_flip.cols );
//     }
//   }
// }

void handleLaserScan(sensor_msgs::LaserScan msg)
{
  map_builder_ptr->grow( msg );

  local_map::robot_map_angle msg_robot_map_angle;

  msg_robot_map_angle.map = map_builder_ptr->getMap();
  msg_robot_map_angle.robot_angle = map_builder_ptr->GetAngle();

  // AlignImageToWorldCoord( map_builder_ptr->getMap() );
  map_publisher.publish( map_builder_ptr->getMap() );
  map_angle_publisher.publish( msg_robot_map_angle );
}

bool save_map(local_map::SaveMap::Request& req, local_map::SaveMap::Response& res)
{
  return map_builder_ptr->saveMap(req.name);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_map");
  ros::NodeHandle nh("~");
  //Default was 200 for 2 meters. 1000 to get all the range from the laser which is 10 meters. Using the default resolution "0.020"
  int dimensions = 1000;

  double map_width;
  double map_height;
  double map_resolution;
  nh.param<double>("map_width", map_width, dimensions);
  nh.param<double>("map_height", map_height, dimensions);
  nh.param<double>("map_resolution", map_resolution, 0.020);
  local_map::MapBuilder map_builder(map_width, map_height, map_resolution);
  map_builder_ptr = &map_builder;

  ros::Subscriber scanHandler = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, handleLaserScan);
  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
  map_angle_publisher = nh.advertise<local_map::robot_map_angle>("local_map_angle", 1, true);

  ros::ServiceServer service = nh.advertiseService("save_map", save_map);

  ros::spin();
}