//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include "ros/ros.h"

#include <local_map/SaveMap.h>

#include "mrpt_msgs/ObservationRangeBearing.h"
#include "mrpt_msgs/SingleRangeBearingObservation.h"

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//Include headers for OpenCV GUI handling
#include "opencv2/highgui/highgui.hpp"
//Include headers for OpenCV Image processing
#include "opencv2/imgproc/imgproc.hpp"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <local_map/robot_map_angle.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cmath>

typedef struct Corners
{
  // foo() : a(true), b(true) {}

  int x;
  int y;

  int x_respect_robot;
  int y_respect_robot;

  double angle_respect_robot;
  double range_respect_robot;

} Corner_obs;

const char* map_window = "Map image";
const char* corners_window = "Corners detected";

const int thresh = 150;
const int map_x = 200;
const int map_y = 200;

ros::Publisher chatter_pub;
ros::Publisher pub_map_corners;

bool isMoving = false;
bool isMovingPrev = false;
bool changeState = false;

ros::Time DelayStartTime;//Start time of the rotation.
ros::Duration DelayDuration;//Duration of the rotation.
const int DURATION = 2;
bool isTimerActivated = false;

const int rnd = 1000;

nav_msgs::OccupancyGrid MatToOGMmsg( cv::Mat& mat_map, nav_msgs::OccupancyGrid ogm_map );
cv::Mat ReadMatFromVector( std::vector< signed char > v );
void local_map_angleCallback(const local_map::robot_map_angle::ConstPtr& msg);
void FrontEndSLAM_DetectCorners();
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
double CalculateAngleVec( int x1, int y1, int x2, int y2);
void getInfoCorner(cv::Point& corner, double robot_angle, Corner_obs& corner_obs);
void Publish_Corners_To_SLAM_Backend( std::vector<Corner_obs>& Corners_obs );
void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle);
cv::Mat ReadMatFromTxt(std::string filename, int rows, int cols);
double ReadAngle(std::string pathFile);
void Wrapper_Corner_Harris(cv::Mat& map, std::vector<Corner_obs>& Corners_obs, double corner_robot_angle);

inline double EuclideanDistance(double x1, double y1, double x2, double y2)
{
  double x = x1 - x2; //calculating number to square in next step
  double y = y1 - y2;
  double dist;

  dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
  dist = sqrt(dist);                  

  return dist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "corner_detector");
  ros::NodeHandle n;
  // ros::ServiceClient client = n.serviceClient<local_map::SaveMap>("/local_map/save_map");
  chatter_pub = n.advertise<mrpt_msgs::ObservationRangeBearing>("landmark", 1000);
  pub_map_corners = n.advertise< nav_msgs::OccupancyGrid >( "map_corners", 1000 );
  ros::Subscriber sub3 = n.subscribe( "odom", 1000, odomCallback );
  ros::Subscriber sub_local_map_angle = n.subscribe( "local_map/local_map_angle", 1000, local_map_angleCallback );
  local_map::SaveMap srv;

  // srv.request.name = "local_map.txt";

  // ros::Rate loop_rate(10);

  while ( ros::ok() )
  {
    // if (client.call(srv))
    // {
    //   ROS_INFO("Success! Map saved");

      // FrontEndSLAM_DetectCorners();
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service save map");

    //   return 1;
    // }

    // ROS_INFO("Here");

    ros::spinOnce();

    // loop_rate.sleep();
  }

  return 0;
}

nav_msgs::OccupancyGrid MatToOGMmsg( cv::Mat& mat_map, nav_msgs::OccupancyGrid ogm_map )
{
  nav_msgs::OccupancyGrid new_ogm_map;
  std::string map_frame_id_;

  map_frame_id_ = ros::this_node::getName() + "/local_map";
  new_ogm_map.header.frame_id = map_frame_id_;
  new_ogm_map.info.width = ogm_map.info.width;
  new_ogm_map.info.height = ogm_map.info.height;
  new_ogm_map.info.resolution = ogm_map.info.resolution;
  new_ogm_map.info.origin.position.x = ogm_map.info.origin.position.x;
  new_ogm_map.info.origin.position.y = ogm_map.info.origin.position.y;
  new_ogm_map.info.origin.orientation.w = ogm_map.info.origin.orientation.w;
  new_ogm_map.data.assign(new_ogm_map.info.width * new_ogm_map.info.height, -1);

  // ogm_map.data.clear();

  int nRows = mat_map.rows;
  int nCols = mat_map.cols;

  if ( mat_map.isContinuous() )
  {
      nCols *= nRows;
      nRows = 1;
  }

  int i,j;
  uchar* p;

  for( i = 0; i < nRows; ++i )
  {
    p = mat_map.ptr< uchar >( i );

    for ( j = 0; j < nCols; ++j )
    {
      // p[ j ] = table[ p[ j ] ];
      // ogm_map.data[ i*nCols + j ] = ;

      if ( p[ j ] == 0 )
      {
        // out.at<unsigned char>(temprow, tempcol) = 0;
        new_ogm_map.data[ i*nCols + j ] = 100;
      }
      else if( p[ j ] == 255)
      {
        // out.at<unsigned char>(temprow, tempcol) = 255;
        new_ogm_map.data[ i*nCols + j ] = 0;
      }
      // else
      // {
      //   // out.at<unsigned char>(temprow, tempcol) = 127;
      //   new_ogm_map.data[ i*nCols + j ] = -1;
      // }
    }
  }

  return new_ogm_map;
}

cv::Mat ReadMatFromVector( std::vector< signed char > v )
{
  int m;
  // Mat out = cv::Mat::zeros(rows, cols, CV_64FC1);//Matrix to store values
  //Rows and columns
  cv::Mat out = cv::Mat::zeros(map_y, map_x, CV_8UC1);//Matrix to store values

  int cnt = 0;//index starts from 0

  for (std::vector<signed char>::iterator it = v.begin(); it != v.end(); ++it)
  {

    int temprow = cnt / map_x;
    int tempcol = cnt % map_x;

    if ( *it == 100 )
    {
      out.at<unsigned char>(temprow, tempcol) = 0;
    }
    else if( *it == 0)
    {
      out.at<unsigned char>(temprow, tempcol) = 255;
    }
    else
    {
      out.at<unsigned char>(temprow, tempcol) = 127;
    }

    cnt++;
  }

  return out;
}

void local_map_angleCallback( const local_map::robot_map_angle::ConstPtr& msg )
{
  std::vector<cv::Point> corners;
  std::vector<Corner_obs> Corners_obs;
  double corner_robot_angle = 0.0;

  if(changeState == true)
  {
    /* Activate the timer */
    DelayStartTime = ros::Time::now();
    DelayDuration = ros::Duration(DURATION);
    isTimerActivated = true;
    changeState = false;
  }
  else if(!isMoving)
  {
    if (isTimerActivated)
    {
      int elapsedTime = ros::Time::now().sec - DelayStartTime.sec;

      //Checking if it has passed the duration of the rotation.
      if(elapsedTime >= DelayDuration.sec)
      {
        // DetectObstacles(msg->ranges, msg->range_min, msg->range_max);
        // BuildMsg(msg);
        isTimerActivated = false;
      }

      ROS_INFO_STREAM("Current time: " << elapsedTime);
      // std::cout << "Current time: " << elapsedTime << std::endl;
    }
    else
    {
      cv::Mat tmp = ReadMatFromVector( msg->map.data );
      // imwrite( "/home/andrestoga/ros_Map_Merging/map_image.jpg", tmp );
      corner_robot_angle = msg->robot_angle;
      Wrapper_Corner_Harris( tmp, Corners_obs, corner_robot_angle );
      pub_map_corners.publish( MatToOGMmsg( tmp, msg->map ) );
    }
  }
}

void FrontEndSLAM_DetectCorners()
{
  std::vector<cv::Point> corners;
  std::vector<Corner_obs> Corners_obs;
  double corner_robot_angle = 0.0;

  if(changeState == true)
  {
    /* Activate the timer */
    DelayStartTime = ros::Time::now();
    DelayDuration = ros::Duration(DURATION);
    isTimerActivated = true;
    changeState = false;
  }
  else if(!isMoving)
  {
    if (isTimerActivated)
    {
      int elapsedTime = ros::Time::now().sec - DelayStartTime.sec;

      //Checking if it has passed the duration of the rotation.
      if(elapsedTime >= DelayDuration.sec)
      {
        // DetectObstacles(msg->ranges, msg->range_min, msg->range_max);
        // BuildMsg(msg);
        isTimerActivated = false;
      }

      ROS_INFO_STREAM("Current time: " << elapsedTime);
      // std::cout << "Current time: " << elapsedTime << std::endl;
    }
    else
    {
      cv::Mat tmp = ReadMatFromTxt( "/home/andrestoga/ros_Map_Merging/local_map.txt", 200, 200 );
      corner_robot_angle = ReadAngle( "/home/andrestoga/ros_Map_Merging/local_map_angle.txt" );
      Wrapper_Corner_Harris( tmp, Corners_obs, corner_robot_angle );
      Publish_Corners_To_SLAM_Backend( Corners_obs );
    }
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double linear_x = msg->twist.twist.linear.x;
  double angular_z = msg->twist.twist.angular.z;

  linear_x = roundf(linear_x * rnd) / rnd;  /* Result: 37.78 nearest */
  angular_z = roundf(angular_z * rnd) / rnd;  /* Result: 37.78 nearest */

  // if (linear_x != 0)
  // {
  //  std::cout << linear_x << std::endl;
  // }

  // if (angular_z != 0)
  // {
  //  std::cout << angular_z << std::endl;
  // }

  isMovingPrev = isMoving;

  if (linear_x != 0.0 || angular_z != 0.0)
  {
    isMoving = true;

    // std::cout << "Moving" << std::endl;
  }
  else
  {
    if (isMovingPrev == true)
    {
      changeState = true;
    }

    isMoving = false;
    // std::cout << "Not moving" << std::endl;
  }

  // std::cout << "Linear x: " << linear_x << "\n" << "Angular z: " << angular_z << std::endl;
}

double CalculateAngleVec( int x1, int y1, int x2, int y2)
{
  double dot = x1*x2 + y1*y2;      //dot product
  double det = x1*y2 - y1*x2;      //determinant

  return atan2(det, dot);  //atan2(y, x) or atan2(sin, cos)
}

void getInfoCorner(cv::Point& corner, double robot_angle, Corner_obs& corner_obs)
{
  //Unit vector for the x coordinate
  int x1 = 0;
  int y1 = -1;

  const int world_x = 0;
  const int world_y = 0;

  double angle = 0.0;
  double range = 0.0;

  //Calculating the coordinate of the corner with respect of the robot
  int x2 = corner.x - ( map_x / 2 );
  int y2 = ( map_y / 2 ) - corner.y;

  angle = CalculateAngleVec( x1, y1, x2, y2) - robot_angle;
  angle = angle - 1.5708;
  range = sqrt( pow( y2 - world_y, 2 ) + pow( x2 - world_y, 2 ) ) * 0.020;

  corner_obs.x_respect_robot = x2;
  corner_obs.y_respect_robot = y2;

  corner_obs.x = corner.x;
  corner_obs.y = corner.y;

  corner_obs.angle_respect_robot = angle;
  corner_obs.range_respect_robot = range;

  ROS_INFO_STREAM("Local Map Angle: " << robot_angle*180/M_PI);

  
  ROS_INFO_STREAM("Angle of the corner: " << angle*180/M_PI);
  ROS_INFO_STREAM("Range of the corner: " << range);
  ROS_INFO_STREAM("Original pixel corners: " << corner.x << " " << corner.y);
  ROS_INFO_STREAM("Modified pixel corners: " << x2 << " " << y2);
  ROS_INFO("\n");
}

void Publish_Corners_To_SLAM_Backend( std::vector<Corner_obs>& Corners_obs )
{
  mrpt_msgs::ObservationRangeBearing observation;

  //TODO: Check the frame ID
  observation.header.frame_id = "hokuyo_link";
  observation.header.stamp = ros::Time::now();

  observation.sensor_pose_on_robot.position.x = 0.125;
  observation.sensor_pose_on_robot.position.y = 0.0;
  observation.sensor_pose_on_robot.position.z = 0.033;

  observation.sensor_pose_on_robot.orientation.x = 0;
  observation.sensor_pose_on_robot.orientation.y = 0;
  observation.sensor_pose_on_robot.orientation.z = 0;
  observation.sensor_pose_on_robot.orientation.w = 1;

  observation.min_sensor_distance = 0.10000000149;
  observation.max_sensor_distance = 10.0;

  //TODO: Check these values later
  observation.sensor_std_range = 0.03;
  observation.sensor_std_yaw = 0.08;
  observation.sensor_std_pitch = 0.0;

  for (std::vector<Corner_obs>::iterator i = Corners_obs.begin(); i != Corners_obs.end(); i++)
  {
    mrpt_msgs::SingleRangeBearingObservation single;

    //TODO: Check these values later
    single.range = i->range_respect_robot;
    single.yaw = i->angle_respect_robot;
    single.pitch = 0.0;
    single.id = -1;

    observation.sensed_data.push_back(single);
  }

  // ROS_INFO("\n");

  chatter_pub.publish(observation);
}

void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle)
{        
    CV_Assert(angle % 90 == 0 && angle <= 360 && angle >= -360);

    if(angle == 270 || angle == -90)
    {
      // Rotate clockwise 270 degrees
      cv::transpose(src, dst);
      cv::flip(dst, dst, 0);
    }
    else if(angle == 180 || angle == -180)
    {
        // Rotate clockwise 180 degrees
        cv::flip(src, dst, -1);
    }
    else if(angle == 90 || angle == -270)
    {
        // Rotate clockwise 90 degrees
        cv::transpose(src, dst);
        cv::flip(dst, dst, 1);
    }
    else if(angle == 360 || angle == 0 || angle == -360)
    {
        if(src.data != dst.data)
        {
            src.copyTo(dst);
        }
    }
}

//To read data from a text file. 
//filename is the name of the text file
//rows and cols show dimensions of the matrix written in the text file
cv::Mat ReadMatFromTxt(std::string filename, int rows, int cols)
{
  int m;
  // Mat out = cv::Mat::zeros(rows, cols, CV_64FC1);//Matrix to store values
  cv::Mat out = cv::Mat::zeros(rows, cols, CV_8UC1);//Matrix to store values

  std::ifstream fileStream(filename.c_str());
  int cnt = 0;//index starts from 0

  while (fileStream >> m)
  {
    int temprow = cnt / cols;
    int tempcol = cnt % cols;

    if ( m == 100 )
    {
      out.at<unsigned char>(temprow, tempcol) = 0;
    }
    else if( m == 0)
    {
      out.at<unsigned char>(temprow, tempcol) = 255;
    }
    else
    {
      out.at<unsigned char>(temprow, tempcol) = 127;
    }

    cnt++;
  }

  cv::Mat dst;

  rotate_90n(out, dst, -90);
  cv::flip(dst, out, 1);     // because you can't flip in-place (leads to segfault)

  return out;
}

double ReadAngle(std::string pathFile)
{
  std::ifstream ifile( pathFile.c_str(), std::ios::in );
  // std::vector<double> scores;

  //check to see that the file was opened correctly:
  if (!ifile.is_open())
  {
      std::cerr << "There was a problem opening the input file!\n";
      exit(1);//exit or do additional error checking
  }

  double num = 0.0;

  ifile >> num;

  //keep storing values from the text file so long as data exists:
  // while (ifile >> num)
  // {
  //     scores.push_back(num);
  // }

  //verify that the scores were stored correctly:
  // for (int i = 0; i < scores.size(); ++i)
  // {
  //     std::cout << scores[i] << std::endl;
  // }

  return num;
}

void Wrapper_Corner_Harris( cv::Mat& map, std::vector<Corner_obs>& Corners_obs, double corner_robot_angle )
{
  //TODO: I don't know if it is necessary
  // cv::cvtColor( map, gray, CV_BGR2GRAY );

  //   cv::imshow( "corners_window", gray);
  //   cv::waitKey(0);

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros( map.size(), CV_32FC1 );

    // ROS_INFO("Here!!!");

    // std::cout << dst.size() << std::endl;

    /// Detector parameters
    // int blockSize = 3;
    int blockSize = 2;
    // int apertureSize = 7;
    int apertureSize = 3;
    double k = 0.04;
 
    // Detecting corners
    cornerHarris( map, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    // cv::cornerHarris( gray, dst, 3, 23, 0.19, cv::BORDER_DEFAULT );

    // Normalizing
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    Corners_obs.reserve( dst_norm.rows * dst_norm.cols );

    Corner_obs candidate_corner;
    const double max_radious = 5.0;
 
    // Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    {   
        for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( ( int ) dst_norm.at<float>( j, i ) > thresh )
            {
              double range = 0.0;
              double angle = 0.0;
              Corner_obs tmp_corner;
              cv::Point tmp_point( i, j );              

              getInfoCorner( tmp_point, corner_robot_angle, tmp_corner );
              Corners_obs.push_back( tmp_corner );

              //void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
              cv::circle( map, tmp_point, 5, cv::Scalar( 0 ), 1, 8, 0 );

              //
              // if ( 0 == j )
              // {
              //     candidate_corner = tmp_corner;
              // }
              // else
              // {
              //   if ( EuclideanDistance(candidate_corner.x, candidate_corner.y, tmp_corner.x, tmp_corner.y) > max_radious )
              //   {
              //     cv::circle( map, tmp_point, 10, cv::Scalar(255), 1, 8, 0 );
                  // Corners_obs.push_back( candidate_corner );
              //     candidate_corner = tmp_corner;
              //   }
              //   else
              //   {
              //     cv::circle( map, tmp_point, 2, cv::Scalar(255), 1, 8, 0 );
              //   }
              // }

            }
        }
    }

    ROS_INFO_STREAM("Num of corners: " << Corners_obs.size());

    // cv::namedWindow( map_window, CV_WINDOW_AUTOSIZE );
    // cv::moveWindow(map_window, 100, 100);
    // cv::imshow( map_window, map );
 
    // // Showing the result
    // cv::namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
    // cv::moveWindow( corners_window, 500, 100);
    // cv::imshow( corners_window, dst_norm_scaled );

    cv::waitKey(0);
}