#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "mrpt_msgs/ObservationRangeBearing.h"
#include "mrpt_msgs/SingleRangeBearingObservation.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <cmath>        // std::abs
#include <fstream>

struct Obstacle
{
	int start;
	int end;
	int width;

	Obstacle(int start, int end, int width)
	{
		this->start = start;
		this->end = end;
		this->width = width;
	}

	Obstacle()
	{

	}
};

std::vector<Obstacle> Obstacles;
ros::Publisher chatter_pub;
const float range_threshold = 0.5;
const float PI = 3.1415;
int Counter = 0;
bool isMoving = false;
bool isMovingPrev = false;
bool changeState = false;
const int rnd = 1000;

ros::Time DelayStartTime;//Start time of the rotation.
ros::Duration DelayDuration;//Duration of the rotation.
const int DURATION = 2;

bool isTimerActivated = false;

void getInfoObstacle(std::vector<Obstacle>::iterator obs_iter, const sensor_msgs::LaserScan::ConstPtr& msg, double& range, double& angle)
{
	double minRange = 10.0;
	int pointerMinRange = -1;

	for (int i = obs_iter->start; i < obs_iter->end; ++i)
	{
		if (minRange > msg->ranges[i])
		{
			minRange = msg->ranges[i];
			pointerMinRange = i;
		}
	}

	angle = pointerMinRange * msg->angle_increment * 180/PI;
	double middleAngle = (msg->ranges.size() / 2) * msg->angle_increment * 180/PI;

	// ROS_INFO_STREAM("middleAngle: " << middleAngle);

	if (angle > middleAngle)
	{
		angle = angle - middleAngle;
	}
	else if (angle < middleAngle)
	{
		angle = -middleAngle + angle;
	}
	else
	{
		angle = 0.0;
	}

	range = minRange;

	// obs_iter->start
	// obs_iter->end
}

void SaveMovRanges(const std::vector<float>& ranges, double min, double max, double range, double angle)
{
	std::ofstream rangesToFile("/home/andrestoga/ros_Map_Merging/src/poles_detection/src/output_range.txt", std::ofstream::out);

	// rangesToFile << "Range: " << range << "\n" << "Angle: " << angle << std::endl;

	for (std::vector<float>::const_iterator i = ranges.begin(); i != ranges.end(); ++i)
	{
		if (min < *i && *i < max)
		{
			rangesToFile << *i << std::endl;
		}

		// rangesToFile << *i << std::endl;
	}

	rangesToFile << std::endl;

    rangesToFile.close();

 //    char filename[ ] = "/home/andrestoga/ros_Map_Merging/src/poles_detection/src/output_range.txt";
 //    std::fstream rangesToFile;

	// rangesToFile.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);

	// If file does not exist, Create new file
	// if (!rangesToFile ) 
	// {
	// 	std::cout << "Cannot open file, file does not exist. Creating new file.." << std::endl;

	// 	rangesToFile.open(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
	// } 
	// else   
	// {    // use existing file
	// 	cout<<"success "<<filename <<" found. \n";
	// 	cout<<"\nAppending writing and working with existing file"<<"\n---\n";

	// 	appendFileToWorkWith << "Appending writing and working with existing file"<<"\n---\n";
	// 	appendFileToWorkWith.close();
	// 	cout<<"\n";
	// }

	// rangesToFile << "\nRange: " << range << "\n" << "Angle: " << angle << std::endl;

	// for (std::vector<float>::const_iterator i = ranges.begin(); i != ranges.end(); ++i)
	// {
	// 	if (min < *i && *i < max)
	// 	{
	// 		rangesToFile << *i << std::endl;
	// 	}
	// }

	// rangesToFile << std::endl;

 //    rangesToFile.close();
}

void DetectObstacles(const std::vector<float>& ranges, double range_min, double range_max)
{
	//TODO: Possible bug because i = 1
	for (int i = 1, withObs = 0, isStart = 0, start = 0; i < ranges.size(); i++)
	{
		if (range_min < ranges[i] && ranges[i] < range_max)
		{
			if (!isStart)
			{
				isStart = 1;
				start = i;
				withObs++;
			}
			else
			{
				if (std::abs(ranges[i] - ranges[i - 1]) > range_threshold)
				{
					i--;
					Obstacles.push_back(Obstacle(start, i, withObs));
					isStart = 0;
					withObs = 0;
				}
				else
				{
					withObs++;
				}
			}
		}
		else if (isStart == 1)
		{
			Obstacles.push_back(Obstacle(start, i, withObs));
			isStart = 0;
			withObs = 0;
		}
	}	
}

void BuildMsg(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	mrpt_msgs::ObservationRangeBearing observation;

	observation.header = msg->header;

	observation.sensor_pose_on_robot.position.x = 0.125;
	observation.sensor_pose_on_robot.position.y = 0.0;
	observation.sensor_pose_on_robot.position.z = 0.033;

	observation.sensor_pose_on_robot.orientation.x = 0;
	observation.sensor_pose_on_robot.orientation.y = 0;
	observation.sensor_pose_on_robot.orientation.z = 0;
	observation.sensor_pose_on_robot.orientation.w = 1;

	observation.min_sensor_distance = msg->range_min;
	observation.max_sensor_distance = msg->range_max;

	//TODO: Check these values later
	observation.sensor_std_range = 0.03;
	observation.sensor_std_yaw = 0.08;
	observation.sensor_std_pitch = 0.0;

	for (std::vector<Obstacle>::iterator i = Obstacles.begin(); i != Obstacles.end(); ++i, Counter++)
	{
		mrpt_msgs::SingleRangeBearingObservation single;
		double range = 0.0;
		double angle = 0.0;

		getInfoObstacle(i, msg, range, angle);

		//TODO: Check these values later
		single.range = range;
		single.yaw = angle*PI/180;
		single.pitch = 0.0;
		single.id = -1;

		std::cout << "Angle of the pole: " << angle << std::endl;
		std::cout << "Range of the pole: " << range << std::endl;
		std::cout << std::endl;

		observation.sensed_data.push_back(single);

		if (range > 4.0)
		{	
			// ROS_INFO_STREAM("\nRange: " << range << "\n" << "Angle: " << angle);
			std::cout << "Range: " << range << "\n" << "Angle: " << angle << std::endl;

			for (std::vector<float>::const_iterator i = msg->ranges.begin(); i != msg->ranges.end(); ++i)
			{
				if (msg->range_min < *i && *i < msg->range_max)
				{
					std::cout << *i << std::endl;
				}
			}

			// SaveMovRanges(msg->ranges, msg->range_min, msg->range_max, range, angle);
		}

		// std::cout << std::endl;

		// ROS_INFO_STREAM("\nRange: " << range << "\n" << "Angle: " << angle);
	}

	ROS_INFO("\n");

	chatter_pub.publish(observation);
}

//Callback method
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	Obstacles.clear();

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

			// ROS_INFO_STREAM("Current time: " << elapsedTime);
			std::cout << "Current time: " << elapsedTime << std::endl;
		}
		else
		{
			DetectObstacles(msg->ranges, msg->range_min, msg->range_max);
			BuildMsg(msg);
		}
	}

	// if (!isMoving)
	// {
	// 	DetectObstacles(msg->ranges, msg->range_min, msg->range_max);
	// 	BuildMsg(msg);
	// }
	// else
	// {

	// }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// if ( (msg->linear.x != 0.0) || (msg->angular.z != 0.0))
	// {
	// 	isMoving = true;
	// 	// ROS_INFO("Robot is moving");
	// }
	// else
	// {
	// 	isMoving = false;
	// }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double linear_x = msg->twist.twist.linear.x;
	double angular_z = msg->twist.twist.angular.z;

	linear_x = roundf(linear_x * rnd) / rnd;  /* Result: 37.78 nearest */
	angular_z = roundf(angular_z * rnd) / rnd;  /* Result: 37.78 nearest */

	// if (linear_x != 0)
	// {
	// 	std::cout << linear_x << std::endl;
	// }

	// if (angular_z != 0)
	// {
	// 	std::cout << angular_z << std::endl;
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ObstaclesDetection");

	ros::console::shutdown();

	ros::NodeHandle n;
	chatter_pub = n.advertise<mrpt_msgs::ObservationRangeBearing>("landmark", 1000);

	//Create the objects to subscribre to the /scan topic
	//Link the call back method with the subscription
	ros::Subscriber sub = n.subscribe("scan", 1000, laserScanCallback);
	ros::Subscriber sub2 = n.subscribe("cmd_vel", 1000, velCallback);
	ros::Subscriber sub3 = n.subscribe("odom", 1000, odomCallback);

	ros::spin();

	return 0;
}