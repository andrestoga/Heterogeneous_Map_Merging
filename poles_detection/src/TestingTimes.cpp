#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TestingTimes");

	ros::Time DelayStartTime;//Start time of the rotation.
	ros::Duration DelayDuration;//Duration of the rotation.
	const int DURATION = 5;

	ros::NodeHandle n;

	/* Activate the timer */
	DelayStartTime = ros::Time::now();
	DelayDuration = ros::Duration(DURATION);

	while(true)
	{
		int tmp = ros::Time::now().sec - DelayStartTime.sec;

		if(tmp >= DelayDuration.sec)
		{
			break;
		}

		ROS_INFO_STREAM("Current time: " << tmp);
	}

	return 0;
}