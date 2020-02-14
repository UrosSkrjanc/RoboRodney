#include <ros/ros.h>
#include <pigpio.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servomotor");
	ros::NodeHandle nh;

	ROS_INFO("Postavljam servo na sredino.");

	if(gpioInitialise()<0) {
		ROS_INFO("Ne morem zagnati motorja.");
		exit(1);
	} 

	gpioServo(25, 1500);
	sleep(1);

	return 0;
}
