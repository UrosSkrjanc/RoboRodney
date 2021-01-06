//This is standalone program for placig servo motor shaft in  central position
//It assumes that middle shaft position is 1500 (accurate value depends on motor sepcification) and that servo is connected to computer via pin 25

#include <ros/ros.h>
#include <pigpio.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servtoToCentre");
	ros::NodeHandle nh;

	ROS_INFO("Placing servo in the middle.");

	if(gpioInitialise()<0) {
		ROS_INFO("Servo motor is not responding");
		exit(1);
	} 

	gpioServo(25, 1500);
	sleep(1);

	return 0;
}
