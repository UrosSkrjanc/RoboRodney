//Program that controls servo motor. On servo motor there is a tray with sensors. In original robot configuration, there is camera nad laser sensor on tray
//Via servoPosition topic program receives command with two values - one is servo shaft position, and the second is pin
//The position of the shaft depends on servo motor specifications
//Pin number, to which the motor is connected, is given maunaly. That is because more than one servo motor can be connected on a robot and all motors can be cotrolled with just one program.
//Motor is controlled with WiringPi library 

#include <ros/ros.h>
#include <pigpio.h>
#include "roborodney/servoPosition.h"

void servoOnPosition(const roborodney::servoPosition::ConstPtr& msg)
{
	ROS_INFO("%d, %d",msg->pin, msg->position);
	gpioServo(msg->pin, msg->position);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoDriver");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("servoPosition", 1000, servoOnPosition);

	if(gpioInitialise()<0) {ROS_INFO("Servo motor not ready. Terminating."); return 1;}else ROS_INFO("Servo motor is ready.");

	ros::spin();
}
