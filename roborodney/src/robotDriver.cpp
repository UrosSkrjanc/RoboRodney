#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <termios.h>
#include "std_msgs/String.h"
#include <string>
#include "roborodney/drivingCommand.h"


// pins on L298N motor driver
// with L298N motor driver can control two DC motors. Pins on 5 and 6 are connected to right motor, nad pins 19 and 13 are connected with left motor
// When we send signal on pin 5 it means, that right motor is spinning forward, and when we send signal on pin 6 it means, thar right motor is spinnig backward
// The same means for pins 19 and 13 for right motor 

int in1 = 5;  //right motor forward
int in2 = 6;  //right motor backward
int in3 = 19;  //left motor forward
int in4 = 13;  //left motor backward

// Next four functions are actually driving robot. They send direction and speed to left and right motor, then wait for a certain period of time, then stop motors.

//moving forward 
void forward(int duration, int speed)
{
  softPwmWrite (in1, speed);  
  softPwmWrite (in3, speed);
  delay(duration);
  softPwmWrite (in1, 0);  
  softPwmWrite (in3, 0);
}

//moving backward
void backward(int duration, int speed)
{
  softPwmWrite (in2, speed);  
  softPwmWrite (in4, speed);
  delay(duration);
  softPwmWrite (in2, 0);  
  softPwmWrite (in4, 0);
}

//rotating left
void left(int duration, int speed)
{
  softPwmWrite (in2, speed);  
  softPwmWrite (in3, speed);
  delay(duration);
  softPwmWrite (in2, 0);  
  softPwmWrite (in3, 0);
  ROS_INFO("SEL LEVO!!!");
}

//rotating right
void right(int duration, int speed)
{
  softPwmWrite (in4, speed);  
  softPwmWrite (in1, speed);
  delay(duration);
  softPwmWrite (in4, 0);  
  softPwmWrite (in1, 0);
}

// This function gets command for direction, speed and duration for driving robot's motors via driving_command topic and then sends apropriate command to one of the four functions above
void getCommand(const roborodney::drivingCommand& msg)
{
  std::string direction = msg.direction.c_str(); //direction of robot
  int speed = msg.speed;  //speed of motors, from 0 (min) to 100 (max) in PWM duty cycle
  int duration = msg.duration; //signal duration in micro seconds

  if (direction.compare("left")==0){ROS_INFO("Going left.");left(duration,speed);}
  if (direction.compare("right")==0){ROS_INFO("Going right.");right(duration,speed);}
  if (direction.compare("forward")==0){ROS_INFO("Going forward.");forward(duration,speed);}
  if (direction.compare("backward")==0){ROS_INFO("Going backward.");backward(duration,speed);}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	ros::Subscriber ukaz_smeri = nh.subscribe("driving_command", 1, getCommand);

	setenv("WIRINGPI_GPIOMEM", "1", 1);

	//Initialisation of L298N motor driver
	wiringPiSetupGpio();
	pinMode(in4,OUTPUT);
	pinMode(in3,OUTPUT);
	pinMode(in2,OUTPUT);
	pinMode(in1,OUTPUT);
	softPwmCreate (in4, 0, 100);
	softPwmCreate (in3, 0, 100);
	softPwmCreate (in2, 0, 100);
	softPwmCreate (in1, 0, 100);

	ros::Duration(1.0).sleep();
	ros::spinOnce();

	printf("Let's drive! \n");
	
	// Program runs until is interrupted by CTRL-C command
	ros::spin();

	printf("Ciao! \n");
	return 0;
}




