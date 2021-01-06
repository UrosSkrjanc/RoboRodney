//With program user can navigate robot on plane with keyboard. Robot can gor forward, backward, it can turn left or right. Robot can execute one command at the time. 
//It moves until user holds apripriate key, when key is not pressed, it stops with moving

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <std_msgs/String.h>
#include "roborodney/drivingCommand.h"
#include "roborodney/sensorDistance.h"
#include "roborodney/servoPosition.h"

using namespace std;

ros::Publisher directionCommand;

string direction="stop";

//variables for moving tray
float min=570;
float max=2430;
float position=1500;
float centre=1500;
int pinServo=25;

int safetyDistance=50;

//command for sending drive comamnds (left, right, forward, backward)
roborodney::drivingCommand command;

//funkcija, ki s topica razdalja_senzor (ultrasonic in laser) dobiva vrednosti - oddaljenost robota od ovire 
//Function gets distance to obstacle via laser_distance topic
void getSensorDistance(const roborodney::sensorDistance& msg)
{
	std_msgs::String msgg;
	if(msg.distance<safetyDistance)
	{		
		ROS_INFO("Robot is near obstacle: %f cm.",msg.distance);
	}
}


//functions for proper reading from keyboard where key is pressed for a long time
void set_mode(int want_key)
{
	static struct termios old;
    static struct termios nov;
	if (!want_key) {
		tcsetattr(STDIN_FILENO, TCSANOW, &old);
		return;
	}
 
	tcgetattr(STDIN_FILENO, &old);
	nov = old;
	nov.c_lflag &= ~(ICANON);
	tcsetattr(STDIN_FILENO, TCSANOW, &nov);
}
 
int get_key()
{
	int c = 0;
	fd_set fs;
 
	FD_ZERO(&fs);
	FD_SET(STDIN_FILENO, &fs);
	select(STDIN_FILENO + 1, &fs, 0, 0, 0);
 
	if (FD_ISSET(STDIN_FILENO, &fs)) {
		c = getchar();
		set_mode(0);
	}
	return c;
}


//function to display the current settings
void displaySettings() {
	ROS_INFO("Speed: %d  Duration: %d ", command.speed, command.duration);
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "driveKeyboard");
	ros::NodeHandle nh;
	
	ros::Rate secondDelay(1);
	ros::Rate tenthOfSecondDelay(10);
	ros::Rate hundredthsOfSecondDelay(100);
	ros::Rate fiftyMilliseconds(20);
	
	directionCommand = nh.advertise<roborodney::drivingCommand>("driving_command",10);
	ros::Subscriber sensorDistance = nh.subscribe("laser_distance", 1, getSensorDistance);
	ros::Publisher sendServoCommand = nh.advertise<roborodney::servoPosition>("servoPosition",1000);
	
	roborodney::servoPosition servoCommand;
	servoCommand.pin=pinServo;
	
	std_msgs::String msg;
	msg.data="start";
	secondDelay.sleep();
	ros::spinOnce();
	
	command.direction=direction;   
	command.speed=50;
	command.duration=100;

	secondDelay.sleep();
	ros::spinOnce();
	
	ROS_INFO("Ready to drive!");

	
	int c=0;
	  
	while(c !=27)
	{		
		set_mode(1);
		tcflush(STDIN_FILENO, TCIFLUSH);
		fflush(stdout);
		c = tolower(get_key());
	
	
		//unconditionally forward
		if (c=='t')
		{
			ROS_INFO("Moving forward.");
			command.direction="forward";
			directionCommand.publish(command);
		} 
	
		//unconditionally left
		if (c=='f')
		{
			ROS_INFO("Turning left.");
			command.direction="left";
			directionCommand.publish(command);
		}
	
		//unconditionally right
		if (c=='h')
		{
			ROS_INFO("Turning right.");
			command.direction="right";
			directionCommand.publish(command);
		}
	
		//unconditionally backward
		if (c=='b')
		{
			ROS_INFO("Moving backward.");
			command.direction="backward";
			directionCommand.publish(command);
		}

		//display the current settings
		if(c=='y') 
		{
			displaySettings();
		}
	
		//increasing speed
		if(c=='a') {
			command.speed=command.speed+1;
			if(command.speed>100){command.speed=100;}
			ROS_INFO("Increase speed on %d",command.speed);
		}
		
		//reducing speed
		if(c=='s') {
			command.speed=command.speed-1;
			if(command.speed<0){command.speed=0;}
			ROS_INFO("Reduce speed on %d",command.speed);
		}
	
		//moving tray to left
		if(c=='q') {
			ROS_INFO("Move tray to the left.");
			servoCommand.time=ros::Time::now();
			position=position-20;
			servoCommand.position=position;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			secondDelay.sleep();
		}
	
		//moving tray to right
		if(c=='w') {
			ROS_INFO("Move tray to the right.");
			servoCommand.time=ros::Time::now();
			position=position+20;
			servoCommand.position=position;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			secondDelay.sleep();
		}
		
		//moving tray to centre
		if(c=='e') {
			ROS_INFO("Move tray on centre.");
			servoCommand.time=ros::Time::now();
			position=centre;
			servoCommand.position=centre;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			secondDelay.sleep();
		}
	
	ros::spinOnce();
	hundredthsOfSecondDelay.sleep();
	
	ros::spinOnce();
	hundredthsOfSecondDelay.sleep();
  }
  
  printf("Termination of program.");
  return 0;
  
}
