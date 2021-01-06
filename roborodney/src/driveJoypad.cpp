//With program user can navigate robot on plane with PS3 bluetooth controller. Robot can gor forward, backward, it can turn left or right. Robot can execute one command at the time. 
//It moves until user holds apripriate key, when key is not pressed, it stops with moving

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include <string>
#include "roborodney/drivingCommand.h"
#include <iostream>
#include <stdio.h>
#include "roborodney/servoPosition.h"

std::string direction="stop";
std::string joypadCommand="";
ros::Publisher directionCommand;
roborodney::drivingCommand command;

//variables for moving tray
float min=570;
float max=2430;
float position=1500;
float centre=1500;
int pinServo=25;

//function to display the current settings
void displaySettings() {
	ROS_INFO("Speed: %d  Duration: %d ", command.speed, command.duration);
}

//function that gets command from PS3 controller
void joypadmess(const sensor_msgs::Joy& msg)
{
  if(msg.buttons[15]==1) {ROS_INFO("Turning left."); direction="left";}
  if(msg.buttons[16]==1) {ROS_INFO("Turning right."); direction="right";}
  if(msg.buttons[13]==1) {ROS_INFO("Driving forward."); direction="forward";}
  if(msg.buttons[14]==1) {ROS_INFO("Driving backward."); direction="backward";}
  if(msg.buttons[2]==1) {joypadCommand="increaseSpeed";}
  if(msg.buttons[0]==1) {joypadCommand="reduceSpeed";}
  if(msg.buttons[3]==1) {joypadCommand="info";displaySettings();}
  if(msg.buttons[6]==1) {joypadCommand="trayLeft";}
  if(msg.buttons[7]==1) {joypadCommand="trayRight";}
  if(msg.buttons[4]==1 || msg.buttons[5]==1) {joypadCommand="trayCentre";}
  if(msg.buttons[1]==1) {joypadCommand="end";}
  //if no button is pressed, robot must stop
  if(msg.buttons[13]==0 && msg.buttons[14]==0 && msg.buttons[15]==0 && msg.buttons[16]==0 && msg.buttons[2]==0 && msg.buttons[0]==0 && msg.buttons[1]==0 && msg.buttons[6]==0 && msg.buttons[7]==0 && msg.buttons[4]==0 && msg.buttons[5]==0) {direction="stop";joypadCommand="";}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joypadnav");
	ros::NodeHandle nh;
	
	ros::Rate secondDelay(1);

	ros::Subscriber joypad = nh.subscribe("joy", 1, joypadmess);
	directionCommand = nh.advertise<roborodney::drivingCommand>("driving_command",1000);
	
	roborodney::servoPosition servoCommand;
	servoCommand.pin=pinServo;
	ros::Publisher sendServoCommand = nh.advertise<roborodney::servoPosition>("servoPosition",1000);
	
	command.direction=direction;
	command.speed=50;
	command.duration=100;

	secondDelay.sleep();
	ros::spinOnce();
	
	ros::Rate loop_rate(10);

	ROS_INFO("Ready to drive!!");
	
	while (nh.ok()) {
		command.direction=direction;
		directionCommand.publish(command);
		ros::spinOnce();
		if(joypadCommand=="increaseSpeed"){
			command.speed=command.speed+1;
			if(command.speed>100){command.speed=100;}
			ROS_INFO("Increase speed on %d",command.speed);
		}
		if(joypadCommand=="reduceSpeed"){
			command.speed=command.speed-1;
			if(command.speed<0){command.speed=0;}
			ROS_INFO("Reduce speed on %d",command.speed);
		}
		if(joypadCommand=="trayLeft"){
			ROS_INFO("Move tray to the left.");
			servoCommand.time=ros::Time::now();
			position=position-20;
			servoCommand.position=position;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			loop_rate.sleep();
		}
		if(joypadCommand=="trayRight"){
			ROS_INFO("Move tray to the right.");
			servoCommand.time=ros::Time::now();
			position=position+20;
			servoCommand.position=position;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			loop_rate.sleep();
		}
		if(joypadCommand=="trayCentre"){
			ROS_INFO("Move tray on centre.");
			servoCommand.time=ros::Time::now();
			position=centre;
			servoCommand.position=centre;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			loop_rate.sleep();
		}
		if(joypadCommand=="end"){
			ROS_INFO("Termination of program.");
			return 0;
		}
		loop_rate.sleep();
	}
}
