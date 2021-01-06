//This program takes care of moving the sensor tray while driving. It moves trav 45 deegres to left and 45 degrees to right to detect possbile obstacles on robot's way
//Via topic trayLeftRight gets command to start or to stop movement.

#include "ros/ros.h"
#include "roborodney/servoPosition.h"
#include <stdio.h>
#include <termios.h>
#include "std_msgs/String.h"
#include <string>

int left=1300;
int centre=1500;
int right=1700;
int currentPosition=0;
int step=10;
int working=0;

roborodney::servoPosition message;
ros::Publisher sendMessage;

//Function gets command to start or to stop moveng tray. Upon stopping tray gets to central position.
//Status holds varialbe working
void senzorlevodesno(const std_msgs::String::ConstPtr& msg)
{
	std::string ukaz= msg->data.c_str();
	if (ukaz.compare("start")==0)
	{
		ROS_INFO("Starting left-right movement of tray.");
		working=1;
	}
	if (ukaz.compare("stop")==0)
	{
		ROS_INFO("Stopping left-right movement of tray");
		working=0;
		message.position=1500;
		message.time=ros::Time::now();
		sendMessage.publish(message);
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "TrayLeftRight");
	ros::NodeHandle nh;
	sendMessage = nh.advertise<roborodney::servoPosition>("servoPosition",1000);
	ros::Rate loop_rate(1);
	
	ros::Subscriber sub = nh.subscribe("trayLeftRight", 1000, senzorlevodesno);
	
	ros::Rate tenthOfSecondDelay(10);


	message.pin=25;
	message.time=ros::Time::now();
	message.position=1000;
	

	int c=0;
	
	//loop, that takes care of moving the tray
	while(nh.ok()) {
		if(working==1){
			for(int i=left; i<right; i=i+30) {
				if(working==0){break;}
				message.position=i;
				message.time=ros::Time::now();
				sendMessage.publish(message);
				ros::spinOnce();
				tenthOfSecondDelay.sleep();
			}
			for(int i=right; i>left; i=i-30) {
				if(working==0){break;}
				message.position=i;
				message.time=ros::Time::now();
				sendMessage.publish(message);
				ros::spinOnce();
				tenthOfSecondDelay.sleep();
			}
		}
		ros::spinOnce();
		tenthOfSecondDelay.sleep();
	}
}
