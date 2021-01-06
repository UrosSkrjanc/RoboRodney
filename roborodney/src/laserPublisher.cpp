#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stddef.h>	//#define NULL ...
#include <linux/i2c-dev.h>
#include <vl53l0x_driver/VL53L0X.h>
#include "roborodney/sensorDistance.h"

VL53L0X laserSensor;

//frequency of measured distances per second
int hz=10;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserPublisher");
	ros::NodeHandle nh;
	
	//initialization of laser sensor
	laserSensor.init();
	laserSensor.setTimeout(500);

	//publisher that sends distance in cm
	ros::Publisher laser = nh.advertise<roborodney::sensorDistance>("laser_distance", 10);

	ros::Rate loop_rate(hz);

	//program reads distance from sensor in mm
    float distance = laserSensor.readRangeSingleMillimeters();

	// if distance is 65535, that means that sensor is not ready and program is terminated
    if(distance != 65535){
      printf("Laser is ready.\n");
    } else {
      printf("Laser is not ready. I quit.\n");
      exit(0);
    }

	
	//in this loop message for topic is created
	//it sends time of measurement and distance in cm 10 times in secons
	//frekuenzy (times in second) is defined in variable hz
	while (ros::ok())
	{
        roborodney::sensorDistance laser_message;

		distance = laserSensor.readRangeSingleMillimeters();
		
        laser_message.distance = distance/10;
        laser_message.time = ros::Time::now();

        laser.publish(laser_message);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;

}


