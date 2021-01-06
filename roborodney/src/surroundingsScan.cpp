//The program scans surroundings with laser or ultraosnic sensor and with obained data, it calculates point on a plane, where it could be obstacles
//Via surroundScanCommand topic program receives commad and proceeds with scan.
//Servo motor moves from left to right in diameter of 180 degrees. Step of motor is 3 degrees, so it obtais 60 points.
//With laser is eought just one measurement, but with ultrasonic sensor, because of the noise in measurements, it gets three measurements then calculate average and marks it valid, if averaga does not deviate too much from individual values


#include <ros/ros.h>
#include "roborodney/servoPosition.h"
#include "roborodney/sensorDistance.h"
#include <iostream>
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "roborodney/pointsSurroundScan.h"


// here it defines, if measurements are taken with laser or ultrasonic sensor. One measurement is required with laser, and three with ultrasonic.
#define LASER
//#define ULTRASONIC

#if defined ULTRASONIC
	int validScansNumber=3;
	float individualMeasurements[3]={};
#endif

#if defined LASER
	int validScansNumber=1;
	float individualMeasurements[1]={};
#endif

int validScansCounter=0;
float distanceToObstacle=0.0;
float calculatedDistance=0;
std::string command="";
int servoPin=25;

int c=0;
int i=0;
int j=0;

int scan=0;

//via topic surroundScanCommand gets a command and starts with scan
void startScan(const std_msgs::String::ConstPtr& msg){
	std::string command= msg->data.c_str();
	if (command.compare("start")==0)
	{scan=1;}
}


void getUltrasonicDistance(const roborodney::sensorDistance& msg)
{
 	distanceToObstacle=msg.distance+11.0;  //the last value is distance from servo shaft and ultrasonic sensor on the ronot
	individualMeasurements[j]=distanceToObstacle;
	j++;
	validScansCounter++;
	calculatedDistance=calculatedDistance+msg.distance;
}


void getLaserDistance(const roborodney::sensorDistance& msg)
{
 	distanceToObstacle=msg.distance+7.0; //the last value is distance from servo shaft and laser sensor on the ronot
	individualMeasurements[j]=distanceToObstacle;
	j++;
	validScansCounter++;
	calculatedDistance=calculatedDistance+msg.distance;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "surroundingsScan");
	ros::NodeHandle nh;
	
	ros::Rate secondDelay(1);
	ros::Rate tenthOfSecondDelay(10);
	
	#if defined LASER
	ROS_INFO("Laser measurement.");
	#endif
	
	#if defined ULTRASONIC
	ROS_INFO("Ultrasonic measurement.");
	#endif
	
	//subscriber for program, that reads distances for sensor
	ros::Subscriber sensorDistance;
	
	//publisher that sends commands for servo motos moves
	ros::Publisher sendServoCommand = nh.advertise<roborodney::servoPosition>("servoPosition",1000);
	
	//subscrber that receives a command for scan
	ros::Subscriber sub = nh.subscribe("surroundScanCommand", 1000, startScan);
	
	//publisher that publishes calculated points, their angles in radians with respect to the central point, mesaured distances and time whan scan ended 
	ros::Publisher surroundPoints = nh.advertise<roborodney::pointsSurroundScan>("surroundPoints", 10);
	
	roborodney::servoPosition servoCommand;
	servoCommand.pin=servoPin;
	
	secondDelay.sleep();
	ros::spinOnce();

	//180 degree scan from 0 to 180 - motor HITEC HS-322HD
	//https://www.servocity.com/hs-322hd-servo
	float min=570;
	float max=2430;
	float maxAngle=180;
	float centre=((max-min)/2)+min;
	float step=(max-min)/maxAngle;
	
	float scanAngle=180;
	float stepOfAngle=3; 
	float startAngle = (maxAngle-scanAngle)/2;
	float start = min + (step*startAngle);
	float end = start + (scanAngle*step);
	
	//program waits 0.005 seconds per 1 degree after each motor movement
	//it means that for an angle of 5 degrees, program waits for 0.025 second
	float motorPause=0.005*stepOfAngle;
	float frequency=1/motorPause;
	ros::Rate pause(frequency);
	
	int numberOfMeasurements = (scanAngle/stepOfAngle)+1;
	float distances[(int)numberOfMeasurements]={};
	float correctedDistances[(int)numberOfMeasurements]={};
	float measurements[(int)numberOfMeasurements][validScansNumber];
	float angles[(int)numberOfMeasurements]={};
	float Xoffset[(int)numberOfMeasurements]={};
	float Yoffset[(int)numberOfMeasurements]={};
	float radians[(int)numberOfMeasurements]={};
	float motorPosition[(int)numberOfMeasurements]={};
	int valid[(int)numberOfMeasurements]={};
	
	
	while(nh.ok()){
		
		if(scan==1){
			
			//first, program rotates servo shaft to startin position
			servoCommand.pin=servoPin;
			servoCommand.time=ros::Time::now();
			servoCommand.position=start;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
			secondDelay.sleep();

			validScansCounter=0;	
	
			//this loop get through all measuremets
			for(i=0;i<=numberOfMeasurements;i++){
				j=0;
				sensorDistance.shutdown();
				servoCommand.time=ros::Time::now();
				servoCommand.position=start+(step*i*stepOfAngle); 
				sendServoCommand.publish(servoCommand);
				ros::spinOnce();
				pause.sleep();
		
				#if defined LASER
					sensorDistance = nh.subscribe("laser_distance", 1, getLaserDistance);
				#endif
	
				#if defined ULTRASONIC
					sensorDistance = nh.subscribe("laser_distance", 1, getUltrasonicDistance);
				#endif
		
				while(validScansCounter<validScansNumber)
				{
					ros::spinOnce();
				}
		

				#if defined LASER
					correctedDistances[i]=individualMeasurements[0];
					distances[i]==individualMeasurements[0];
					measurements[i][0]=individualMeasurements[0];
					valid[i]=1;	
				#endif
		
				#if defined ULTRASONIC
				//Za ultrasonic vzamem tri meritve, izločim tiste, ki so od ostalih v sklopu ene pozicije, razlikujejo več kot za 10cm
				//With ultrasonic sensor program makes three measuremets and excludes those measurements, which differ significantly from others (10 cm or more)  
				for(j=0;j<validScansNumber;j++){measurements[i][j]=individualMeasurements[j];}
				distances[i]=(calculatedDistance/validScansNumber);
		
				float upperFirst=measurements[i][0]+10;
				float lowerFirst=measurements[i][0]-10;
				float upperSecond=measurements[i][1]+10;
				float lowerSecond=measurements[i][1]-10;
				float upperThird=measurements[i][2]+10;
				float lowerThird=measurements[i][2]-10;
		
				correctedDistances[i]=0;
				valid[i]=0;
		
				//all mesaurements are OK
				if( (measurements[i][1]<upperFirst) &&  (measurements[i][1]>lowerFirst) && (measurements[i][2]<upperFirst) &&  (measurements[i][2]>lowerFirst) )
				{
					correctedDistances[i]=(measurements[i][0]+measurements[i][1]+measurements[i][2])/3;
					valid[i]=1;
				}
		
				//first measurement differs significantly
				if( ((measurements[i][0]>upperSecond) ||  (measurements[i][0]<lowerSecond)) && (measurements[i][2]<upperSecond) &&  (measurements[i][2]>lowerSecond) )
				{
					correctedDistances[i]=(measurements[i][1]+measurements[i][2])/2;
					valid[i]=1;
				}
		
				//second measurement differs significantly
				if( ((measurements[i][1]>upperFirst) ||  (measurements[i][1]<lowerFirst)) && (measurements[i][2]<upperFirst) &&  (measurements[i][2]>lowerFirst) )
				{
					correctedDistances[i]=(measurements[i][0]+measurements[i][2])/2;
					valid[i]=1;
				}
		
				//third measurement differs significantly
				if( ((measurements[i][2]>upperFirst) ||  (measurements[i][2]<lowerFirst)) && (measurements[i][1]<upperFirst) &&  (measurements[i][1]>lowerFirst) )
				{
					correctedDistances[i]=(measurements[i][0]+measurements[i][1])/2;
					valid[i]=1;
				} 
				#endif
		
				//----------------------------------------------------------------
		 
				//Prepares data and excludes measurements that are smaller than 10 cm and greater than 100 cm
				motorPosition[i] = start+(step*i*stepOfAngle);
				angles[i]=round((((motorPosition[i])-min)/(step)) - ((maxAngle-180)/2));
				radians[i]=angles[i]*(M_PI/180);
				Yoffset[i]=sin(radians[i])*correctedDistances[i];
				Xoffset[i]=-cos(radians[i])*correctedDistances[i];
				if(correctedDistances[i]>100 || correctedDistances[i]<10 ){valid[i]=0;}
				validScansCounter=0;
				calculatedDistance=0;
			}
	
	
			sensorDistance.shutdown();
	
			//program rotates servo shaft to central position
			servoCommand.time=ros::Time::now();
			servoCommand.position=centre;
			sendServoCommand.publish(servoCommand);
			ros::spinOnce();
	
			std::string tocke="";
	
			//clears all values
			roborodney::pointsSurroundScan allPoints;
			allPoints.points.clear();
			allPoints.anglesInRad.clear();
			allPoints.distances.clear();

			//writes values to variable with results, that will be send via topis  
			for(i=0; i<numberOfMeasurements; i++)
			{
				if(valid[i]==1){
					std::stringstream ss;
					ss << ",(" << Xoffset[i] << "," << Yoffset[i] << ")";
					tocke = tocke + ss.str();
				}
				geometry_msgs::Point point;
				point.x = Xoffset[i];
				point.y = Yoffset[i];
				allPoints.points.push_back(point);
				allPoints.anglesInRad.push_back(radians[i]);
				allPoints.distances.push_back(correctedDistances[i]);
			}
	
			//scan is finished
			scan=0;
	
			//sends result to topic
			allPoints.time = ros::Time::now();
			surroundPoints.publish(allPoints);
		}
			ros::spinOnce();
			secondDelay.sleep();
	}
	return 0;
}
