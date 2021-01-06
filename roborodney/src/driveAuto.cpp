#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>

#include "roborodney/sensorDistance.h"
#include "roborodney/drivingCommand.h"
#include "roborodney/pointsSurroundScan.h"
#include "roborodney/facePosition.h"
#include "roborodney/servoPosition.h"

using namespace cv;
using namespace std;

//global publishers and subscribers 
ros::Publisher trayLeftRight;
ros::Publisher surroundScanCommand;
ros::Publisher driving_command;

// global variables
int locked=0;
int scanInProgress=0;
//variable for distance measurement to obstacle
float currentDistance=0; 
float currentAngle=0;
string direction="stop";
//variable for goal angle
float goalAngle=0;
//variable that count moves for checking, if robot is going to right direction
int forwardStepCounter=0;

//variables for controlling events 
int startOfDrive=1;
int scannedQR=0;  //
int scannedSurround=0; //ce je skeniral okolico
int turnedRobot=0;
int angleOfRotation=0;
int driveInProgress=0;

//variables that tells if robot has recently detected QR code or face
string qrCodeCommand="";
int encounteredAnObstacle=0;
int encounteredAnQrCode=0;
double xx1,xx2,yy1,yy2, resx, resy, xCentre, xCentreOfPicture; 
bool detectedFace=false; 

//min and max of scurity distance in cm - obstacle lies between these two values
int maxSecurityDistance=30; 
int minSecurityDistance=5;

//for sending driving commands - forward, backward, left, right
roborodney::drivingCommand directionCommand;

//for surroundings scan - used in getEnvironmentalPoints function
cv::Mat matrix(400,400,CV_8UC1,cv::Scalar(255));
cv::Mat matrixWeight(400,400,CV_8UC1,cv::Scalar(255));
cv::Mat matrixWeightInv(400,400,CV_8UC1,cv::Scalar(255));
std::vector<float>anglesOnGraph;
std::vector<float>valuesOnGraph;
int numberOfAnglesOnGraph = 19;
std::vector<Mat> weightOnMatrix;
vector<geometry_msgs::Point> Points;
vector<float>distancesScan;

//-----------------------------------------------------------------------------------------------------------------
//function that rotates robot for desired angle
void robotRotation(int angle)
{
	ros::Rate secondDelay(1);
	ros::Rate tenthOfSecondDelay(10); 
	ros::Rate hundredthsOfSecondDelay(100);
	
	ros::spinOnce();
	tenthOfSecondDelay.sleep();
	ros::spinOnce();
	
	float startAngle = currentAngle;
	float endAngle = 0;
	float safetyAngle=0; //user can manualy set angle, that robot doesn't turn too much on one side
	int numberOfSteps=0;
	
	directionCommand.speed=50; 
	directionCommand.duration=20;  
	
	while(angle>=360){angle=angle-360;}
	while(angle<=-360){angle=angle+360;}
	
	if(angle>180){angle=angle-180;}
	if(angle<-180){angle=angle+180;} 
	
	if(angle > 0)
	{
		endAngle = currentAngle+angle-safetyAngle;
		if(endAngle>180){endAngle=endAngle-360;}
		if(endAngle>178){endAngle=178;}
		
		if(startAngle>endAngle)
		{
			while(currentAngle>0){
				directionCommand.direction="left";
				driving_command.publish(directionCommand);
				numberOfSteps++;
				ros::spinOnce();
				for(int i=0;i<3;i++){ros::spinOnce();hundredthsOfSecondDelay.sleep();} 
				ros::spinOnce();
				hundredthsOfSecondDelay.sleep();
			}
		}
		
		while(currentAngle<endAngle)
		{
			directionCommand.direction="left";
			driving_command.publish(directionCommand);
			numberOfSteps++;
			ros::spinOnce();
			for(int i=0;i<3;i++){ros::spinOnce();hundredthsOfSecondDelay.sleep();}
			ros::spinOnce();
			hundredthsOfSecondDelay.sleep();
		}
	} else 
	{
		endAngle = currentAngle+angle+safetyAngle;
		if(endAngle<-180){endAngle=endAngle+360;} 
		if(endAngle<-178){endAngle=-178;}
		
		if(startAngle<endAngle)
		{
			while(currentAngle<0){
				directionCommand.direction="right";
				driving_command.publish(directionCommand);
				numberOfSteps++;
				ros::spinOnce();
				for(int i=0;i<3;i++){ros::spinOnce();hundredthsOfSecondDelay.sleep();}
				ros::spinOnce();
				hundredthsOfSecondDelay.sleep();
			}
		}
		
		while(currentAngle>endAngle)
		{
			directionCommand.direction="right";
			driving_command.publish(directionCommand);
			numberOfSteps++;
			ros::spinOnce();
			for(int i=0;i<3;i++){ros::spinOnce();hundredthsOfSecondDelay.sleep();}
			ros::spinOnce();
			hundredthsOfSecondDelay.sleep();
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function that gets distance to obstacle that is measured with laser
//when robots gets value for obstacle, that is between boudaries, tray and robot is 
void getSensorDistance(const roborodney::sensorDistance& msg)
{
	currentDistance = msg.distance;
	std_msgs::String msgg;
	if(msg.distance>minSecurityDistance && msg.distance<maxSecurityDistance && locked==0 && scanInProgress==0)
	{
		encounteredAnObstacle=1;
		msgg.data="stop";
		trayLeftRight.publish(msgg);
	}
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function, that gets detected QR code
void getQrCommand(const std_msgs::String& msg){ 
	encounteredAnQrCode=1;
	
	if(msg.data=="LEFT"){

		qrCodeCommand="LEFT";
	}
	if(msg.data=="RIGHT"){

		qrCodeCommand="RIGHT";
	}
	if(msg.data=="TURN"){

		qrCodeCommand="TURN";
	}
	if(msg.data=="CONTINUE"){

		qrCodeCommand="CONTINUE";
	}
	if(msg.data=="STOP"){

		qrCodeCommand="STOP";
	}	
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function, that gets points from surroundings scan nd with VFH algorithm calculates directon to avoid obstacles
void gerEnvironmentalPoints(const roborodney::pointsSurroundScan& msg)
{
	ros::Rate tenthOfSecondDelay(10);
	
	anglesOnGraph.clear();
	valuesOnGraph.clear();
	weightOnMatrix.clear();
	Points.clear();
	distancesScan.clear();

	scanInProgress=0;

	//loop that creates matrix with wich calculates right direction
	//kartezis histogram
	matrix = cv::Scalar(255);
	matrixWeight = cv::Scalar(255);
	matrixWeightInv = cv::Scalar(255);
	for(int i=0;i<Points.size();i++) {
		if(Points[i].x<200 && Points[i].x>-200 && Points[i].y<200 && Points[i].y>5) {
			int x=(int)(Points[i].x+200); 
			int y=(int)(400-Points[i].y);
			matrix.at<uchar>(y,x)=0;
			int distanceWeights=(fabs(100-distancesScan[i])/100)*30;
			matrixWeight.col(x).row(y)=matrixWeight.col(x).row(y)-15;
			for(int j=x-1; j<=x+1; j++){
				for(int k=y-1;k<=y+1; k++){
					matrixWeight.col(j).row(k)=matrixWeight.col(j).row(k)-distanceWeights;
				}
			}
			for(int j=x-2; j<=x+2; j++){
				for(int k=y-2;k<=y+2; k++){
					matrixWeight.col(j).row(k)=matrixWeight.col(j).row(k)-distanceWeights;
				}
			}
			for(int j=x-3; j<=x+3; j++){
				for(int k=y-3;k<=y+3; k++){
					matrixWeight.col(j).row(k)=matrixWeight.col(j).row(k)-distanceWeights;
				}
			}
			for(int j=x-4; j<=x+4; j++){
				for(int k=y-4;k<=y+4; k++){
					matrixWeight.col(j).row(k)=matrixWeight.col(j).row(k)-distanceWeights;
				}
			}
		}
	}

	//loop that creates negative of matrix - for easier calculation of direction
	for(int i=0;i<400;i++){
		for(int j=0;j<400;j++){
			matrixWeightInv.row(i).col(j) = 255-matrixWeight.row(i).col(j);
		}
	}
	
	
	//Tukaj imam inverzno matriko z utezmi. Sedaj bi moral kopije, 18, spraviti v vector 
	//loop creates apropriate copies of matrix to calculate the value of each section separately
	std::vector<float>radiani;
	for(int i=0;i<numberOfAnglesOnGraph;i++){
		Mat temp;
		matrixWeightInv.copyTo(temp);
		weightOnMatrix.push_back(temp);
		radiani.push_back(i*180/((float)numberOfAnglesOnGraph)*(M_PI/180));
	}
	
	
	//for each of matrix loop sets area
	Mat matrixWeightInvLines;
	matrixWeightInv.copyTo(matrixWeightInvLines);
	vector <float>exYi;
	vector <float>Yi;
	vector <float>YiMax;
	exYi.clear();	
	Yi.clear();
	for(int i=0;i<weightOnMatrix.size();i++){
		exYi.push_back(0);
		YiMax.push_back(0);
	}
	
	//loop that creates all images to calculate values 
	for(int i=0;i<200;i++){
		exYi=Yi;
		Yi.clear();
		for(int j=0;j<weightOnMatrix.size();j++){
			float yPoint=tan(radiani[j])*i;
			Yi.push_back(yPoint);
			if(fabs(Yi[j])>YiMax[j]&&fabs(Yi[j])<399){YiMax[j]=fabs(Yi[j]);}
		}
		Yi.push_back(0);
		
		
		for(int j=0;j<fabs(weightOnMatrix.size()/2);j++){
			if(Yi[j]<399 && Yi[j]>-399){
				matrixWeightInvLines.col(200-i).row(399-fabs(Yi[j]))=255;
				matrixWeightInvLines.col(199+i).row(399-fabs(Yi[j]))=255;
				weightOnMatrix[j].colRange(0,200-i).row(399-fabs(Yi[j]))=0;
				weightOnMatrix[numberOfAnglesOnGraph-1-j].colRange(199+i,399).row(399-fabs(Yi[j]))=0;
				for(int k=fabs(exYi[j])+1;k<fabs(Yi[j]);k++) {
					matrixWeightInvLines.col(200-i).row(399-k)=255;
					matrixWeightInvLines.col(199+i).row(399-k)=255;
					weightOnMatrix[j].colRange(0,200-i).row(399-k)=0;
					weightOnMatrix[numberOfAnglesOnGraph-1-j].colRange(199+i,399).row(399-k)=0;
				}
				if(Yi[j+1]<399 && Yi[j+1]>-399){
					weightOnMatrix[j].colRange(200-i,399).row(399-fabs(Yi[j+1]))=0;
					weightOnMatrix[numberOfAnglesOnGraph-1-j].colRange(0,199+i).row(399-fabs(Yi[j+1]))=0;
					for(int k=fabs(exYi[j+1])+1;k<fabs(Yi[j+1]);k++) {
						weightOnMatrix[j].colRange(200-i,399).row(399-k)=0;
						weightOnMatrix[numberOfAnglesOnGraph-1-j].colRange(0,199+i).row(399-k)=0;
					}
				}
			}
		}
		
		int j = fabs(weightOnMatrix.size()/2);
		if(Yi[j]<399 && Yi[j]>-399){
			matrixWeightInvLines.col(200-i).row(399-fabs(Yi[j]))=255;
			matrixWeightInvLines.col(199+i).row(399-fabs(Yi[j]))=255;
			weightOnMatrix[j].colRange(0,200-i).row(399-fabs(Yi[j]))=0;
			for(int k=fabs(exYi[j])+1;k<fabs(Yi[j]);k++) {
				matrixWeightInvLines.col(200-i).row(399-k)=255;
				matrixWeightInvLines.col(199+i).row(399-k)=255;
				weightOnMatrix[j].colRange(0,200-i).row(399-k)=0;
			}
			if(Yi[j+1]<399 && Yi[j+1]>-399){
				weightOnMatrix[j].colRange(200+i,399).row(399-fabs(Yi[j+1]))=0;
				for(int k=fabs(exYi[j+1])+1;k<fabs(Yi[j+1]);k++) {
					weightOnMatrix[j].colRange(200+i,399).row(399-k)=0;
				}
			}
		}
	}

	//a loop that erases the top of the image for proper calculations
	for(int j=0;j<fabs(weightOnMatrix.size()/2);j++){
		weightOnMatrix[j].colRange(0,399).rowRange(0, 399-YiMax[j+1])=0;
		weightOnMatrix[numberOfAnglesOnGraph-1-j].colRange(0,399).rowRange(0, 399-YiMax[numberOfAnglesOnGraph-1-j])=0;
	}
	int j = fabs(weightOnMatrix.size()/2);
	weightOnMatrix[j].colRange(0,399).rowRange(0, 399-YiMax[j])=0;
	
	//loop that calculates angle and value in hat direction
	//https://stackoverflow.com/questions/21874774/sum-of-elements-in-a-matrix-in-opencv
	anglesOnGraph.clear();
	valuesOnGraph.clear();
	for(int i=0;i<weightOnMatrix.size();i++){
		anglesOnGraph.push_back(i*(180.0/weightOnMatrix.size()));
		valuesOnGraph.push_back(sum(weightOnMatrix[i])[0]);
	}
	
	//loop gets direction with smallest value to turn the robot in that direction
	float minimum=100000;
	int position=0;
	for(int i=4;i<anglesOnGraph.size()-4;i++){
		float currentValue = (0.5*valuesOnGraph[i-1])+(2*valuesOnGraph[i])+(0.5*valuesOnGraph[i+1]);
		if(minimum>currentValue){
			minimum=currentValue;
			position=i;
		}
	}
	
	//sets direction for robot to continue
	float shiftAngle = 90 - (((anglesOnGraph[position+1]-anglesOnGraph[position])/2)+anglesOnGraph[position]);
	angleOfRotation=int(shiftAngle);
	locked=0;		
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function gets position on a plane from BNO055 senzor in quaternion and calculates angle on 2D plane (yaw) and also alerts if BNO055 sensor doesn't send data
void getAngleOnPlane(const sensor_msgs::Imu& msg)
{
	ros::Rate secondDelay(1);
	
	//racunam se yaw
	double siny_cosp = +2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y);
	double cosy_cosp = +1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z);
	double kot = atan2(siny_cosp, cosy_cosp);
	double kotkot = (kot*180)/3.14;
	currentAngle=kotkot;
	
	if(isnan(msg.orientation.x)){
		ROS_INFO("BNO055 DOWN!!!!");
		secondDelay.sleep();
		secondDelay.sleep();
		secondDelay.sleep();
	}
	
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function for display current settings
void displayData() {
	ROS_INFO("Speed: %d  Duration: %d Kot: %f Angle of goal: %f", directionCommand.speed, directionCommand.duration, currentAngle, goalAngle);
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//function that gets position of a detected face from a camera
void functionFacePosition(const roborodney::facePosition& msg)
{
    if (msg.p2.x>0 && msg.p2.y>0) 
    {
        ROS_INFO("Face detection!");
        detectedFace=true;
        xx1=msg.p1.x;
        yy1=msg.p1.y;
        xx2=msg.p2.x;
        yy2=msg.p2.y;     
        resx=msg.res.x;
        resy=msg.res.y;
    }
}


int main(int argc, char **argv)
{
	int counter=0;
	
	ros::init(argc, argv, "driveAuto");
	ros::NodeHandle nh;
	
	ros::Rate secondDelay(1);
	ros::Rate tenthOfSecondDelay(10);
	ros::Rate hundredthsOfSecondDelay(100);
	ros::Rate dvajsetinkasekundePostanek01(20);

	
	driving_command = nh.advertise<roborodney::drivingCommand>("driving_command",10);
	trayLeftRight = nh.advertise<std_msgs::String>("trayLeftRight", 10);
	surroundScanCommand = nh.advertise<std_msgs::String>("surroundScanCommand", 10);
	ros::Subscriber sensorDistance = nh.subscribe("laser_distance", 1, getSensorDistance);
	ros::Subscriber envoronmentPoints = nh.subscribe("surroundPoints", 1, gerEnvironmentalPoints);
	ros::Subscriber angleOnPlane = nh.subscribe("data", 1, getAngleOnPlane);
	ros::Subscriber facePosition = nh.subscribe("facePosition", 1, functionFacePosition);
	ros::Subscriber qrCode = nh.subscribe("barcode", 1, getQrCommand);
	ros::Publisher scannerCommand = nh.advertise<std_msgs::String>("pictureScanCommand", 10);
	
	std_msgs::String msg;
	msg.data="start";
	secondDelay.sleep();
	ros::spinOnce();
	
	directionCommand.direction=direction;
	directionCommand.speed=30;
	directionCommand.duration=20;

	secondDelay.sleep();
	ros::spinOnce();
	
 
	ROS_INFO("Robot is running.");
	
	goalAngle=currentAngle;
	
	int c=0;


	//loop that repeats as long as STOP qr code is not detected or program is not terminated with CTRL-C	
	while(nh.ok()){
		
		//this while statement is executed while surround scan
		while(scanInProgress==1){
			ROS_INFO("Surroundings scan in progress");
			ros::spinOnce();
			secondDelay.sleep();
		} 
		
		ros::spinOnce();
		
		//this if statement starts drive under certain conditions
		if(encounteredAnQrCode==0 && encounteredAnObstacle==0 && driveInProgress==0){	
			scannedQR=0;
			driveInProgress=1;
			ROS_INFO("Tray is moving.");
			msg.data="start";
			trayLeftRight.publish(msg);
			//Robot waits 2 seconds and then begins with driving
			for(int i=0;i<2;i++){ros::spinOnce();secondDelay.sleep();}
			ROS_INFO("Start driving.");
		}
		
		//this if statement is exectued if robot is driving
		if(encounteredAnQrCode==0 && encounteredAnObstacle==0 && driveInProgress==1){
			directionCommand.direction="forward";
			directionCommand.speed=20;
			directionCommand.duration=20;
			driving_command.publish(directionCommand);
			ros::spinOnce();
			hundredthsOfSecondDelay.sleep();
			forwardStepCounter++;
			//this if statement is executed if robot makes 300 moves while driving to check if direction is proper
			if(forwardStepCounter==300){
				ROS_INFO("Checking the driving angle.");
				ROS_INFO("Curent angle %f",currentAngle);
				ROS_INFO("Target angle %f", goalAngle);
				
				//calculates angle for turn
				int newStarting=currentAngle;
				int newEnd=goalAngle;
				int turn=newEnd-newStarting;
				if(turn>180){turn=turn-360;}
				if(turn<-180){turn=turn+360;}

				if(abs(turn)>2){
					ROS_INFO("Turning in the target angle.");
					for(int i=0;i<2;i++){ros::spinOnce();secondDelay.sleep();}
					robotRotation(turn);
					for(int i=0;i<2;i++){
						ros::spinOnce();
						secondDelay.sleep();
					}
					msg.data="start";
					trayLeftRight.publish(msg);
					//Robot waits 2 seconds and then begins with driving
					for(int i=0;i<2;i++){ros::spinOnce();secondDelay.sleep();}
				}
				forwardStepCounter=0;
				encounteredAnObstacle=0;
				driveInProgress==0;
			}
			continue;
			ROS_INFO("Continue driving.");
		}
		
		//this statement is executed if robot encounters obstacle
		if(encounteredAnObstacle==1){
			ROS_INFO("Ran into an obstacle.");
			driveInProgress=0;
			msg.data="stop";
			trayLeftRight.publish(msg);
			ros::spinOnce();
			hundredthsOfSecondDelay.sleep();
			//this if statement is executed if robot encounters obstacle and did not check for QR code yet
			if(scannedQR==0){
				ROS_INFO("Scanning for a QR code");
				msg.data="qrDetection";
				scannerCommand.publish(msg);
				//Robot waits 5 seconds for QR code
				for(int counter=0;counter<5;counter++){
					ros::spinOnce();
					secondDelay.sleep();
				}
				//this if sttatement executes if QR code is detected
				if(encounteredAnQrCode==1){
					if(qrCodeCommand=="STOP"){
						ROS_INFO("Found QR code STOP.");
						exit(0);
					}
					if(qrCodeCommand=="LEFT"){
						ROS_INFO("Found QR code LEFT");
						ROS_INFO("Current target angle %f",goalAngle);
						robotRotation(90);
						encounteredAnObstacle=0;
						goalAngle=goalAngle+90;
						if(goalAngle>180){goalAngle=goalAngle-360;}
						ROS_INFO("New target angle %f",goalAngle);
					}
					if(qrCodeCommand=="RIGHT"){
						ROS_INFO("Found QR code RIGHT");
						ROS_INFO("Current target angle %f",goalAngle);
						robotRotation(-90);
						encounteredAnObstacle=0;
						goalAngle=goalAngle-90;
						if(goalAngle<-180){goalAngle=goalAngle+360;}
						ROS_INFO("New target angle %f",goalAngle);
					}
					if(qrCodeCommand=="TURN"){
						ROS_INFO("Found QR code TURN");
						ROS_INFO("Current target angle %f",goalAngle);
						robotRotation(180);
						encounteredAnObstacle=0;
						goalAngle=goalAngle+180;
						if(goalAngle>180){goalAngle=goalAngle-360;}
						ROS_INFO("New target angle %f",goalAngle);
					} 
					encounteredAnQrCode=0;
					forwardStepCounter=0;
				}
				scannedQR=1; 
				continue;
			}
			
			//tihs if statement is executed if robot stops for obstacle and QR code is not detected - than scans the surroundings
			if(scannedSurround==0){
					ROS_INFO("Scanning the surroundings");
					msg.data="start";
					surroundScanCommand.publish(msg);
					scannedSurround=1;
					scanInProgress=1;
					ros::spinOnce();
					secondDelay.sleep();
					continue;
			} else {
				//after scans the surroundings robot turn for calculated angle
				if(turnedRobot==0){
					ROS_INFO("Turnig the robot for angle %d",angleOfRotation);
					robotRotation(angleOfRotation);
					scannedQR=0;
					turnedRobot=1;
					forwardStepCounter=0;
					secondDelay.sleep();
				} else {
					//this part executes after robot turns for ceratin angle to avoid obstacle
					turnedRobot=0;
					encounteredAnObstacle=0;
					scannedSurround=0;
					continue;
				}
			}
		}
		ros::spinOnce();		
	}
	ROS_INFO("Terminating the program.");
	return 0;
}
