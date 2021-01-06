//Program scans picture and detects either QR code or face. It is written so that when via topic pictureScanCommand receives command, checks if it has to scan for QR code or face
//then proceeds apropriate action
//If program is required to scan for face, scans picture and via topic facePosition returns results, eithe points of face frame or zeroes
//If program is required to scan for QR code, scans picture and via topic barcode it retuns data found in code. If program does not detecct code, nothing is sent to topic.

#include <ros/ros.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <iostream>
#include <stdio.h>
#include "roborodney/facePosition.h"
#include "zbar.h"

using namespace std;
using namespace cv;

string face_cascade_name = "/root/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
std::string arg1;
std::string arg2;

zbar::ImageScanner scanner;

ros::Publisher facePositionPublish;
ros::Publisher barcodePublish; 

// Function Headers
void detectAndDisplay(Mat frame, Point* p1, Point* p2);


//function is detecting face on picture and if fonds it, in variable facePosition writes time, starting(upper left) and ending point(lower right) of the frame with face 
//and width and height of the frame
//face detection is done with openCV library
//if function detectAndDisplay returns point, program writes result in variable facePosition, otherwise it writes zeroes
//then variable facePosition is published on facePosition topic
//It is possible to found more than one face with detectAndDisplay, but in this case is importaint just to detect face, and it does not matter if there is more than one face on the picture
void faceDetection() {
	
	ros::Rate secondDelay(1);
	roborodney::facePosition facePosition;
		
	std::istringstream video_sourceCmd("0");
	int video_source;
	if(!(video_sourceCmd >> video_source)) ROS_INFO("Problem with camera.");
	cv::VideoCapture cap(video_source);
	if(!cap.isOpened()) ROS_INFO("Problem with camera.");
	
	
	cv::Mat dImg;
	cap >> dImg;
	cv::flip(dImg,dImg,-1);
	Point pt1; 
	Point pt2; 
	detectAndDisplay(dImg,&pt1,&pt2);
	Point pt3((pt1.x+pt2.x)/2,(pt1.y+pt2.y)/2);
	if(pt2.x > 0)
	{
		facePosition.time = ros::Time::now();
		facePosition.p1.x = pt1.x;
		facePosition.p1.y = pt1.y;
		facePosition.p2.x = pt2.x;
		facePosition.p2.y = pt2.y;
		facePosition.res.x = dImg.cols;
		facePosition.res.y = dImg.rows;
	} else {
		facePosition.time = ros::Time::now();
		facePosition.p1.x = 0;
		facePosition.p1.y = 0;
		facePosition.p2.x = 0;
		facePosition.p2.y = 0;
		facePosition.res.x = 0;
		facePosition.res.y = 0;
	}
	
	facePositionPublish.publish(facePosition);
	ros::spinOnce();  
	secondDelay.sleep();
	ros::spinOnce();
}

//function is detecteing QR code and if founds it, content of the code is written in barcode_string variable and sent via barcode topic
//detection is done with zbar library 
//if program founds moure than one code, it all sneds via topic and the program, that is subscribed to that topic, handles the codes
void QRDetection() {
	ros::Rate secondDelay(1);

	std::istringstream video_sourceCmd("0");
	int video_source;
	if(!(video_sourceCmd >> video_source)) ROS_INFO("Problem with camera.");
	cv::VideoCapture cap(video_source);
	if(!cap.isOpened()) ROS_INFO("Problem with camera.");
	cv::Mat frame;
	sensor_msgs::ImagePtr msg;
	cap >> frame;
	if(!frame.empty())
	{
		cv::flip(frame,frame,-1);
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		cv_bridge::CvImageConstPtr cv_image;
		cv_image = cv_bridge::toCvShare(msg, "mono8");
		zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,cv_image->image.cols * cv_image->image.rows);
		scanner.scan(zbar_image);
		int found=0;
		for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
			{
				std::string barcode = symbol->get_data();
				std_msgs::String barcode_string;
				barcode_string.data = barcode;
				barcodePublish.publish(barcode_string);
				found=1;
			}
	}
	ros::spinOnce();  
	secondDelay.sleep();
	ros::spinOnce();  
}


//function that receives command for scanning
void strsken(const std_msgs::String& msg)
{
	if(msg.data=="faceDetection"){
		ROS_INFO("Detection of faces.");
		faceDetection();
	}
	
	if(msg.data=="qrDetection"){
		ROS_INFO("Detection of QR codes.");
		QRDetection();
	}
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pictureScan");
    ros::NodeHandle nh;

	//program receives comamd for scan (face or qr detection)
	ros::Subscriber ukaz = nh.subscribe("pictureScanCommand", 1000, strsken);
	
	//topic for sending the data about face scan
    facePositionPublish = nh.advertise<roborodney::facePosition>("facePosition",1000);
	
	//topic for sending the data about QR scan
	barcodePublish = nh.advertise<std_msgs::String>("barcode", 10);
	
	//qualifires for face scan that comes with openCV library
    face_cascade.load(face_cascade_name);
	
	//loop that ensures that the program is running util termination
    ros::spin();
}


//---------------------------------------
// Function detectAndDisplay form page https://docs.opencv.org/2.4/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html
void detectAndDisplay(Mat frame, Point* p1, Point* p2)
{
    std::vector<Rect> faces;
    Mat frame_gray;
    Mat crop;
    Mat res;
    Mat gray;
	
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;
    cv::Rect roi_c;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element    //waitKey(0);

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)
    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);
        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, CV_BGR2GRAY); // Convert cropped image to Grayscale

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
		
        *p1 = pt1;
        *p2 = pt2;
    } 
}
