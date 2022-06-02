#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <sstream>

// using namespace std;
// using namespace cv;
// using namespace std_msgs;
// using namespace cv_bridge;
// using namespace image_transport;
// using namespace sensor_msgs;
// using namespace ros;


void callback(const sensor_msgs::ImageConstPtr& msg) {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr; 
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frameGray, cdst, cdstP;
    cvtColor(cv_ptr->image, frameGray, cv::COLOR_BGR2GRAY);
      
    GaussianBlur(frameGray,frameGray, cv::Size(3,3), 0);    
    cv::Canny(frameGray,frameGray, 100, 100*3, 3);
    threshold(frameGray,frameGray, 200, 255,CV_THRESH_BINARY); 

    cvtColor(frameGray, cdstP, cv::COLOR_GRAY2BGR);
    cvtColor(frameGray, cdst, cv::COLOR_GRAY2BGR);

    
    //finding all contours in the image
    /** method1 */
    // std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    // HoughLinesP(frameGray, linesP, 1, CV_PI/180, 10, 200, 10); // runs the actual detection
    // // Draw the lines
    // for(size_t i = 0; i < linesP.size(); i++ ) {
    //     if (linesP.size()>3 ) {
    //         cv::Vec4i l = linesP[i];
    //         line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    //     }
    //   }
    /** method2 */
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    HoughLines(frameGray, lines, 1, CV_PI/180, 100, 0, 0); // runs the actual detection

    // Draw the lines
    if (lines.size()>3) {
        cv::Point C, P[500], pt1[500], pt2[500], f1[500], f2[500];				//P's of intersection, array arbitrary size 500
        double numer3, denom3, numer4, m1, m2, m3, m4, m5, m6, denom1, denom2, denom4, denom5;	//more doubles
        int numlines = 0;       
        
        for( size_t i = 0; i < lines.size(); i++ ) {
            float rho = lines[i][0], theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1[i].x = cvRound(x0 + 1000*(-b));
            pt1[i].y = cvRound(y0 + 1000*(a));
            pt2[i].x = cvRound(x0 - 1000*(-b));
            pt2[i].y = cvRound(y0 - 1000*(a));
        
        	for (size_t i = 0; i < lines.size(); i++) {
                line(cdst, pt1[i], pt2[i], cv::Scalar(255,0,0), 3, cv::LINE_AA);
            }
            std::cout<<(pt1[i])<<" "<<(pt1[i].y)<<" "<<(pt2[i].x)<<" "<<(pt2[i].y)<<std::endl;

        }

        for (size_t start = 0; start < lines.size() - 1; start++) {																	
            for (size_t i = start + 1; i < lines.size(); i++) {
                denom1 = (pt2[start].x - pt1[start].x);				
                denom2 = (pt2[i].x - pt1[i].x);								
                if (denom1 == 0.0) { denom1 = 0.00001; }				
                if (denom2 == 0.0) { denom2 = 0.00001; }					
                m1 = (pt2[start].y - pt1[start].y) / denom1;				
                m2 = (pt2[i].y - pt1[i].y) / denom2;						
                m3 = -(m1*m2);												

                if (m3 > 0.1  && m3 < 4.0) {			
                    f1[numlines] = pt1[start];  
                    f2[numlines] = pt2[start];
                    numlines++;               
                    f1[numlines] = pt1[i];  
                    f2[numlines] = pt2[i];
                    numlines++;			
                }
            }
            
        }     


        if (numlines > 3) {																
            for (size_t i = 0; i < numlines - 1; i++) {															
                for (size_t r = i + 1; r < numlines; r++) {														
                    if (f1[i].x == f1[r].x && f1[i].y == f1[r].y) {															
                        for (size_t m = r; m < numlines - 1; m++) {														
                            f1[m] = f1[m + 1];									
                            f2[m] = f2[m + 1];									
                        }
                        numlines--;
                    }
                }
            }	
        }
        for (size_t i = 0; i < numlines; i++) {			
			line(cdst, f1[i], f2[i], cv::Scalar(0, 255, 0), 3, cv::LINE_AA);			
        }																
        // int counter = 0;														//
        // for (size_t start = 0; start < numlines - 1; start++) {																		// Double loop system for comparisons, same as before
        //     for (size_t i = start + 1; i < numlines; i++) {																	//
        //         denom4 = (f2[start].x - f1[start].x);							//
        //         denom5 = (f2[i].x - f1[i].x);									//
        //         if (denom4 == 0.0) { denom4 = 0.00001; }						//
        //         if (denom5 == 0.0) { denom5 = 0.00001; }						// Finding the gradient between the f's, used as the order (horizontal, vertical) of the array may not be right for what we need to do 
        //         m4 = (f2[start].y - f1[start].y) / denom1;						// below. Hence check
        //         m5 = (f2[i].y - f1[i].y) / denom2;								//
        //         m6 = -(m4*m5);													//
        //         if (m6 > 0.1  && m6 < 4.0) {
        //             numer3 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i].x - f2[i].x)) - (((f1[start].x - f2[start].x)*((f1[i].x*f2[i].y) - (f1[i].y*f2[i].x))));
        //             numer4 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i].y - f2[i].y)) - (((f1[start].y - f2[start].y)*((f1[i].x*f2[i].y) - (f1[i].y*f2[i].x))));
        //             denom3 = ((f1[start].x - f2[start].x)*(f1[i].y - f2[i].y)) - ((f1[start].y - f2[start].y)*(f1[i].x - f2[i].x));
        //             P[counter].x = (numer3 / denom3);
        //             P[counter].y = (numer4 / denom3);
        //             if ((P[counter].x > 0 && P[counter].x < 640) || (P[counter].y > 0 && P[counter].y < 480)) {
        //                 counter++;												// increment counter
        //             }
        //         } else {																//else, use "start" and "i+1"
        //             numer3 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i + 1].x - f2[i + 1].x)) - (((f1[start].x - f2[start].x)*((f1[i + 1].x*f2[i + 1].y) - (f1[i + 1].y*f2[i + 1].x))));
        //             numer4 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i + 1].y - f2[i + 1].y)) - (((f1[start].y - f2[start].y)*((f1[i + 1].x*f2[i + 1].y) - (f1[i + 1].y*f2[i + 1].x))));
        //             denom3 = ((f1[start].x - f2[start].x)*(f1[i + 1].y - f2[i + 1].y)) - ((f1[start].y - f2[start].y)*(f1[i + 1].x - f2[i + 1].x));
        //             P[counter].x = (numer3 / denom3);
        //             P[counter].y = (numer4 / denom3);
        //             if ((P[counter].x > 0 && P[counter].x < 640) || (P[counter].y > 0 && P[counter].y < 480)) {
        //                 counter++;												// increment counter
        //             }
        //         }

		// 	}
        // }   
        printf("Numlines = %d\n", numlines);		//Checking value of numlines
						
    }
       

    cv::imshow("image window", cdst);
    cv::waitKey(1);
}


int main(int argc, char** argv) {
    // init with a name
    ros::init(argc, argv, "detect_openings");
    // create a node
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/camera/color/image_raw", 1, callback);
    image_transport::Publisher pub_image = it.advertise("/detect_openings/output_video", 1);
    

    // // publishing rate 1 Hz
    // ros::Rate loop_rate(1);

    // while (ros::ok()) {
    //     // publish
    //     // pub_image.publish(cv_ptr->toImageMsg());

    //     // check status
    //     ros::spinOnce();

    //     // sleep
    //     loop_rate.sleep();
    // }
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}







// void callback(const sensor_msgs::ImageConstPtr& msg) {
//     cv_bridge::CvImagePtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
// }


// void detectOpenings(cv::Mat frame) {
//     // cv::Mat frameGray, cdstP;
//     // cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
    
//     // GaussianBlur(frameGray,frameGray, cv::Size(3,3), 0);    
//     // cv::Canny(frameGray,frameGray, 100, 100*3, 3);
//     // threshold(frameGray,frameGray, 200, 255,CV_THRESH_BINARY); 

//     // cvtColor(frameGray, cdstP, cv::COLOR_GRAY2BGR);
    
//     // //finding all contours in the image
//     // std::vector<cv::Vec4i> linesP; // will hold the results of the detection
//     // HoughLinesP(frameGray, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
//     // // Draw the lines
//     // for(size_t i = 0; i < linesP.size(); i++ ) {
//     //     cv::Vec4i l = linesP[i];
//     //     line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
//     // }

//     // // Vector for storing contour
//     // std::vector< std::vector <cv::Point> > contours; 
//     // std::vector< cv::Vec4i > hierarchy;

//     // imshow("camCapture (frameGray)", cdstP);
//     // cv::waitKey(1);
// }

















/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









































// //	Final 8
// //	The final version of the image detection program I developed. This program detects red rectangle 
// //	targets and outputs a centre point value, designed for use with a UAV for augmentation of control 
// //	for payload delivery
// //	Oliver Clark, University of Liverpool, ollierclark101@gmail.com, +447929994837


// // #include <opencv2/cvaux.h>							//
// // #include <opencv/highgui.h>							//
// #include <opencv2/core/core.hpp>						//
// // #include <opencv2/cxcore.h>							// OpenCV includes, links to the libraries
// #include <opencv2/highgui/highgui.hpp>				//
// #include <stdio.h>									//
// #include "opencv2/opencv.hpp"						//
// #include <stdlib.h>									//
// #include <iostream>									//

// using namespace cv;									// class type, means you dont need to call every function including its full library info like "cv::inRange(" becomes "inRange("
// using namespace std;								//

// int main(int argc, char** argv)    //for conversion between Video and PNG (int argc, char** argv)
// {
// 	VideoCapture cap(0);		//VideoCapture is OpenCV function that uses webcam stream - cap(0) <-- zero being the first camera, 1 being 2nd camera if multiple
// 	if (!cap.isOpened())		//if not cap opened
// 	{
// 		cout << "Cannot Open Video Camera" << endl;		//disp error message
// 		cin.ignore();		//wait until key hit
// 		return -1;			//close program, return -1
// 		}

// 	for (;;)				//for loop, keeps the program running until the esc key break (at the end)
// 	{	
// 		Mat Original;		//Declare Matrix, Original (ie. camera feed)
// 		namedWindow("Original", 1);		//declare window
// 		cap >> Original;		//put "cap" (captured images from webcam) into the Matrix for processing
		
// 		Mat BW;		//Declare Matrix BW (to be used for the red filter)
// 		namedWindow("Red Colour Detect, GreyScale", 1);

// 		Mat Processing;		// (to be used with canny)
// 		namedWindow("Processing", 1);
		
// 		//Colour Filtration
// 		inRange(Original, Scalar(0, 0, 125), Scalar(80, 80, 256), BW);		//Colour filtration		(source, low threshold, high threshold, Output)
// 		//Image Processing
// 		GaussianBlur(BW, BW, Size(15, 15), 1.5, 1.5);		//Gaussian blur
// 		erode(BW, BW, Mat(), Point(-1, -1));				//Erosion
// 		dilate(BW, BW, Mat(), Point(-1, -1), 10, 1, 10);	//Dilation

// 		//Canny Edge Detection
// 		Canny(BW, Processing, 0, 30, 3);

// 		//Vector of lines, followed by Houghlines
// 		vector<Vec2f> lines;											//storage for the HoughLines
// 		HoughLines(Processing, lines, 1, CV_PI / 180, 100, 0, 0);		//Houghlines Function(Input, Storage(output), double rho (keep 1), double theta (keep pi/180), threshold value (pix), keep 0, keep 0)
	
// 		printf("%d Lines Detected.\n", lines.size());			//No. lines detected

// 		if (lines.size() > 3)								//if the number of lines is greater than 3, continue processing, otherwise skip. This was put in because the program errored later due to loops having values i<array-1 if the array contained no elements
// 		{
// 			//Declaring variables
// 			Point C;					//Centre point
// 			Point P[500];				//P's of intersection, array arbitrary size 500
// 			double numer3, denom3, numer4;		//doubles, used later, for calcs of gradients
// 			Point pt1[500], pt2[500];		//output hough lines end points
// 			double m1, m2, m3, m4, m5, m6, denom1, denom2, denom4, denom5;	//more doubles
// 			Point f1[500], f2[500];			//two arrays, for confirmed perpendicular lines f1-->pt1  f2-->pt2
// 			int numlines = 0;        // Number of intersecting lines detected.
			

// 			// Show how many lines were detected by opencv
			
// 			//		printf("Lines Detected = %d\n", lines.size());			//uncomment if you want to check this variable


// 			for (size_t i = 0; i < lines.size(); i++)			//loops through each line
// 			{
// 				// Convert from Polar to Cartesian Co-ordinates.
// 				float rho = lines[i][0], theta = lines[i][1];					//
// 				double a = cos(theta), b = sin(theta);							//
// 				double x0 = a*rho, y0 = b*rho;									// This section does the conversion from lines array polar into cartesian end points of the lines, arbitrary length 1000, can be changed but not necessary
// 				pt1[i].x = cvRound(x0 + 1000 * (-b));							//
// 				pt1[i].y = cvRound(y0 + 1000 * (a));							//
// 				pt2[i].x = cvRound(x0 - 1000 * (-b));							//
// 				pt2[i].y = cvRound(y0 - 1000 * (a));							//
// 			}
// 			for (size_t i = 0; i < lines.size(); i++)
// 			{
// 				line(Original, pt1[i], pt2[i], Scalar(255, 255, 0), 2, LINE_AA);		//draws all detected lines in CYAN
// 			}


// 			for (size_t start = 0; start < lines.size() - 1; start++)			//
// 			{																	//
// 				for (size_t i = start + 1; i < lines.size(); i++)				// Double looping system for comparisons
// 				{
// 					denom1 = (pt2[start].x - pt1[start].x);						//Gradient comparison between the lines start and i
// 					denom2 = (pt2[i].x - pt1[i].x);								//
// 					if (denom1 == 0.0) { denom1 = 0.00001; }					//allows for complete comparison between all elements
// 					if (denom2 == 0.0) { denom2 = 0.00001; }					//
// 					m1 = (pt2[start].y - pt1[start].y) / denom1;				//
// 					m2 = (pt2[i].y - pt1[i].y) / denom2;						//
// 					m3 = -(m1*m2);												//

// 					// Check if these two lines intersect at ~90 deg. If correct save into arrays f1 (pt1) and f2 (pt2). I.E. end points of the lines
					
// 					if (m3 > 0.1  && m3 < 4.0)			// Constraints for angles. Change for a more stringent 90 degrees (tend to 1) for less stringent, widen constraint
// 					{										
// 						// If Lines intersect - increment the counter.
// 						f1[numlines] = pt1[start];    // And save the lines to our final output arrays.
// 						f2[numlines] = pt2[start];
// 						numlines++;                // Lines intersect - increment the counter.
// 						f1[numlines] = pt1[i];    // And save the lines to our final output arrays.
// 						f2[numlines] = pt2[i];
// 						numlines++;
// 					}
// 				}
// 			}



// 			printf("Numlines = %d\n", numlines);		//Checking value of numlines
			
// 			if (numlines > 3)														//
// 			{																		//
// 				for (size_t i = 0; i < numlines - 1; i++)							//
// 				{																	//
// 					for (size_t r = i + 1; r < numlines; r++)						// Removes duplicates from the array, makes things simpler later on LEAVE THIS IN!
// 					{																//
// 						if (f1[i].x == f1[r].x && f1[i].y == f1[r].y)				//
// 						{															//
// 							for (size_t m = r; m < numlines - 1; m++)				//
// 							{														//
// 								f1[m] = f1[m + 1];									//
// 								f2[m] = f2[m + 1];									//
// 							}
// 							numlines--;
// 							//				printf("Numlines is currently %d\n", numlines);				//uncomment if you want to check this variable
// 						}
// 					}
// 				}

// 				//	printf("Numlines = %d\n", numlines);
// 				//	for (size_t r = 0; r < numlines; r++)
// 				//	{
// 				//		printf("x[%d] = %5d  y[%d] = %5d\n", r, f1[r].x, r, f1[r].y);					//uncomment if you want to check this variable
// 				//	}
													

// 				// Section for finding intersection points

// 				for (size_t i = 0; i < numlines; i++)									//
// 				{																		//
// 					line(Original, f1[i], f2[i], Scalar(0, 255, 0), 3, LINE_AA);			// Display lines found from gradient comparison. Here it should only be the 4 (or maybe double detections) lines around the 
// 				}																		// rectangle.
// 				int counter = 0;														//
// 				for (size_t start = 0; start < numlines - 1; start++)					//
// 				{																		// Double loop system for comparisons, same as before
// 					for (size_t i = start + 1; i < numlines; i++)						//
// 					{																	//

// 						denom4 = (f2[start].x - f1[start].x);							//
// 						denom5 = (f2[i].x - f1[i].x);									//
// 						if (denom4 == 0.0) { denom4 = 0.00001; }						//
// 						if (denom5 == 0.0) { denom5 = 0.00001; }						// Finding the gradient between the f's, used as the order (horizontal, vertical) of the array may not be right for what we need to do 
// 						m4 = (f2[start].y - f1[start].y) / denom1;						// below. Hence check
// 						m5 = (f2[i].y - f1[i].y) / denom2;								//
// 						m6 = -(m4*m5);													//
// 						if (m6 > 0.1  && m6 < 4.0)										// if between, use "start" and "i" 
// 						{
// 							numer3 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i].x - f2[i].x)) - (((f1[start].x - f2[start].x)*((f1[i].x*f2[i].y) - (f1[i].y*f2[i].x))));
// 							numer4 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i].y - f2[i].y)) - (((f1[start].y - f2[start].y)*((f1[i].x*f2[i].y) - (f1[i].y*f2[i].x))));
// 							denom3 = ((f1[start].x - f2[start].x)*(f1[i].y - f2[i].y)) - ((f1[start].y - f2[start].y)*(f1[i].x - f2[i].x));
// 							P[counter].x = (numer3 / denom3);
// 							P[counter].y = (numer4 / denom3);
// 							if ((P[counter].x > 0 && P[counter].x < 640) || (P[counter].y > 0 && P[counter].y < 480))			// if within the size of the camera feed (640x480)
// 							{
// 								counter++;												// increment counter
// 							}
// 						}
// 						else
// 						{																//else, use "start" and "i+1"
// 							numer3 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i + 1].x - f2[i + 1].x)) - (((f1[start].x - f2[start].x)*((f1[i + 1].x*f2[i + 1].y) - (f1[i + 1].y*f2[i + 1].x))));
// 							numer4 = (((f1[start].x*f2[start].y) - (f1[start].y*f2[start].x))*(f1[i + 1].y - f2[i + 1].y)) - (((f1[start].y - f2[start].y)*((f1[i + 1].x*f2[i + 1].y) - (f1[i + 1].y*f2[i + 1].x))));
// 							denom3 = ((f1[start].x - f2[start].x)*(f1[i + 1].y - f2[i + 1].y)) - ((f1[start].y - f2[start].y)*(f1[i + 1].x - f2[i + 1].x));
// 							P[counter].x = (numer3 / denom3);
// 							P[counter].y = (numer4 / denom3);
// 							if ((P[counter].x > 0 && P[counter].x < 640) || (P[counter].y > 0 && P[counter].y < 480))			// if within the size of the camera feed (640x480)
// 							{
// 								counter++;												// increment counter
// 							}
// 						}

// 					}
// 				}

// 				//	printf("Final value for counter = %d\n", counter);							//uncomment if you want to check this variable

// 				for (size_t i = 0; i < counter - 1; i++)								//
// 				{																		//
// 					for (size_t r = i + 1; r < counter; r++)							//
// 					{																	// Remove duplicates from the P array as was done before with f1 and f2
// 						if (P[i].x == P[r].x && P[i].y == P[r].y)						// if P = next P
// 						{																//
// 							for (size_t m = r; m < counter - 1; m++)					//
// 							{															//
// 								P[m] = P[m + 1];										// Make P = next P
// 							}															//
// 							counter--;													// Reduce size of array by 1
// 							//printf("Counter (points) is now %d\n", counter);									//uncomment if you want to check this variable
// 						}
// 					}
// 				}





// 				for (size_t i = 0; i < counter; i++){
// 					circle(Original, P[i], 3, cv::Scalar(255, 0, 0), 3, 8, 0);			// Draws points of intersection
// 				}

// 				int xmax = -999999, xmin = 999999, ymax = -999999, ymin = 999999;		// Starting values
// 				for (size_t i = 0; i < counter; i++)
// 				{
// 					if (P[i].x < xmin){ xmin = P[i].x; }								// Finds smallest x
// 					if (P[i].x > xmax){ xmax = P[i].x; }
// 					if (P[i].y < ymin){ ymin = P[i].y; }					
// 					if (P[i].y > ymax){ ymax = P[i].y; }								// Finds biggest y
// 				}
// 				//		printf("MinX=%d  MaxX=%d  MinY=%d  MaxY=%d\n", xmin, xmax, ymin, ymax);					//uncomment if you want to check this variable


// 				C.x = round(((xmax - xmin) / 2) + xmin);								// x value of C = average of xmax and xmin (plus xmin to place it in centre)
// 				C.y = round(((ymax - ymin) / 2) + ymin);								// y value of C = average of ymax and ymin (plus ymin to place it in centre)

// 				circle(Original, C, 10, cv::Scalar(0, 0, 0), 3, 8, 0);					// Disp C on Original. Circle, size 10, thickness 3

// 				printf("Centre X =%d    Centre y =%d\n", C.x, C.y);						// Centrepoints out, x and y



// 			}
// 		}																	// provides a 0.25s wait betweeen loops. Comment out for full speed *I used this to help see what was going on in real time*
// 		imshow("Original", Original);
// 		imshow("Processing", Processing);												// shows the windows
// 		imshow("Red Colour Detect, GreyScale", BW);
// 		//		for (size_t r = 0; r < counter; r++)
// 		//		{																		// uncomment for P point data
// 		//			printf("P.x[%d] = %d  P.y[%d] = %d\n", r, P[r].x, r, P[r].y);
// 		//		}
// 		if (waitKey(30) >= 1) break;													// If ESC key hit, break from program
// 	}
// 	return 0;																			//return output 0

// }