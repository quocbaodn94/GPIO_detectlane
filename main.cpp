#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_function.h"
#include <list>
#include <sstream>
#include<wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace cv;
int i,j;

int main()
{
    int fd;
    int old_angle = 90;
    string temp;
    //char buffer;
    if(wiringPiSetup() == -1){
        cout << "Unble to start wiringPi";
        return 0;
    }
    if((fd = serialOpen("/dev/ttyACM0", 115200)) < 0){
        cout << "Unable to open serial device";
        return 0;
    }

   int tTime=0, k = 0;
   wiringPiSetup();
   VideoCapture cap(0);
   cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
   while(cap.isOpened()){
       Mat img;
       cap >> img;   // Read the file

       if(! img.data )         // Check for invalid input
       {
           cout <<  "Could not open or find the image" << endl ;
           return -1;
       }

       Mat gray;
       cvtColor(img, gray, CV_RGB2GRAY);

       Mat edges;
       edges = calcEdgeAndROI(gray);

       vector<Vec2f> lines;
       HoughLines(edges, lines, 6, CV_PI/180, 180);

       list<float> rads = selectRadians(lines);

       bool flag1 = true, flag2 = true;

       //find the best line to the left and right of the vehicle
       StraightLine leftlane, rightlane;
       if(rads.size() > 0 ){
           float maxr = findMaxRads(rads);
           float minr = findMinRads(rads);
           StraightLine l;

           for(size_t i=0; i<lines.size(); i++){
               float theta = lines[i][1];
               float rho = lines[i][0];
               //for RIGHT LANE
               if(maxr == theta and flag1 == true){
                   l = calcX1Y1X2Y2(maxr, rho);
                   if((flag2 == false && maxr-minr > 0.5) || flag2 == true || (flag1== false && flag2 == false)){
                       img = forLine(l, img);
                       rightlane.A.x = l.A.x;
                       rightlane.A.y = l.A.y;
                       rightlane.B.x = l.B.x;
                       rightlane.B.y = l.B.y;
                       flag1 = false;
                   }
               }

               //for LEFT LANE
               if(minr == theta and flag2 == true){
                   l = calcX1Y1X2Y2(minr, rho);
                   if((flag1 == false && maxr-minr > 0.5) || flag1 == true || (flag1== false && flag2 == false)){
                       leftlane.A.x = l.A.x;
                       leftlane.A.y = l.A.y;
                       leftlane.B.x = l.B.x;
                       leftlane.B.y = l.B.y;
                       img = forLine(l, img);
                       flag2 = false;
                   }
               }
           }
       }
       Point_<int> I = findIntersection(leftlane, rightlane);
       circle(img, I, 10, CV_RGB(255,0,0),5);
       line(img, Point(j/2,i), Point(j/2,0),CV_RGB(255,0,0), 5);
       line(img, Point(j/2,i), I, CV_RGB(255,0,0), 5);
       int angle = atan2(I.x-j/2, i - I.y) * 180 / CV_PI;
       cout << angle <<  endl;
       fflush(stdout);
       ostringstream lable;
       lable << angle;
       putText(img, lable.str(), Point(j/2, i/2), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0,255,0), 2.0);
       k++;
       ostringstream lable2;
       lable2 << k;
       putText(img, lable2.str(), Point(j/4, i/4), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0,255,0), 2.0);
       if(millis() - tTime > 1000){
           tTime = millis();
           k = 0;
       }
       // Create a window for display.
       //namedWindow( "Display window", CV_WINDOW_NORMAL );
       //resizeWindow("Display window", 400, 400);
       //imshow( "Display window", img );
       //imwrite("hinh.png", img);
       /*if(serialDataAvail(fd)){
           while(serialDataAvail(fd)){
               buffer = serialGetchar(fd);
               temp = temp + buffer;
           }
           if(temp.size()){
               cout << temp;
               fflush(stdout);
           }
       }*/
       if(abs(angle) < 10 ){
           angle = 0;
       }

       if(abs(angle - old_angle) > 5 && angle < 30 && angle > -30){
           char tt[10];
           sprintf(tt, "%d", angle);
           serialPuts(fd, tt);
           serialPutchar(fd, '.');
           old_angle = angle;
       }
       if((char)49== waitKey(1)){// Wait for a keystroke in the window
           break;
       }
   }
    return 0;
}

