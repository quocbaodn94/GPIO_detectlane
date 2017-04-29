#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include "image_function.h"
using namespace std;
using namespace cv;
extern int i,j;
Mat calcEdgeAndROI(Mat gray)
{
    Mat edges;
    Canny(gray, edges, 100, 110);
    i = edges.rows;
    j = edges.cols;
    for(int x=0; x < i/2; x++){
        for(int y=0; y < j; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }

    /*for(int x=i*85/100; x < i; x++){
        for(int y=0; y < j; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }

    for(int x=i*65/100; x < i; x++){
        for(int y=j*40/100; y < j*70/100; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }*/

    /*int k = j*1/5;
    int start = i/2;

    while(k>0){
        for(int x=start; x < i; x++){
            for(int y=0; y < k; y++){
                edges.at<uchar>(x,y)= 0;
            }
            k = k - 2;
        }
    }

    k = j*4/5;
    while(k<=j){
        for(int x=start; x < i; x++){
            for(int y=k; y < j; y++){
                edges.at<uchar>(x,y)= 0;
            }
            k = k + 2;
        }
    }*/

    return edges;
}

list<float> selectRadians(vector<Vec2f> lines)
{
    list<float> rads;
    for(size_t i=0; i<lines.size(); i++){
        float theta = lines[i][1];
        float radian = theta;
        theta = theta * 180 / CV_PI;
        if(theta > 10 and theta < 80){
            rads.push_back(radian);
        }
        if(theta > 100 and theta < 170){
            rads.push_back(radian);
        }
    }
    return rads;
}

StraightLine calcX1Y1X2Y2(float rad, float rho)
{
    StraightLine line;
    float a = cos(rad);
    float b = sin(rad);\
    float X0 = a*rho;
    float Y0 = b*rho;
    line.A.x = int(X0 + 2000*(-b));
    line.A.y = int(Y0 + 2000*(a));
    line.B.x = int(X0 - 2000*(-b));
    line.B.y = int(Y0 - 2000*(a)) ;
    return line;
}

Mat forLine(StraightLine l, Mat img){
    line(img, l.A, l.B,CV_RGB(0,255,255),5);
    return img;
}

float findMaxRads(list<float> rads)
{
    list<float>::iterator position = rads.begin();
    float max = *position;
    for(size_t i=1; i< rads.size()-1; i++){
        position++;
        if(*position > max){
            max = *position;
        }
    }
    return max;
}
float findMinRads(list<float> rads)
{
    list<float>::iterator rad = rads.begin();
    float min = *rad;
    for(size_t i=1; i< rads.size()-1; i++){
        rad++;
        if(*rad < min){
            min = *rad;
        }
    }
    return min;
}

Point_<int> findIntersection(StraightLine l1, StraightLine l2)
{
    Point_<int> I;
    float a1, a2;
    Point_<float> a = l1.A;
    Point_<float> b = l1.B;
    Point_<float> c = l2.A;
    Point_<float> d = l2.B;
    a1 = ((a.y-b.y)/(a.x-b.x)); //slope of the line 1
    a2 = ((c.y-d.y)/(c.x-d.x)); //slope of the line 2
    if(a1 != a2)
    {
        I.x = (a1*a.x - a2*c.x - a.y + c.y)/(a1-a2);
        I.y = a1*(I.x-a.x) + a.y;
    }
    return I;
}

