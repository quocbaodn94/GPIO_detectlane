#ifndef IMAGE_FUNCTION_H
#define IMAGE_FUNCTION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include "image_function.h"
using namespace std;
using namespace cv;

typedef struct StraightLine{
    Point_<int> A;
    Point_<int> B;
}StraightLine;

//to calculate edge and mask most of non-road area in the image
Mat calcEdgeAndROI(Mat gray);

list<float> selectRadians(vector<Vec2f> lines);

StraightLine calcX1Y1X2Y2(float rad, float rho);

Mat forLine(StraightLine l, Mat img);

float findMaxRads(list<float> rads);

float findMinRads(list<float> rads);

Point_<int> findIntersection(StraightLine l1, StraightLine l2);

#endif // IMAGE_FUNCTION_H
