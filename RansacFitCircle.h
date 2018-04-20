#pragma once
#ifndef RANSACFITCIRCLE_H
#define RANSACFITCIRCLE_H
#include<vector>
#include<opencv.hpp>
#include "cv.h"
#include "highgui.h"
#include "math.h"
#include "cxcore.hpp"
//using namespace cv ;
using namespace std;
class RansacFitCircle
{
public:
	RansacFitCircle();
	//RansacFitCircle();
	~RansacFitCircle();
	std::vector<cv::Point2d> pt;
	cv::Point2d bestCenter;
	cv::Mat img;


	double bestRadius;
	int maxIteration;
	double critera;
	int bwpara;

	std::vector<cv::Point2d> ptLeft;
	std::vector<cv::Point2d>  ptRight;
	cv::Point bestMatchPoint;
	cv::Point destination_pt;



public:
	void getBestFitCircle();
	//3 Point to form a circle
	inline void getCircle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& p3, cv::Point2d& center, double& radius);
	int fitNumber(std::vector<cv::Point2d> pt, cv::Point2d center, double radius, int criteria);
	void randSample();
	//caculate two point distance
	inline double distance(cv::Point2d pt1, cv::Point2d pt2);
	//from an Mat to get the circle feature point by using sobel 
	void getFeaturePoint(std::vector<cv::Point2d>&ptLeft,std::vector<cv::Point2d> &ptRight);

private:

};

#endif


/////