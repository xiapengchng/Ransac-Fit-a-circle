#ifndef DUIZHUN_H
#define DUIZHUN_H
#include<qdebug.h>
#include<math.h>
#include<opencv.hpp>
#include<cv.h>
#include<vector>
#define pi 3.1415926  
class GluePos
{
public:
	GluePos();
	~GluePos();
	int thresh;
	double degreeUp;
	double degreeDown;
	cv::Mat GluePosImg2;
	std::vector<cv::Point2d> ptLU;
	std::vector<cv::Point2d> ptRU;
	std::vector<cv::Point2d> ptLD;
	std::vector<cv::Point2d> ptRD;
	void getPosEdgePoint();
	void ransacFitLine(std::vector<cv::Point2d> points,cv::Vec4f &line);
	void splitDown(std::vector<cv::Point2d> &points,int direction);
	void showResult();
	void linePara2Pt(std::vector<cv::Point2d> points, cv::Point2d &pt1, cv::Point2d &pt2, cv::Vec4f &line);
};


#endif // !DUIZHUN_H