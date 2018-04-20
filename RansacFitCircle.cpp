#include "RansacFitCircle.h"
RansacFitCircle::RansacFitCircle()
{

}

RansacFitCircle::~RansacFitCircle()
{
}

void RansacFitCircle::getBestFitCircle()
{
	getFeaturePoint(ptLeft,ptRight);
	pt = ptRight;
	randSample();
}
inline void RansacFitCircle::getCircle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& p3, cv::Point2d& center, double& radius)
{
	double x1 = p1.x;
	double x2 = p2.x;
	double x3 = p3.x;

	double y1 = p1.y;
	double y2 = p2.y;
	double y3 = p3.y;

	// PLEASE CHECK FOR TYPOS IN THE FORMULA :)
	center.x = (x1*x1 + y1 * y1)*(y2 - y3) + (x2*x2 + y2 * y2)*(y3 - y1) + (x3*x3 + y3 * y3)*(y1 - y2);
	if((2 * (x1*(y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2))!=0)		
		center.x /= (2 * (x1*(y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));
	if((x1*x1 + y1 * y1)*(x3 - x2) + (x2*x2 + y2 * y2)*(x1 - x3) + (x3*x3 + y3 * y3)*(x2 - x1)!=0)
		center.y = (x1*x1 + y1 * y1)*(x3 - x2) + (x2*x2 + y2 * y2)*(x1 - x3) + (x3*x3 + y3 * y3)*(x2 - x1);
	center.y /= (2 * (x1*(y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));

	radius = sqrt((center.x - x1)*(center.x - x1) + (center.y - y1)*(center.y - y1));
}

int RansacFitCircle::fitNumber(std::vector<cv::Point2d> pt, cv::Point2d center, double radius, int criteria)
{
	int num = 0;
	for (int i = 0; i < pt.size(); i++)
	{
		if (abs(distance(pt[i], center) - radius) < criteria)
			num++;
	}
	return num;
}

void RansacFitCircle::randSample()
{	

	int max_fit_num = 0;
	for (int i = 0; i < maxIteration; i++)
	{
		unsigned int idx1 = rand() % pt.size();
		unsigned int idx2 = rand() % pt.size();
		unsigned int idx3 = rand() % pt.size();

		if (idx1 == idx2) continue;
		if (idx1 == idx3) continue;
		if (idx3 == idx2) continue;

		// create circle from 3 points:
		cv::Point2d center; double radius;
		getCircle(pt[idx1], pt[idx2], pt[idx3], center, radius);
		int current_fit_num = fitNumber(pt, center, radius, critera);
		if (current_fit_num > max_fit_num)
		{
			max_fit_num = current_fit_num;
			bestCenter = center;
			bestRadius = radius;

		}
	}
}



inline double RansacFitCircle::distance(cv::Point2d pt1, cv::Point2d pt2)
{
	return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
}

void RansacFitCircle::getFeaturePoint(std::vector<cv::Point2d> &ptLeft,std::vector<cv::Point2d> &ptRight)
{
	//cv::GaussianBlur()
	uchar *p;
	//清楚之前的pt
	ptLeft.clear();
	ptRight.clear();
	cv::Mat img_display;

	cv::GaussianBlur(img, img, cv::Size(3, 3), 0, 0, 4);
	int scale = 1;
	//qDebug()<<"gaussian succ";
	int delta = 0;
	int ddepth = CV_16S;
	cv::Mat gradx, grady, abs_grady, abs_gradx, grad, bw;

	cv::Sobel(img, gradx, ddepth, 1, 0, 3, scale, delta, 4);
	cv::convertScaleAbs(gradx, abs_gradx);

	cv::Sobel(img, grady, ddepth, 0, 1, 3, scale, delta, 4);
	cv::convertScaleAbs(grady, abs_grady);
	/// 合并梯度(近似)
	cv::addWeighted(abs_gradx, 0.5, abs_grady, 0.5, 0, grad);
	cv::threshold(grad, bw, bwpara,255,0);
	//寻找特征点
	for (int i = 0; i < bw.rows; i++)
	{
		p = bw.ptr<uchar>(i);
		for (int j = 0; j < bw.cols; j++)
		{
			if (p[j] > 0)
			{
				ptLeft.push_back(cv::Point2d(j, i));
				break;
			}

		}
		for (int j = 0; j < bw.cols; j++)
		{
			if (p[bw.cols - j] > 0)
			{
				ptRight.push_back(cv::Point2d(bw.cols - j, i));
				break;
			}

		}
	}



}

