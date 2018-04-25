#include<test.h>

GluePos::GluePos()
{

}

GluePos::~GluePos()
{

}

void GluePos::getPosEdgePoint()
{
	cv::Mat dst;
	cv::threshold(GluePosImg2, dst, thresh, 255, 0);
	//cv::imshow("1", dst);
	for (int i = 0; i < dst.rows; i++)
	{
		uchar *p;
		p = dst.ptr<uchar>(i);
		for (int j = 0; j<dst.cols; j++)
		{

			if (p[j] == 0)
			{
				if (j == 0)
					break;
				//qDebug() << "j:" << j << "i:" << i;
				ptLU.push_back(cv::Point2d(j, i));
				break;
			}
		}

		for (int j = 0; j < dst.cols; j++)
		{

			if (p[dst.cols - j] == 0)
			{
				if (j == 0||j==1)
					break;
				ptRU.push_back(cv::Point2d(dst.cols - j, i));
				break;
			}
		
		}
	}

	//将LU和RU分割成2个部分
	for (int i = 0; i < ptLU.size(); i++)
	{
		if (ptLU[i + 1].y - ptLU[i].y > 5)
		{
			ptLD.insert(ptLD.end(), ptLU.begin()+i, ptLU.end());
			ptLU.erase(ptLU.begin() + i, ptLU.end());
			break;
		}
		//qDebug() << "ptLU x:"<<ptLU[i].x;
	}

	for (int i = 0; i < ptRU.size(); i++)
	{
		if (ptRU[i + 1].y - ptRU[i].y > 5)
		{
			ptRD.insert(ptRD.end(), ptRU.begin() + i, ptRU.end());
			ptRU.erase(ptRU.begin() + i, ptRU.end());
			break;
		}
		//qDebug() << "ptLU x:"<<ptLU[i].x;
	}

}

void GluePos::ransacFitLine(std::vector<cv::Point2d> points, cv::Vec4f &line)
{
	// 使用Ransack拟合直线
	std::vector<cv::Point> temp_points;
	cv::Vec4f temp_line;
	int iner_count_latest = 0;     // 记录最新的
	cv::Vec4f temp_line_latest;    // 记录最新的直线的参数
	for (int i = 0; i < 2000; i++)
	{
		//cout << i << endl;
		int num = points.size();      // 点集个数
		int index_a = rand() % num;   // 产生拟合直线的两个点索引
		int index_b = rand() % num;
		while (index_a == index_b)
		{
			index_b = rand() % num;
		}
		temp_points.clear();
		temp_points.push_back(points[index_a]);
		temp_points.push_back(points[index_b]);
		cv::fitLine(cv::Mat(temp_points), temp_line, CV_DIST_L2, 0, 0.01, 0.01);
		double u = temp_line[0] / temp_line[1];
		double b = temp_line[2] - u * temp_line[3];
		int iner_count = 0;            // 满足这条直线的点的个数
		for (int i = 0; i < points.size(); i++)
		{
			double x0 = points[i].x;
			double y0 = points[i].y;
			double juli = abs(u*y0+b-x0) / sqrt(u*u + 1);
			if (juli < 5)
				iner_count++;
		}
		if (iner_count>iner_count_latest)
		{
			iner_count_latest = iner_count;
			temp_line_latest = temp_line;
		}


	}
	line = temp_line_latest;
}

//direction = 1 :right; direction =2 :left; direction = 3 :up;direciton = 4 :down;
void GluePos::splitDown(std::vector<cv::Point2d> &points,int direction)
{
	std::vector<cv::Point2d> ptUp,ptDown,ptLeft,ptRight;
	long long int avgx = 0;
	long long int avgy = 0;
	for (int i = 0; i < points.size(); i++)
	{
		avgx += points[i].x;
		avgy += points[i].y;
	}
	avgx = avgx / points.size();
	avgy = avgy / points.size();
	for (int i = 0; i < points.size(); i++)
	{
		if (points[i].x > avgx)
			ptUp.push_back(points[i]);
		if (points[i].x < avgx)
			ptDown.push_back(points[i]);
		if (points[i].y > avgy)
			ptRight.push_back(points[i]);
		if (points[i].y < avgy)
			ptLeft.push_back(points[i]);
	
	}
	switch(direction)
	{
		case 1: 
			points = ptUp;
			break;
		case 2:
			points = ptDown;
			break;
		case 3:
			points = ptLeft;
			break;
		case 4:
			points = ptRight;
			break;
		default:
			break;
	}
}

void GluePos::showResult()
{
	getPosEdgePoint();
	cv::Mat src_display;
	cv::cvtColor(GluePosImg2, src_display, CV_GRAY2RGB);
	for (int i = 0; i <ptLU.size(); i++)
	{
		cv::circle(src_display, ptLU[i], 2, cv::Scalar(255, 0, 0), 2, 8, 0);
	}

	for (int i = 0; i <ptRU.size(); i++)
	{
		cv::circle(src_display, ptRU[i], 2, cv::Scalar(255, 0, 0), 2, 8, 0);
	}

	for (int i = 0; i <ptLD.size(); i++)
	{
		cv::circle(src_display, ptLD[i], 2, cv::Scalar(0, 255, 0), 2, 8, 0);
	}

	//splitDown(ptRD, 2);
	//splitDown(ptRD, 3);
	for (int i = 0; i <ptRD.size(); i++)
	{
		cv::circle(src_display, ptRD[i], 2, cv::Scalar(0, 255, 0), 2, 8, 0);
	}

	cv::Vec4f lineLU;
	cv::Vec4f lineLD;
	cv::Vec4f lineRU;
	cv::Vec4f lineRD;
	cv::Point2d pt1, pt2;
	linePara2Pt(ptLU, pt1, pt2, lineLU);
	cv::line(src_display, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8, 0);
	linePara2Pt(ptLD, pt1, pt2, lineLD);
	cv::line(src_display, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8, 0);
	linePara2Pt(ptRU, pt1, pt2, lineRU);
	cv::line(src_display, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8, 0);
	linePara2Pt(ptRD, pt1, pt2, lineRD);
	cv::line(src_display, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8, 0);
	//qDebug() << lineLD[0] << lineLD[1] << lineRD[0] << lineRD[1];
	//qDebug()<< atan2(lineLD[0], lineLD[1]) / 2<< atan2(lineRD[0], lineRD[1]) / 2;
	degreeUp = atan2(lineLU[0]/ lineLU[1], 1) / 2 + atan2(lineRU[0]/ lineRU[1],0 ) / 2;
	degreeDown = atan2(lineLD[0]/ lineLD[1], 1) / 2 + atan2(lineRD[0]/ lineRD[1], 1) / 2;
	cv::namedWindow("class result", 0);
	cv::imshow("class result", src_display);


}

void GluePos::linePara2Pt(std::vector<cv::Point2d> points,cv::Point2d &pt1, cv::Point2d &pt2, cv::Vec4f &line)
{

	ransacFitLine(points, line);
	double u = line[0] / line[1];
	double b = line[2] - u * line[3];
	pt1.y = line[3] - 500;
	pt2.y = line[3] + 500;
	pt2.x = u * pt2.y + b;
	pt1.x = u * pt1.y + b;
	//qDebug() << line[0] << line[1] << line[2] << line[3];
//	qDebug() << pt1.x << pt1.y << pt2.x << pt2.y;
}