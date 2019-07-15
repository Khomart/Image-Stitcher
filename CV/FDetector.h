#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Feature.h" 
using namespace cv;
using namespace std;
class FDetector
{
public:
	FDetector();
	vector<KeyPoint> GetKeyPoints(Mat src);
	vector<vector<DMatch>> SSD(vector<Feature> f1, vector<Feature> f2);
	bool compareMatches(DMatch first, DMatch second);
	~FDetector();
	void GetMatches(Mat src, Mat src2, vector<array<Point2f, 2>>& matchingPoints, vector<DMatch>& dmatches);
};

