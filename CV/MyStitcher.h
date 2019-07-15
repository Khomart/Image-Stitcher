#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace cv;
using namespace std;
class MyStitcher
{
private:
	int  totalPointsNum;
public:
	int inliersCount;
	MyStitcher();
	~MyStitcher();
	void project(int x1, int y1, Mat H, int& x2, int& y2);
	void computeInlierCount(Mat H, vector<array<Point2f, 2>> matches, int& numMatches, float inlierThreshold);
	void RANSAC(vector<array<Point2f, 2>> matches, vector<DMatch> numMatches,int numIterations, float inlierThreshold, Mat& hom, Mat& homInv, Mat image1Display, Mat  image2Display);
	void stitch(Mat image1, Mat image2, Mat hom, Mat  homInv, Mat& stitchedImage);
	bool CheckValidity();
};

