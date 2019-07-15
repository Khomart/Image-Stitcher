#pragma once
#include <opencv2/opencv.hpp>
#include "Histogram.h"

using namespace cv;
class Feature
{
public:
	// Data Members 
	Mat src;
	int kpI, kpJ;
	Histogram histograms[4][4];
	float res[16 * 8];
	Mat mag, anglde;
	Feature();
	float dist(Feature feature2);
	float * Init(Mat img, int iCord, int jCord);
	void ClapHists();
	void RestorHists();
	void CalcHists();
	void CalcSingle(int indI, int indJ);
	float CalcMag(int y, int x, Mat img);
	float CalcRot(int y, int x, Mat img);
	~Feature();
};

