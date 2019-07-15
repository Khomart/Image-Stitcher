#include "pch.h"
#include "FDetector.h"


FDetector::FDetector()
{
}
vector<KeyPoint> FDetector::GetKeyPoints(Mat src) {
	//imshow("src", src);
	//waitKey();
	//system("pause");
	cout << "InputType: " << src.depth() << "\n";
	Mat src_gray;
	cvtColor(src, src_gray, COLOR_BGR2GRAY);
	//imshow("src_gray", src_gray);
	//waitKey();
	//system("pause");
	src_gray.convertTo(src_gray, CV_32FC1, 1.0 / 255.0);
	cout << "AfterGrayph: : " << src_gray.depth() << "\n";
	Mat dx, dy, dxdy;
	dxdy = Mat::zeros(src_gray.rows, src_gray.cols, CV_32F);
	Sobel(src_gray, dx, -1, 1, 0, 3);
	Sobel(src_gray, dy, -1, 0, 1, 3);
	Mat dx2, dy2;
	pow(dx, 2, dx2);
	pow(dy, 2, dy2);
	for (int i = 0; i < dx.rows; i++) {
		for (int j = 0; j < dx.cols; j++) {
			float temp = dx.at<float>(i, j) * dy.at<float>(i, j);
			dxdy.at<float>(i, j) = temp;
		}
	}

	Mat result = Mat::zeros(src_gray.rows, src_gray.cols, CV_32F);

	float tr = (float)3e-8; //threshhold is picked experimentally.

	for (int i = 0; i < src_gray.rows; i++) {
		for (int j = 0; j < src_gray.cols; j++) {

			float det = dx2.at<float>(i, j) * dy2.at<float>(i, j) - dxdy.at<float>(i, j) * dxdy.at<float>(i, j);
			float trace = 0;
			float response = 0;

			if (det != 0) {
				trace = dx2.at<float>(i, j) + dy2.at<float>(i, j);
				if (trace != 0) {
					response = det / trace; // this function works better then other.
					//response = det - 0.05*pow(trace, 2);
				}
			}
			if (response > tr) {
				result.at<float>(i, j) = src_gray.at<float>(i, j);
			}
		}
	}

	int border = 1;
	Mat buffer(result.rows + border * 2, result.cols + border * 2, result.depth());
	copyMakeBorder(result, buffer, border, border, border, border, BORDER_CONSTANT);

	for (int i = 1; i < buffer.rows - 1; i++) {
		for (int j = 1; j < buffer.cols - 1; j++) {
			if (buffer.at<float>(i, j) > buffer.at<float>(i - 1, j - 1)) {
				buffer.at<float>(i - 1, j - 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i - 1, j + 1)) {
				buffer.at<float>(i - 1, j + 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i, j - 1)) {
				buffer.at<float>(i, j - 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i, j - 1)) {
				buffer.at<float>(i, j - 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i, j + 1)) {
				buffer.at<float>(i, j + 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i + 1, j - 1)) {
				buffer.at<float>(i + 1, j - 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i + 1, j)) {
				buffer.at<float>(i + 1, j) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
			if (buffer.at<float>(i, j) > buffer.at<float>(i + 1, j + 1)) {
				buffer.at<float>(i + 1, j + 1) = 0;
			}
			else {
				buffer.at<float>(i, j) = 0;
				break;
			}
		}
	}


	vector<KeyPoint> points;
	for (int i = 0; i < src_gray.rows; i++) {
		for (int j = 0; j < src_gray.cols; j++) {
			if (buffer.at<float>(i + 1, j + 1) > 0) {
				if (i >= 15 && j >= 15 && i <= (src_gray.rows - 15) && j <= (src_gray.cols - 15))
					points.push_back(KeyPoint((float)j, (float)i, 1));
			}
		}
	}
	cvtColor(src, src_gray, COLOR_BGR2GRAY);

	//imshow("out1", out);
	//waitKey();
	//system("pause");
	return points;

}

vector<vector<DMatch>> FDetector::SSD(vector<Feature> f1, vector<Feature> f2) {
	vector<float> sums;
	vector<DMatch> dMatches;
	vector<vector<DMatch>> vectors;
	for (int i = 0; i < f1.size(); i++) {
		dMatches.clear();
		for (int j = 0; j < f2.size(); j++) {
			dMatches.push_back(DMatch(i, j, f1[i].dist(f2[j])));
		}
		vectors.push_back(dMatches);
	}
	return vectors;
}

bool FDetector::compareMatches(DMatch first, DMatch second) {
	return first.distance < second.distance;
}

FDetector::~FDetector()
{
}

void FDetector::GetMatches(Mat src, Mat src2, vector<array<Point2f, 2>>& matchingPoints, vector<DMatch>& dmatches) {
	Mat src_gray, src_gray2;
	cvtColor(src, src_gray, COLOR_BGR2GRAY);
	cvtColor(src2, src_gray2, COLOR_BGR2GRAY);
	src_gray.convertTo(src_gray, CV_32FC1, 1.0 / 255.0);
	src_gray2.convertTo(src_gray2, CV_32FC1, 1.0 / 255.0);

	Mat keypShow;
	vector<KeyPoint> points1 = GetKeyPoints(src);
	//drawKeypoints(src, points1, keypShow);
	//imwrite("1b.png", keypShow);

	vector<KeyPoint> points2 = GetKeyPoints(src2);
	//drawKeypoints(src2, points2, keypShow);
	//imwrite("1c.png", keypShow);

	vector<Feature> features1, features2;

	for (int i = 0; i < points1.size(); i++) {
		features1.push_back(Feature());
		features1[i].Init(src_gray, (int)points1[i].pt.x, (int)points1[i].pt.y);
	}
	for (int i = 0; i < points2.size(); i++) {
		features2.push_back(Feature());
		features2[i].Init(src_gray2, (int)points2[i].pt.x, (int)points2[i].pt.y);
	}
	vector< vector<DMatch>> dMatches = SSD(features1, features2);
	vector<DMatch> finalSSD, finalRatio;
	for (int i = 0; i < dMatches.size(); i++) {
		DMatch shortest = dMatches[i][0];
		DMatch secondShortest = dMatches[i][0];
		for (int j = 1; j < dMatches[i].size(); j++) {
			if (dMatches[i][j].distance < shortest.distance) {
				secondShortest = shortest;
				shortest = dMatches[i][j];
			}
		}
		if (shortest.distance <= 700) {
			float dist = shortest.distance / secondShortest.distance;
			if(dist<0.7)
				finalSSD.push_back(shortest);
		}
	}
	Mat result;

	cvtColor(src, src_gray, COLOR_BGR2GRAY);
	cvtColor(src2, src_gray2, COLOR_BGR2GRAY);

	for (int i = 0; i < dMatches.size(); i++) {
		sort(dMatches[i].begin(), dMatches[i].end());
	}

	float matching = 0;
	for (int i = 0; i < dMatches.size(); i++) {
		float dist = dMatches[i][0].distance / dMatches[i][1].distance;
		matching += dist;
	}
	matching = matching / dMatches.size();
	//cout << "the average ambigiousity is " << matching << "\n";

	//imshow("SSD", result);
	//waitKey();
	//system("pause");

	vector<array<Point2f, 2>> output;
	for (int i = 0; i < finalSSD.size(); i++) {
		array<Point2f, 2> temp;
		temp[0] = points1[finalSSD[i].queryIdx].pt;
		temp[1] = points2[finalSSD[i].trainIdx].pt;
		output.push_back(temp);
	}
	drawMatches(src_gray, points1, src_gray2, points2, finalSSD, result);
	//imwrite("2.png", result); //red channel	
	matchingPoints = output;
	dmatches = finalSSD;
	return;
}