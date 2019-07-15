#include "pch.h"
#include "MyStitcher.h"


MyStitcher::MyStitcher()
{
}


MyStitcher::~MyStitcher()
{
}

void MyStitcher::project(int x1, int y1, Mat H, int& x2, int& y2) {
	float xP, yP;
	float test = H.at<double>(1, 1);
	xP = (H.at<double>(0, 0) * x1 + H.at<double>(0, 1) * y1 + H.at<double>(0, 2)) / (H.at<double>(2, 0) * x1 + H.at<double>(2, 1) * y1 + H.at<double>(2, 2));
	yP = (H.at<double>(1, 0) * x1 + H.at<double>(1, 1) * y1 + H.at<double>(1, 2)) / (H.at<double>(2, 0) * x1 + H.at<double>(2, 1) * y1 + H.at<double>(2, 2));

	int xT, yT;
	vector<Point2f> org, des;
	Point2f point = Point2f(x1, y1);
	org.push_back(point);
	perspectiveTransform(org, des,H);
	
	x2 = (int)xP;
	y2 = (int)yP;
}

void MyStitcher::computeInlierCount(Mat H, vector<array<Point2f, 2>> matches, int & numMatches, float inlierThreshold)
{
	int count = 0;
	for (int matchID = 0; matchID < matches.size();matchID++) {
		int projX, projY, orgX, orgY, realX, realY;
		orgX = (int)matches[matchID][0].x;
		orgY = (int)matches[matchID][0].y;
		realX = (int)matches[matchID][1].x;
		realY = (int)matches[matchID][1].y;
		project(orgX, orgY, H, projX, projY);
		float diff1 = (projX - realX);
		float diff2 = (projY - realY);
		float dist = sqrt(diff1*diff1 + diff2*diff2);
		if (dist < inlierThreshold)
			count++;
	}
	numMatches = count;
}

void MyStitcher::RANSAC(vector<array<Point2f,2>> matches, vector<DMatch> numMatches, int numIterations, float inlierThreshold, Mat& hom, Mat& homInv, Mat image1Display, Mat image2Display)
{
	if (matches.size() < 20) {
		totalPointsNum = matches.size();
		return;
	}
	Mat bestHom;
	int maxInl = -1;
	
	for (int iteration = 0; iteration < numIterations; iteration++) {
		int m[4];int inl;
		for (int i = 0;i < 4;i++) {
			m[i] = rand() % matches.size();
		}
		bool allDiff = false;
		while (!allDiff) {
			allDiff = true;
			if (m[0] == m[1] || m[0] == m[2] || m[0] == m[3]) {
				m[0] = rand() % matches.size();
				allDiff = false;
			}
			if (m[1] == m[0] || m[1] == m[2] || m[1] == m[3]) {
				m[1] = rand() % matches.size();
				allDiff = false;
			}
			if (m[2] == m[1] || m[2] == m[0] || m[2] == m[3]) {
				m[2] = rand() % matches.size();
				allDiff = false;
			}
			if (m[3] == m[1] || m[3] == m[0] || m[3] == m[2]) {
				m[3] = rand() % matches.size();
				allDiff = false;
			}
		}

		vector<Point2f> orgMatches = vector<Point2f>(), expMatches = vector<Point2f>();
		for (int i = 0;i < 4;i++) {
			orgMatches.push_back(matches[m[i]][0]);
			expMatches.push_back(matches[m[i]][1]);
		}
		Mat H = Mat(3,3, CV_16UC3);
		H = findHomography(orgMatches, expMatches, 0);
		if (H.dims > 0) {
			computeInlierCount(H, matches, inl, inlierThreshold);
			if (inl > maxInl) {
				bestHom = H;
				maxInl = inl;
			}
		}
	}
	vector<KeyPoint> inliersKP, inliersExpKP;
	vector<Point2f> inliers, inliersExp;
	vector<DMatch> matchings;
	for (int matchID = 0; matchID < matches.size(); matchID++) {
		int projX, projY, orgX, orgY, realX, realY;
		orgX = (int)matches[matchID][0].x;
		orgY = (int)matches[matchID][0].y;
		realX = (int)matches[matchID][1].x;
		realY = (int)matches[matchID][1].y;
		project(orgX, orgY, bestHom, projX, projY);
		
		float dist = (float)sqrt(pow((projX - realX), 2) + pow((projY - realY), 2));
		if (dist < inlierThreshold) {
			matchings.push_back( DMatch((int)(matchings.size()), (int)(matchings.size()), numMatches[matchID].distance) );
			inliers.push_back(Point2f(orgX, orgY));
			inliersKP.push_back(KeyPoint(orgX, orgY, 1));
			inliersExp.push_back(Point2f(realX, realY));
			inliersExpKP.push_back(KeyPoint(realX, realY, 1));
		}
	}
	this->inliersCount = inliers.size();
	this->totalPointsNum = matches.size();
	hom = findHomography(inliers, inliersExp, 0);
	homInv = hom.inv();
	/*hom = bestHom;
	homInv = hom.inv();*/

	Mat result, src_gray, src_gray2;
	//cvtColor(image1Display, src_gray, COLOR_BGR2GRAY);
	//cvtColor(image2Display, src_gray2, COLOR_BGR2GRAY);
	drawMatches(image1Display, inliersKP, image2Display, inliersExpKP, matchings, result);
	imwrite("3.png", result);

	//Mat write;
	//result.convertTo(write, CV_8U, 255);
	//imwrite("3.png", write);

	return;
}

void MyStitcher::stitch(Mat image1, Mat image2, Mat hom, Mat homInv, Mat & stitchedImage)
{
	Point2f points[4];
	int x, y;
	project(0, 0, homInv, x, y);
	points[0] = Point2f(x, y);
	project(image2.cols, 0, homInv, x, y);
	points[1] = Point2f(x, y);
	project(image2.cols, image2.rows, homInv, x, y);
	points[2] = Point2f(x, y);
	project(0, image2.rows, homInv, x, y);
	points[3] = Point2f(x, y);

	int beforeX = 0, afterX = image1.cols, beforeY = 0, afterY = image1.rows;
	if (points[0].x < 0)
		beforeX = points[0].x;
	if (points[3].x < beforeX)
		beforeX = points[3].x;

	if (points[1].x > afterX)
		afterX = points[1].x;
	if (points[2].x > afterX)
		afterX = points[2].x;
	afterX = afterX - image1.cols;

	if (points[0].y < 0)
		beforeY = points[0].y;
	if (points[1].y < beforeY)
		beforeY = points[1].y;

	if (points[2].y > afterY)
		afterY = points[2].y;
	if (points[3].y > afterY)
		afterY = points[3].y;
	afterY = afterY - image1.rows;

	int len = -beforeX + image1.cols + afterX;
	int height = -beforeY + image1.rows + afterY;


	Mat temp = Mat::zeros(height, len, CV_32FC3);
	image1.convertTo(image1, CV_32FC3, 1.0 / 255.0);
	image2.convertTo(image2, CV_32FC3, 1.0 / 255.0);


	for (int rowI1 = 0; rowI1 < image1.rows; rowI1++)
		for (int colI1 = 0; colI1 < image1.cols; colI1++) {
			temp.at<Vec3f>(-beforeY + rowI1, -beforeX + colI1)[0] = image1.at<Vec3f>(rowI1, colI1)[0];
			temp.at<Vec3f>(-beforeY + rowI1, -beforeX + colI1)[1] = image1.at<Vec3f>(rowI1, colI1)[1];
			temp.at<Vec3f>(-beforeY + rowI1, -beforeX + colI1)[2] = image1.at<Vec3f>(rowI1, colI1)[2];
			//temp.at<float>(-beforeY + rowI1, -beforeX + colI1) = image1.at<float>(rowI1, colI1);
		}
	//imshow("res", temp);
	//waitKey();
	//system("pause");

	float distIm2 = sqrt((image2.cols / 2.0f) * (image2.cols / 2.0f) + (image2.rows / 2.0f) * (image2.rows / 2.0f));
	//for (int rowSt = 0;rowSt < temp.rows + beforeY;rowSt++)
	//	for (int colSt = 0;colSt < temp.cols + beforeX;colSt++) {
	for (int rowSt = beforeY;rowSt < image1.rows+afterY;rowSt++)
		for (int colSt = beforeX;colSt < image1.cols+afterX;colSt++) {
			int xst, yst;
			project(colSt, rowSt, hom, xst, yst);
			if (xst >= 0 && xst < image2.cols && yst >= 0 && yst < image2.rows) {
				Mat arrr;
				getRectSubPix(image2, Size(3, 3), Point2f(xst, yst), arrr);
				if (temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX) == Vec3f(0, 0, 0)) {
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[0] = arrr.at<Vec3f>(1, 1)[0];
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[1] = arrr.at<Vec3f>(1, 1)[1];
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[2] = arrr.at<Vec3f>(1, 1)[2];
				}
				else {
					float alfa = sqrt((xst - (image2.cols / 2.0f)) * (xst - (image2.cols / 2.0f)) + (yst - (image2.rows / 2.0f)) * (yst - (image2.rows / 2.0f)));
					alfa = alfa / distIm2;
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[0] = (arrr.at<Vec3f>(1, 1)[0] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[0]);
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[1] = (arrr.at<Vec3f>(1, 1)[1] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[1]);
					temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[2] = (arrr.at<Vec3f>(1, 1)[2] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt - beforeY, colSt - beforeX)[2]);
				}

				//if (temp.at<Vec3f>(rowSt, colSt) == Vec3f(0, 0, 0)) {
				//	temp.at<Vec3f>(rowSt, colSt)[0] = arrr.at<Vec3f>(0, 0)[0];
				//	temp.at<Vec3f>(rowSt, colSt)[1] = arrr.at<Vec3f>(0, 0)[1];
				//	temp.at<Vec3f>(rowSt, colSt)[2] = arrr.at<Vec3f>(0, 0)[2];
				//}
				//else {
				//	float alfa = sqrt((xst - (image2.cols / 2.0f)) * (xst - (image2.cols / 2.0f)) + (yst - (image2.rows / 2.0f)) * (yst - (image2.rows / 2.0f)));
				//	alfa = alfa / distIm2;
				//	temp.at<Vec3f>(rowSt, colSt)[0] = arrr.at<Vec3f>(0, 0)[0] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt, colSt)[0];
				//	temp.at<Vec3f>(rowSt, colSt)[1] = arrr.at<Vec3f>(0, 0)[1] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt, colSt)[0];
				//	temp.at<Vec3f>(rowSt, colSt)[2] = arrr.at<Vec3f>(0, 0)[2] * alfa + (1 - alfa)*temp.at<Vec3f>(rowSt, colSt)[0];
				//}
			}
			//temp.at<float>(-beforeY + rowI1, -beforeX + colI1) = image1.at<float>(rowI1, colI1);
		}
	//imshow("res", temp);
	//waitKey();
	//system("pause");
	/*temp.convertTo(temp, CV_8U, 255);

	imwrite("4.png", temp);	*/

	//temp.convertTo(temp, CV_32FC3, 255);

	//imwrite("result2.png", temp); //red channel	
	stitchedImage = temp;
}



bool MyStitcher::CheckValidity() {
	if (totalPointsNum < 20) return false;
	return (this->inliersCount > 8 + 0.3f*this->totalPointsNum);
}