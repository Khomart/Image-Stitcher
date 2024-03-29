// CV.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "pch.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "CV.h"
#include <math.h> 
#include "FDetector.h"
#include "MyStitcher.h"

using namespace cv;
using namespace std;

void multiple();
void multipleMy();
void baseStuff();

struct Connectivity {
	Mat im1, im2, hom, homInv, stitchedImage;
	FDetector detector;
	MyStitcher stitcher;
	vector<DMatch> dmatches;
	vector<array<Point2f, 2>> matches;
	bool valid;
};

int main() {

	//baseStuff();
	multiple();
	//multipleMy();
	return 0;
}

void baseStuff() {
	Mat src, src2;

	src = imread("project_images/Rainier1.png", CV_32F);

	if (src.empty()) {
		cout << "File could not be read" << endl;
		system("pause");
	}
	src2 = imread("project_images/Rainier2.png", CV_32F);

	if (src2.empty()) {
		cout << "File 2 could not be read" << endl;
		system("pause");
	}

	FDetector detector = FDetector();
	vector<DMatch> dmatches;
	vector<array<Point2f, 2>> matches;
	detector.GetMatches(src, src2, matches, dmatches);
	MyStitcher stitcher = MyStitcher();
	Mat hom, homInv, stitchedImage;
	int numM;
	stitcher.RANSAC(matches, dmatches, 100, 5, hom, homInv, src, src2);
	stitcher.stitch(src, src2, hom, homInv, stitchedImage);
	stitchedImage.convertTo(stitchedImage, CV_8U, 255);
	imwrite("4.png", stitchedImage);
}

void multipleMy() {
	const int count = 3;

	Mat images[count];
	images[0] = imread("project_images/dt33.jpg", CV_32F);
	images[1] = imread("project_images/dt32.jpg", CV_32F);
	images[2] = imread("project_images/dt31.jpg", CV_32F);
	Connectivity connectivity[count][count];
	for(int i=0;i< count;i++)
		for (int j = 0; j < count;j++)
		{
			if (i == j) continue;
			if (j < i)
				connectivity[i][j] = connectivity[j][i];
			else {
				connectivity[i][j] = Connectivity();
				connectivity[i][j].im1 = images[i];
				connectivity[i][j].im2 = images[j];
				connectivity[i][j].detector = FDetector();
				connectivity[i][j].detector.GetMatches(connectivity[i][j].im1, connectivity[i][j].im2, connectivity[i][j].matches, connectivity[i][j].dmatches);
				connectivity[i][j].stitcher = MyStitcher();
				connectivity[i][j].stitcher.RANSAC(
					connectivity[i][j].matches,
					connectivity[i][j].dmatches,
					150, 3,
					connectivity[i][j].hom,
					connectivity[i][j].homInv,
					connectivity[i][j].im1,
					connectivity[i][j].im2);
				connectivity[i][j].valid = connectivity[i][j].stitcher.CheckValidity();
			}
			cout << "Check false: " << i << " + " << j << ": " << connectivity[i][j].valid <<"\n";
		}
	vector<int> indexes;
	for (int i = 0;i < count;i++)
		indexes.push_back(i);
	int iterator = 0;
	Mat surf = images[0];
	for (int i = 0;i < count - 1;i++)
	{
		bool connectionExist = true;
		while (connectionExist) {
			int bestMatch = -1;
			for (int j = 0; j < count;j++) {
				if (connectivity[i][j].valid && j > i) {
					if(bestMatch == -1 || (connectivity[i][j].stitcher.inliersCount > connectivity[i][bestMatch].stitcher.inliersCount))
						bestMatch = j;
				}
			}
			if (bestMatch > -1) {
				iterator++;
				FDetector detector = FDetector();
				vector<DMatch> dmatches;
				vector<array<Point2f, 2>> matches;
				detector.GetMatches(surf, connectivity[i][bestMatch].im2, matches, dmatches);
				MyStitcher stitcher = MyStitcher();
				Mat hom, homInv, stitchedImage;
				int numM;
				stitcher.RANSAC(matches, dmatches, 250, 3, hom, homInv, surf, connectivity[i][bestMatch].im2);
				stitcher.stitch(surf, connectivity[i][bestMatch].im2, hom, homInv, surf);
				cout << "Stitching " << i << " + " << bestMatch << "\n";
				for (int k = i;k < count;k++)
					connectivity[k][bestMatch].valid = false;
				//string vova = "result" + iterator;
				//vova = vova + ".png";
				////surf.convertTo(surf, CV_8U, 255);
				//imshow("surf", surf);
				//waitKey();
				//system("pause");
				surf.convertTo(surf, CV_8U, 255);

				imwrite("ResultMy.png", surf); //red channel	
			}
			else
				connectionExist = false;
		}
	}
}

void multiple() {
	const int count = 6;

	Mat images[count];
	images[0] = imread("project_images/Rainier1.png", CV_32F);
	images[1] = imread("project_images/Rainier2.png", CV_32F);
	images[2] = imread("project_images/Rainier3.png", CV_32F);
	images[3] = imread("project_images/Rainier4.png", CV_32F);
	images[4] = imread("project_images/Rainier5.png", CV_32F);
	images[5] = imread("project_images/Rainier6.png", CV_32F);
	Connectivity connectivity[count][count];
	for (int i = 0;i < count;i++)
		for (int j = 0; j < count;j++)
		{
			if (i == j) continue;
			if (j < i)
				connectivity[i][j] = connectivity[j][i];
			else {
				connectivity[i][j] = Connectivity();
				connectivity[i][j].im1 = images[i];
				connectivity[i][j].im2 = images[j];
				connectivity[i][j].detector = FDetector();
				connectivity[i][j].detector.GetMatches(connectivity[i][j].im1, connectivity[i][j].im2, connectivity[i][j].matches, connectivity[i][j].dmatches);
				connectivity[i][j].stitcher = MyStitcher();
				connectivity[i][j].stitcher.RANSAC(
					connectivity[i][j].matches,
					connectivity[i][j].dmatches,
					150, 3,
					connectivity[i][j].hom,
					connectivity[i][j].homInv,
					connectivity[i][j].im1,
					connectivity[i][j].im2);
				connectivity[i][j].valid = connectivity[i][j].stitcher.CheckValidity();
			}
			cout << "Check false: " << i << " + " << j << ": " << connectivity[i][j].valid << "\n";
		}
	vector<int> indexes;
	for (int i = 0;i < count;i++)
		indexes.push_back(i);
	int iterator = 0;
	Mat surf = images[0];
	for (int i = 0;i < count - 1;i++)
	{
		bool connectionExist = true;
		while (connectionExist) {
			int bestMatch = -1;
			for (int j = 0; j < count;j++) {
				if (connectivity[i][j].valid && j > i) {
					if (bestMatch == -1 || (connectivity[i][j].stitcher.inliersCount < connectivity[i][bestMatch].stitcher.inliersCount))
						bestMatch = j;
				}
			}
			if (bestMatch > -1) {
				iterator++;
				FDetector detector = FDetector();
				vector<DMatch> dmatches;
				vector<array<Point2f, 2>> matches;
				detector.GetMatches(surf, connectivity[i][bestMatch].im2, matches, dmatches);
				MyStitcher stitcher = MyStitcher();
				Mat hom, homInv, stitchedImage;
				int numM;
				stitcher.RANSAC(matches, dmatches, 250, 5, hom, homInv, surf, connectivity[i][bestMatch].im2);
				stitcher.stitch(surf, connectivity[i][bestMatch].im2, hom, homInv, surf);
				cout << "Stitching " << i << " + " << bestMatch << "\n";
				for (int k = i;k < count;k++)
					connectivity[k][bestMatch].valid = false;
				//surf.convertTo(surf, CV_8U, 255);
				//imshow("surf", surf);
				//waitKey();
				//system("pause");
				surf.convertTo(surf, CV_8U, 255);


				imwrite("ResultMultiple.png", surf);
			}
			else
				connectionExist = false;
		}
	}
}

