#include "pch.h"
#include "Feature.h"

#define PI 3.14159265



	

	Feature::Feature() {
	}
	float Feature::dist(Feature feature2) {
		float val = 0;
		for (int i = 0;i < 16 * 8;i++) {
			val += pow((res[i] - feature2.res[i]), 2);
		}
		return val;
	}

	float * Feature::Init(Mat img, int iCord, int jCord)
	{
		src = img;
		kpI = iCord;
		kpJ = jCord;
		/*kpI = jCord;
		kpJ = iCord;*/
		float angle = CalcRot(iCord, jCord, src);

		for (int i = 0;i < 4;i++)
			for (int j = 0;j < 4;j++) {
				histograms[i][j] = Histogram(angle);
			}



		//Mat gx, gy;
		//Sobel(img, gx, -1, 1, 0, 3);
		//Sobel(img, gy, -1, 0, 1, 3);
		//cartToPolar(gx, gy, mag, anglde, 1);

		CalcHists();


		for (int i = 0;i < 4;i++)
			for (int j = 0;j < 4;j++) {
				res[i * 32 + j * 8] = histograms[i][j].getAt(0);
				res[i * 32 + j * 8 + 1] = histograms[i][j].getAt(1);
				res[i * 32 + j * 8 + 2] = histograms[i][j].getAt(2);
				res[i * 32 + j * 8 + 3] = histograms[i][j].getAt(3);
				res[i * 32 + j * 8 + 4] = histograms[i][j].getAt(4);
				res[i * 32 + j * 8 + 5] = histograms[i][j].getAt(5);
				res[i * 32 + j * 8 + 6] = histograms[i][j].getAt(6);
				res[i * 32 + j * 8 + 7] = histograms[i][j].getAt(7);
			}
		ClapHists();
		RestorHists();
		return res;
	}
	void Feature::ClapHists() {
		float temp = 0;
		for (int i = 0; i < 16 * 8; i++) {
			temp += pow(res[i], 2);
		}
		temp = sqrt(temp);
		for (int i = 0; i < 16 * 8; i++) {
			res[i] = (res[i]) / temp;
		}
		for (int i = 0; i < 16 * 8; i++) {
			if (res[i] > 0.2)
				res[i] = 0.2f;
		}
	}
	void Feature::RestorHists() {
		for (int i = 0; i < 16 * 8; i++) {
			res[i] = res[i] * 100;
		}
	}
	void Feature::CalcHists() {
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {
				CalcSingle(i, j);
			}
	}
	void Feature::CalcSingle(int indI, int indJ) {
		float angle, value;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {
				angle = CalcRot(kpI - 7 + i + indI * 4, kpJ - 7 + j + indJ * 4, src);
				value = CalcMag(kpI - 7 + i + indI * 4, kpJ - 7 + j + indJ * 4, src);
				//angle = anglde.at<float>(kpI - 7 + i + indI * 4, kpJ - 7 + j + indI * 4);
				//value = mag.at<float>(kpI - 7 + i + indI * 4, kpJ - 7 + j + indI * 4);
				histograms[indI][indJ].AddValue(angle, value);
			}

	}
	float Feature::CalcMag(int y, int x, Mat img)
	{
		float t1 = pow(img.at<float>(x - 1, y) - img.at<float>(x + 1, y), 2);
		float t2 = pow(img.at<float>(x, y - 1) - img.at<float>(x, y + 1), 2);
		return sqrt(t1 + t2);
	}
	float Feature::CalcRot(int y, int x, Mat img)
	{
		float val1, val2, val3, val4;
		val1 = img.at<float>(x, y - 1);
		val2 = img.at<float>(x, y + 1);
		val3 = img.at<float>(x - 1, y);
		val4 = img.at<float>(x + 1, y);
		float t1 = pow(img.at<float>(x, y - 1) - img.at<float>(x, y + 1), 2);
		float t2 = pow(img.at<float>(x - 1, y) - img.at<float>(x + 1, y), 2);
		float rad = atan2(t1, t2);
		return (float)(rad * 180.f / PI);
	}



Feature::~Feature()
{
}
