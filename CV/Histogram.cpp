#include "pch.h"
#include "Histogram.h"




	Histogram::Histogram(float angle) {
		for (int i = 0;i < 8; i++)
			values[i] = 0;
		offset = angle;
	}
	Histogram::Histogram() {
		for (int i = 0;i < 8; i++)
			values[i] = 0;
		offset = 0;
	}
	float Histogram::getAt(int k) {
		if (k >= 0 && k < 8)
			return values[k];
		else return -1;
	}
	void Histogram::setAt(int k, float val) {
		if (k >= 0 && k < 8)
			values[k] = val;
	}
	void Histogram::AddValue(float angle, float value) {
		//float calcAngle = angle - offset;
		/*if (calcAngle > 360)
			calcAngle -= 360;
		else if (calcAngle < 0)
			calcAngle += 360;*/
		float calcAngle = angle;
		if (calcAngle >= 0 && calcAngle < 45)
			values[0] += value;
		else if (calcAngle >= 45 && calcAngle < 90)
			values[1] += value;
		else if (calcAngle >= 90 && calcAngle < 135)
			values[2] += value;
		else if (calcAngle >= 135 && calcAngle < 180)
			values[3] += value;
		else if (calcAngle >= 180 && calcAngle < 225)
			values[4] += value;
		else if (calcAngle >= 225 && calcAngle < 270)
			values[5] += value;
		else if (calcAngle >= 270 && calcAngle < 315)
			values[6] += value;
		else if (calcAngle >= 315 && calcAngle < 360)
			values[7] += value;
		return;
	}


Histogram::~Histogram()
{
}
