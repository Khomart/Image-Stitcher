#pragma once
class Histogram
{
public:
	float values[8];
	float offset;
	Histogram(float angle);
	Histogram();
	float getAt(int k);
	void setAt(int k, float val);
	void AddValue(float angle, float value);
	~Histogram();
};

