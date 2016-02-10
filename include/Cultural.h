#pragma once

#include <string>

using namespace std;

class Cultural {
public:
	Cultural(string name,
			float translationX,
			float translationY,
			float translationZ,
			float Q1,
			float Q2,
			float Q3,
			float Q4,
			float scaleX,
			float scaleY,
			float scaleZ);
	string& getName();

private:
	string name;
	float translationX;
	float translationY;
	float translationZ;
	float Q1;
	float Q2;
	float Q3;
	float Q4;
	float scaleX;
	float scaleY;
	float scaleZ;
};