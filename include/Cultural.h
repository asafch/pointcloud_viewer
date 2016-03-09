#pragma once

#include <string>

#include "ofVec3f.h"

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
	ofVec3f& getCenter();
	ofVec3f& getTranslation();
	ofVec4f& getRotation();
	ofVec3f& getScale();

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