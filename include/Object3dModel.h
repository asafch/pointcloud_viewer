#pragma once

/*
All of the class's documentation is within Object3dModel.cpp
*/

#include <string>
#include <vector>

#include <pcl/common/common_headers.h>

#include "ofVec3f.h"
#include "ofxAssimpModelLoader\src\ofxAssimpModelLoader.h"

#define PATH_PREFIX "C:\\scans\\culturals\\"
#define PATH_SUFFIX ".stl"

class Object3dModel {

public:
	Object3dModel::Object3dModel(string &filename,
		ofMatrix4x4 convertMatrix,
		ofVec3f translation,
		ofVec4f rotation,
		ofVec3f scale);
	~Object3dModel();
	void draw();
	string getFilename();
	string getName();
	string extractCulturalTypeFromFilename(const string& filename);

private:
	string filename;
	string name;
	ofxAssimpModelLoader* model;
	ofMatrix4x4 convertMatrix;
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

