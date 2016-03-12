#pragma once
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
	string getName();

private:
	string name;
	ofxAssimpModelLoader* model;
	ofMatrix4x4 convertMatrix;
	float localX;
	float localY;
	float localZ;
	float qX;
	float qY;
	float qZ; 
	float qW;
	float sX;
	float sY;
	float sZ;
	float sW;
	int r;
	int g;
	int b;
};

