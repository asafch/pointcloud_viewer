#pragma once
#include <vector>

#include <pcl/common/common_headers.h>

#include "ofVec3f.h"
#include "ofxAssimpModelLoader\src\ofxAssimpModelLoader.h"

class Object3dModel {
public:
	Object3dModel::Object3dModel(const char* filename,
		ofMatrix4x4 convertMatrix,
		float x,
		float y,
		float z,
		float qx,
		float qy,
		float qz,
		float qw,
		float sx,
		float sy,
		float sz);
	~Object3dModel();
	void draw();

private:
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

