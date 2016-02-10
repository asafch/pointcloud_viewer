#pragma once
#include <vector>

#include <pcl/common/common_headers.h>

#include "ofVec3f.h"
#include "ofxAssimpModelLoader\src\ofxAssimpModelLoader.h"

class Object3dModel {
public:
	Object3dModel::Object3dModel(const char* filename,
		aiMatrix4x4 convertMatrix,
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
	float localX, localY, localZ;
	float qX, qY, qZ, qW;
	float sX, sY, sZ, sW;
	int r, g, b;
};

