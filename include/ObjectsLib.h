#pragma once

#include <vector>

#include "ofxAssimpModelLoader.h"
#include "ofVec3f.h"

#include "Cloud.h"
#include "Object3dModel.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/extract_indices.h>

class ObjectsLib {
public:
	ObjectsLib::ObjectsLib(ofVec3f transCam, ofVec3f rotateCam, ofVec3f transRoute, ofVec3f rotateRoute);
	~ObjectsLib(void);
	void draw(void);
	void addModel(const char* filename, float x, float y, float z, float qx, float qy, float qz, float qw, float sX, float sY, float sZ);
	void loadModels();

private:
	aiMatrix4x4 convertMatrix;
	vector<Object3dModel*> models;
	float cameraLocationX;
	float cameraLocationY;
	float cameraLocationZ;
};

