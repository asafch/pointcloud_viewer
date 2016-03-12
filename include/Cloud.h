#pragma once

#include "ofMain.h"
#include "ofVec3f.h"
#include "ofxAssimpModelLoader\src\ofxAssimpModelLoader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include "Cultural.h"
#include "Mappings.h"
#include "Object3dModel.h"
#include "PclMethods.h"

#define FILTER_PROB 95
#define CLOUD_CULTURAL_MAX_DISTANCE 65.0

void  initTargetVer(const char* filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
ofMesh* pclNodesToPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr vec);

class Cloud {
public:
	Cloud(const char *filename, ofMatrix4x4 *laserToWorld, Mappings *mappings, unordered_map<string, bool> *selectedCulturalCategories);
	~Cloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFullCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud();
	ofMesh* getFullCloudMesh();
	ofMesh* getFilteredCloudMesh();
	ofMatrix4x4 getLaserToWorld();
	void addModel(Cultural *cultural);
	void drawModels();
	ofVec3f& getCloudGlobalCenter();

private:
	const char *filename;
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
	unordered_map<string, bool> *selectedCulturalCategories;
	vector<Object3dModel*> models;
	ofMesh* fullCloudMesh;
	ofMesh* filteredCloudMesh;
	ofMatrix4x4 laserToWorld;
	Mappings *mappings;
};