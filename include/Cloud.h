#pragma once

#include "ofMain.h"
#include "ofVec3f.h"
#include "ofxAssimpModelLoader\src\ofxAssimpModelLoader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include "Cultural.h"
#include "Object3dModel.h"
#include "PclMethods.h"

#define FILTER_PROB 95

void  initTargetVer(const char* filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
ofMesh* pclNodesToPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr vec);

class Cloud {
public:
	Cloud(const char *filename, ofMatrix4x4 *laserToWorld);
	~Cloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFullCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud();
	ofMesh* getFullCloudMesh();
	ofMesh* getFilteredCloudMesh();
	ofMatrix4x4 getLaserToWorld();
	void addcultural(Cultural *cultural);
	void drawculturals();

private:
	const char *filename;
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
	vector<Cultural*> culturals;
	ofMesh* fullCloudMesh;
	ofMesh* filteredCloudMesh;
	ofMatrix4x4 laserToWorld;
};