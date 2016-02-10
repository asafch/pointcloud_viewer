#pragma once

#include "ofMain.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include "PclMethods.h"

#define FILTER_PROB 95

void  initTargetVer(const char* filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
ofMesh* pclNodesToPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr vec);

class Cloud {
public:
	Cloud(const char *filename);
	~Cloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFullCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud();
	ofMesh* getFullCloudMesh();
	ofMesh* getFilteredCloudMesh();

private:
	const char *filename;
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
	ofMesh* fullCloudMesh;
	ofMesh* filteredCloudMesh;
};