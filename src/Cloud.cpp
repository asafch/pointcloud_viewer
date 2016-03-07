#include "../include/Cloud.h"

Cloud::Cloud(const char *filename, ofMatrix4x4 *laserToWorld) : filename(filename), laserToWorld(*laserToWorld) {
	fullCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	filteredCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	const size_t length = strlen(filename);
	if (filename[length - 3] == 'p' &&  filename[length - 2] == 'c' &&  filename[length - 1] == 'd')
		pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *fullCloud);
	else
		initTargetVer(filename, fullCloud);
	filterByProbality(fullCloud, filteredCloud, FILTER_PROB);
	printf("Inserting to mesh...\n");
	fullCloudMesh = pclNodesToPoints(fullCloud);
	filteredCloudMesh = pclNodesToPoints(filteredCloud);
	printf("Done.\n");
}

Cloud::~Cloud() {
	fullCloud->clear();
	filteredCloud->clear();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud::getFullCloud() {
	return fullCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud::getFilteredCloud() {
	return filteredCloud;
}

ofMesh* Cloud::getFullCloudMesh() {
	return fullCloudMesh;
}

ofMesh* Cloud::getFilteredCloudMesh() {
	return filteredCloudMesh;
}

ofMatrix4x4 Cloud::getLaserToWorld() {
	return laserToWorld;
}