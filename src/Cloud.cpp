#include "../include/Cloud.h"

Cloud::Cloud(const char *filename, ofMatrix4x4 *laserToWorld) : filename(filename), laserToWorld(*laserToWorld) {
	fullCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	filteredCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	const size_t length = strlen(filename);
	if (filename[length - 3] == 'p' &&  filename[length - 2] == 'c' &&  filename[length - 1] == 'd')
		pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *fullCloud);
	else
		initTargetVer(filename, fullCloud);
	filterByProbability(fullCloud, filteredCloud, FILTER_PROB);
	cout << "Inserting to mesh..." << endl;
	fullCloudMesh = pclNodesToPoints(fullCloud);
	filteredCloudMesh = pclNodesToPoints(filteredCloud);
	cout << "Done." << endl;
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


void Cloud::addModel(Cultural* cultural) {
	ofVec3f a = cultural->getCenter();
	ofVec3f b = getCloudGlobalCenter();
	float f = a.distance(b);
	ofMatrix4x4 mat = laserToWorld.getInverse();
	if (f < CLOUD_CULTURAL_MAX_DISTANCE) {
		models.push_back(new Object3dModel(cultural->getName(), mat, cultural->getTranslation(), cultural->getRotation(), cultural->getScale()));
	}
}

void Cloud::drawModels() {
	for (vector<Object3dModel*>::iterator model = models.begin(); model != models.end(); model++) {
		(*model)->draw();
	}
}

ofVec3f& Cloud::getCloudGlobalCenter() {
	float x = laserToWorld._mat[0][3];
	float y = laserToWorld._mat[1][3];
	float z = laserToWorld._mat[2][3];
	return ofVec3f(x, y, z);
}