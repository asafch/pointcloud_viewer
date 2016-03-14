#include "../include/Cloud.h"
#include "../include/ofApp.h"

/*
filename: the full path to the file
laserToWorld: the transformation the translates the cloud from local space to world space
mappings: an instance of the 'Mappings' class that is a singleton in ofApp
app: ofApp's address
*/
Cloud::Cloud(const char *filename, ofMatrix4x4 *laserToWorld, Mappings *mappings, ofApp *app) : filename(filename), laserToWorld(*laserToWorld) {
	fullCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	filteredCloud = *new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	this->mappings = mappings;
	this->app = app;
	const size_t length = strlen(filename);
	if (filename[length - 3] == 'p' &&  filename[length - 2] == 'c' &&  filename[length - 1] == 'd')
		pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *fullCloud);
	else
		initTargetVer(filename, fullCloud);
	filterByProbability(fullCloud, filteredCloud, FILTER_PROB);
	cout << "Inserting to meshes... ";
	fullCloudMesh = pclNodesToPoints(fullCloud);
	filteredCloudMesh = pclNodesToPoints(filteredCloud);
	cout << "Done." << endl;
}

Cloud::~Cloud() {
	fullCloud->clear();
	filteredCloud->clear();
	delete &fullCloud;
	delete &filteredCloud;
	delete fullCloudMesh;
	delete filteredCloudMesh;
	for (vector<Object3dModel*>::iterator it = models.begin(); it != models.end(); it++) {
		(*it)->~Object3dModel();
	}
	models.clear();
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

/*
After the constructin of the cloud, ofApp will iterate through all culturals and send them to this method.
The cloud is responsible to check if the cultural's center is within a defined radius from the cloud's center.
*/
void Cloud::addModel(Cultural* cultural) {
	ofVec3f a = cultural->getCenter();
	ofVec3f b = getCloudGlobalCenter();
	float f = a.distance(b);
	ofMatrix4x4 mat = laserToWorld.getInverse();
	if (f < CLOUD_CULTURAL_MAX_DISTANCE) {
		models.push_back(new Object3dModel(cultural->getName(), mat, cultural->getTranslation(), cultural->getRotation(), cultural->getScale()));
	}
}

/*
Each model belongs to a certain culutural category, which has a corresponding toggle button in the GUI.
For each model, the cloud queries ofApp whether that model's toggle is turned on, and if so renders the model.
*/
void Cloud::drawModels() {
	for (vector<Object3dModel*>::iterator model = models.begin(); model != models.end(); model++) {
		string name = (*model)->getName();
		string category = mappings->getCulturalCategory(name);
		if (app->isCategorySelected(category))
			(*model)->draw();
	}
}

/*
Get the cloud's center point in global coordinates
*/
ofVec3f Cloud::getCloudGlobalCenter() {
	float x = laserToWorld._mat[0][3];
	float y = laserToWorld._mat[1][3];
	float z = laserToWorld._mat[2][3];
	return ofVec3f(x, y, z);
}

/*
Take the full point cloud and filter out 95% of the points by even distribution.
When rendering the filtered clouds instead of the full clouds, there are less computations, and thus the ssystem is far more responsive to user input.
*/
void Cloud::filterByProbability(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr destination, double threshold) {
	cout << "Filtering by probability above " << threshold << "%" << endl;
	int percent = 10;
	size_t sourceSize = source->points.size();
	size_t tenth = source->points.size() / 10 - 1;
	size_t counter = 0;
	for (size_t i = 0; i < sourceSize; i++) {
		if ((rand() % 100) > threshold)
			destination->points.push_back(source->points[i]);
		if (counter == tenth) {
			counter = 0;
			cout << "\r" << percent << "%";
			percent += 10;
		}
		else
			counter++;
	}
	cout << endl << "Before: " << source->points.size() << ", After: " << destination->points.size() << endl;
}