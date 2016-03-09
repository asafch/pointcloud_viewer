#include "../include/Object3dModel.h"

Object3dModel::Object3dModel(string filename,
							ofMatrix4x4 convertMatrix,
							ofVec3f translation,
							ofVec4f rotation,
							ofVec3f scale) {
	model = new ofxAssimpModelLoader();
	model->setScaleNormalization(false);
	size_t start = filename.rfind("_");
	size_t end = start;
	start = filename.rfind("_", start - 1) + 1;
	string ok = PATH_PREFIX + filename.substr(start, end - start) + PATH_SUFFIX;
	model->loadModel(ok);
	//this->convertMatrix = convertMatrix;
	this->convertMatrix = convertMatrix.getTransposedOf(convertMatrix);
	//aiVector3D newXYZ = convertMatrix * aiVector3D(x, y, z);
	//localX = newXYZ.x;
	//localY = newXYZ.y;
	//localZ = newXYZ.z;
	localX = translation[0];
	localY = translation[1];
	localZ = translation[2];
	qX = rotation[0];
	qY = rotation[1];
	qZ = rotation[2];
	qW = rotation[3];
	sX = scale[0];
	sY = scale[1];
	sZ = scale[2];
	//r = 60 + rand() % 20;
	//g = 60 + rand() % 100;
	//b = 60 + rand() % 100;
	//ofMesh mesh = model->getMesh(0);
}

Object3dModel::~Object3dModel() {

}

void Object3dModel::draw() {
	ofPushMatrix();
	//ofLoadIdentityMatrix();
	ofQuaternion qaut(qX, qY, qZ, qW);
	ofVec3f qaxis;
	float qangle;
	qaut.getRotate(qangle, qaxis);
	model->setPosition(localX, localY, localZ);
	model->setRotation(0, qangle, qaxis.x, qaxis.y, qaxis.z);
	model->setScale(sX, sY, sZ);
	ofSetColor(0, 200, 200, 156);
	model->updateMatrix(convertMatrix);
	model->drawFaces();
	ofPopMatrix();
}

