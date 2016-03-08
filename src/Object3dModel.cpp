#include "../include/Object3dModel.h"

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
	float sz) {
	model = new ofxAssimpModelLoader();
	model->setScaleNormalization(false);
	model->loadModel(filename);
	//this->convertMatrix = convertMatrix;
	this->convertMatrix = convertMatrix.getTransposedOf(convertMatrix);
	//aiVector3D newXYZ = convertMatrix * aiVector3D(x, y, z);
	//localX = newXYZ.x;
	//localY = newXYZ.y;
	//localZ = newXYZ.z;
	localX = x;
	localY = y;
	localZ = z;
	qX = qx;
	qY = qy;
	qZ = qz;
	qW = qw;
	sX = sx;
	sY = sy;
	sZ = sz;
	r = 60 + rand() % 20;
	g = 60 + rand() % 100;
	b = 60 + rand() % 100;
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

