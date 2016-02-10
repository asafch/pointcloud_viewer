#include "../include/Object3dModel.h"

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
	float sz) {
	model = new ofxAssimpModelLoader();
	model->setScaleNormalization(false);
	model->loadModel(filename);
	aiVector3D newXYZ = convertMatrix * aiVector3D(x, y, z);
	localX = newXYZ.x;
	localY = newXYZ.y;
	localZ = newXYZ.z;
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
	ofMesh mesh = model->getMesh(0);
}

Object3dModel::~Object3dModel() {

}

void Object3dModel::draw() {
	ofPushMatrix();
	//ofLoadIdentityMatrix();
	ofTranslate(localX, localY, localZ);
	ofQuaternion qaut(qX, qY, qZ, qW);
	ofVec3f qaxis; float qangle;
	qaut.getRotate(qangle, qaxis);
	model->setRotation(0, qangle, qaxis.x, qaxis.y, qaxis.z);
	model->setScale(sX, sY, sZ);
	ofSetColor(0, 200, 200, 156);
	model->drawFaces();
	ofPopMatrix();
}
