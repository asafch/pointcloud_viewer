#include "../include/Object3dModel.h"

/*
This object describes one model in one cloud.
It holds all of the relevant transformation data, along with the full path to the .stl file which holds the model's 3D data. The 3D model is hold in an instance of ofxAssimpModelLoader.
*/

Object3dModel::Object3dModel(string &filename,
							ofMatrix4x4 convertMatrix,
							ofVec3f translation,
							ofVec4f rotation,
							ofVec3f scale) {
	model = new ofxAssimpModelLoader();
	model->setScaleNormalization(false);
	size_t start = filename.rfind("_");
	size_t end = start;
	start = filename.rfind("_", start - 1) + 1;
	string temp = filename.substr(start, end - start);
	string fullPath = PATH_PREFIX + temp + PATH_SUFFIX;
	this->name = extractCulturalTypeFromFilename(fullPath);
	model->loadModel(fullPath);
	this->convertMatrix = convertMatrix.getTransposedOf(convertMatrix);
	translationX = translation[0];
	translationY = translation[1];
	translationZ = translation[2];
	Q1 = rotation[0];
	Q2 = rotation[1];
	Q3 = rotation[2];
	Q4 = rotation[3];
	scaleX = scale[0];
	scaleY = scale[1];
	scaleZ = scale[2];
}

Object3dModel::~Object3dModel() {
	delete model;
}

void Object3dModel::draw() {
	ofPushMatrix();
	ofQuaternion qaut(Q1, Q2, Q3, Q4);
	ofVec3f qaxis;
	float qangle;
	qaut.getRotate(qangle, qaxis);
	model->setPosition(translationX, translationY, translationZ);
	model->setRotation(0, qangle, qaxis.x, qaxis.y, qaxis.z);
	model->setScale(scaleX, scaleY, scaleZ);
	ofSetColor(0, 200, 200, 156);
	model->updateMatrix(convertMatrix);
	model->drawFaces();
	ofPopMatrix();
}

string Object3dModel::getFilename() {
	return filename;
}

string Object3dModel::getName() {
	return name;
}


string Object3dModel::extractCulturalTypeFromFilename(const string& filename) {
	size_t dot = filename.rfind(".");
	string result = filename.substr(0, dot);
	size_t lastSlash = result.rfind("\\");
	result = result.substr(lastSlash + 1, result.length() - lastSlash);
	size_t junk = result.rfind("-");
	if (junk != string::npos) {
		result = result.substr(0, junk);
		return result;
	}
	junk = result.rfind("_");
	if (junk != string::npos) {
		result = result.substr(0, junk);
		return result;
	}
	return result;
}
