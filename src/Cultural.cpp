#include "../include/Cultural.h"

/*
No special documentation required.
*/

Cultural::Cultural(string name,
	float translationX,
	float translationY,
	float translationZ,
	float Q1,
	float Q2,
	float Q3,
	float Q4,
	float scaleX,
	float scaleY,
	float scaleZ) : name(name){
	
	this->name = name;
	this->translationX = translationX;
	this->translationY = translationY;
	this->translationZ = translationZ;
	this->Q1 = Q1;
	this->Q2 = Q2;
	this->Q3 = Q3;
	this->Q4 = Q4;
	this->scaleX = scaleX;
	this->scaleY = scaleY;
	this->scaleZ = scaleZ;
}

Cultural::~Cultural() {

}

string Cultural::getName() {
	return name;
}

ofVec3f Cultural::getCenter() {
	return ofVec3f(translationX, translationY, translationZ);
}

ofVec3f Cultural::getTranslation() {
	return ofVec3f(translationX, translationY, translationZ);
}

ofVec4f Cultural::getRotation() {
	return ofVec4f(Q1, Q2, Q3, Q4);
}

ofVec3f Cultural::getScale() {
	return ofVec3f(scaleX, scaleY, scaleZ);
}
