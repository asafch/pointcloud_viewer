#include "../include/Cultural.h"

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

string& Cultural::getName() {
	return name;
}