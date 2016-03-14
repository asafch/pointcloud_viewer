#pragma once

/*
All of the class's documentation is within Mappings.cpp
*/

#include <iostream>
#include <string>
#include <unordered_map>

using namespace std;

class Mappings {

public:
	Mappings();
	~Mappings();
	string getCulturalCategory(string &cultural);

private:
	unordered_map<string, string> mappings;

};