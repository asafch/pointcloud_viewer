#pragma once

#include <string>
#include <unordered_map>

using namespace std;

class Mappings {

public:
	Mappings();
	~Mappings();
	string getCulturalCategory(const string &cultural);
	string extractCulturalTypeFromFilename(const string& filename);

private:
	unordered_map<string, string> mappings;

};