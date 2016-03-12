#include "../include/Mappings.h"

Mappings::Mappings() {
	mappings.emplace("Ambulance", "Transportation");
	mappings.emplace("AP", "Street Objects");
	mappings.emplace("Barrier", "Street Objects");
	mappings.emplace("Boat", "Transportation");
	mappings.emplace("BR", "Street Objects");
	mappings.emplace("BS", "Bus Stations");
	mappings.emplace("Bush", "Plants");
	mappings.emplace("BushEnd", "Plants");
	mappings.emplace("BushPart", "Plants");
	mappings.emplace("Canoe", "Transportation");
	mappings.emplace("Car", "Transportation");
	mappings.emplace("Chair", "Furniture");
	mappings.emplace("ChairNew", "Furniture");
	mappings.emplace("Cone", "Street Objects");
	mappings.emplace("Construction", "Construction & Buildings");
	mappings.emplace("Conatainer", "Construction & Buildings");
	mappings.emplace("Crane", "Construction & Buildings");
	mappings.emplace("CraneHighGray", "Construction & Buildings");
	mappings.emplace("CraneHighRed", "Construction & Buildings");
	mappings.emplace("CraneHighYellow", "Construction & Buildings");
	mappings.emplace("CraneLowGray", "Construction & Buildings");
	mappings.emplace("CraneLowRed", "Construction & Buildings");
	mappings.emplace("CraneLowYellow", "Construction & Buildings");
	mappings.emplace("Debris", "Construction & Buildings");
	mappings.emplace("DW", "Street Objects");
	mappings.emplace("EB", "Street Objects");
	mappings.emplace("FH", "Street Objects");
	mappings.emplace("Flag", "Street Objects");
	mappings.emplace("Flower", "Plants");
	mappings.emplace("Forklift", "Construction & Buildings");
	mappings.emplace("FS", "Street Objects");
	mappings.emplace("GB", "Street Objects");
	mappings.emplace("GC", "Street Objects");
	mappings.emplace("GH", "Street Objects");
	mappings.emplace("Helicopter", "Transportation");
	mappings.emplace("kayak", "Transportation");
	mappings.emplace("LP", "Street Objects");
	mappings.emplace("MetalPile", "Construction & Buildings");
	mappings.emplace("misc", "Misc.");
	mappings.emplace("NB", "Misc.");
	mappings.emplace("PB", "Street Objects");
	mappings.emplace("PileOfTires", "Construction & Buildings");
	mappings.emplace("PL", "Street Objects");
	mappings.emplace("Plane", "Transportation");
	mappings.emplace("Plant", "Plants");
	mappings.emplace("Playground", "Parks");
	mappings.emplace("PM", "Street Objects");
	mappings.emplace("Pole", "Street Objects");
	mappings.emplace("PP", "Misc.");
	mappings.emplace("PR", "Misc.");
	mappings.emplace("Props_Bag", "Transportation");
	mappings.emplace("Props_Bicycle", "Transportation");
	mappings.emplace("Props_Box", "Misc.");
	mappings.emplace("Props_Chalkboard", "Misc.");
	mappings.emplace("PT", "Phone Booths");
	mappings.emplace("RecyclingBin", "Street Objects");
	mappings.emplace("SB", "Furniture");
	mappings.emplace("SBnew", "Furniture");
	mappings.emplace("Scooter", "Transportation");
	mappings.emplace("Sculpture", "Parks");
	mappings.emplace("SkyTrain", "Transportation");
	mappings.emplace("StoneRamp", "Parks");
	mappings.emplace("Sunshade", "Furniture");
	mappings.emplace("SunTent", "Furniture");
	mappings.emplace("Table", "Furniture");
	mappings.emplace("TableNew", "Furniture");
	mappings.emplace("TankerCar", "Transportation");
	mappings.emplace("TC", "Street Objects");
	mappings.emplace("TL", "Street Objects");
	mappings.emplace("Train_Car", "Transportation");
	mappings.emplace("Train_Engine", "Transportation");
	mappings.emplace("Tree_Maple", "Plants");
	mappings.emplace("Tree", "Plants");
	mappings.emplace("TS", "Misc.");
	mappings.emplace("WarningPost", "Street Objects");
	mappings.emplace("WL", "Misc.");
}

Mappings::~Mappings() {
	mappings.clear();
}

string Mappings::getCulturalCategory(const string &cultural) {
	string name = extractCulturalTypeFromFilename(cultural);
	return mappings.at(name);
}

string Mappings::extractCulturalTypeFromFilename(const string& filename) {
	//this method is lacking: for culturals like "Car04_Damaged01" it doesn't work
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