#include "../include/Mappings.h"

/*
These mappings are used in order to associate the different cultural items into categories, which have a 1-to-1 relation
with the toggle buttons in the GUI.
The culturals' names are cropped from the file names, as they were provided from the company the created them.
*/
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
	mappings.emplace("Car01", "Transportation");
	mappings.emplace("Car02", "Transportation");
	mappings.emplace("Car03", "Transportation");
	mappings.emplace("Car04", "Transportation");
	mappings.emplace("Car05", "Transportation");
	mappings.emplace("Car06", "Transportation");
	mappings.emplace("Car07", "Transportation");
	mappings.emplace("Car08", "Transportation");
	mappings.emplace("Car09", "Transportation");
	mappings.emplace("Car10", "Transportation");
	mappings.emplace("Car11", "Transportation");
	mappings.emplace("Car12", "Transportation");
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
	mappings.emplace("Bag", "Transportation");
	mappings.emplace("Bicycle", "Transportation");
	mappings.emplace("Bicycle02", "Transportation");
	mappings.emplace("Bicycle03", "Transportation");
	mappings.emplace("Box", "Misc.");
	mappings.emplace("Chalkboard", "Misc.");
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
	mappings.emplace("Maple", "Plants");
	mappings.emplace("Tree", "Plants");
	mappings.emplace("TS", "Misc.");
	mappings.emplace("TS-17", "Misc.");
	mappings.emplace("WarningPost", "Street Objects");
	mappings.emplace("WL", "Misc.");
}

Mappings::~Mappings() {
	mappings.clear();
}

string Mappings::getCulturalCategory(string &name) {
	return mappings.at(name);
}