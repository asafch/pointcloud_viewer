#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"

#include "Camera.h"
#include "Cloud.h"
#include "Cultural.h"
#include "FileHandler.h"
#include "Mappings.h"

#define NORMAL_RADIUS 0.01
#define MAX_NUM_OF_CONCURRENT_CLOUDS 3
#define CULTURALS_LIST "C:\\scans\\transformations\\ALL_CULT.csv"
#define MOUSE_SENSITIVITY 3

using namespace std;

typedef struct mpoints {
	GLuint	 numberOfPoints;
	GLfloat* points;
	GLfloat* colors;
} Points;

class ofApp : public ofBaseApp {

public:
	bool isCategorySelected(string &category);

private:
	//methods
	// should a destructor be implemented?
	//virtual ~ofApp();
	void parseCulturals();
	void parseTransformations();
	void setup();
	void update();
	void draw();
	void exit();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseScrolled(int x, int y, float scrollX, float scrollY);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	void configViewportFullScreen(ofRectangle & viewport);
	void showModelsFunction();
	void loadScanFunction();
	void changeMode();

	// what are these fields?
	enum STATUS_MOVE { GLOBAL = 0, LOCAL = 1 } statusMoveEnum;
	bool bHide;
	bool mouseTouch;
	ofTrueTypeFont font;
	string outputInfo;
	ofxLabel cameraStatus;

	//known fields
	Camera camera;
	ofRectangle viewport3D;
	Mappings* mappings;
	vector<string> transformationFiles;
	unordered_map<string, ofMatrix4x4*> transformations;
	unordered_map<string, string> culturalCategories;
	vector<Cloud*> clouds;
	vector<Cultural*> culturals;
	ofxToggle showFilteredCloudsToggle;
	ofxButton showModelsButton;
	ofxButton loadScanButton;
	ofxColorSlider color;
	ofxPanel gui;
	bool showModelsButtonPressed;
	ofxToggle transportation;
	ofxToggle streetObjects;
	ofxToggle busStations;
	ofxToggle plants;
	ofxToggle constructionAndBuildings;
	ofxToggle misc;
	ofxToggle parks;
	ofxToggle furniture;
	ofxToggle phoneBooths;
	float oldX;
	float oldY;
};
