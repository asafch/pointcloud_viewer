#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include <pcl/features/principal_curvatures.h>

#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"

#include "Camera.h"
#include "Cloud.h"
#include "Cultural.h"
#include "FileHandler.h"
#include "ObjectsLib.h"

#define NORMAL_RADIUS 0.01
#define MAX_NUM_OF_CONCURRENT_CLOUDS 3
#define CULTURALS_LIST "C:\\scans\\transformations\\TERRAIN_CULT.csv"

using namespace std;

typedef struct mpoints {
	GLuint	 numberOfPoints;
	GLfloat* points;
	GLfloat* colors;
} Points;

class ofApp : public ofBaseApp {

private:
	//methods
	enum STATUS_MOVE { GLOBAL = 0, LOCAL = 1 } statusMoveEnum;
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
	void circleResolutionChanged(int & circleResolution);
	void drawAxis();
	void loadStlFunction();
	void loadScanFunction();
	void saveScanFunction();
	void changeMode();
	//ofMesh* meshFromCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures);
	//fields
	bool bHide;
	bool mouseTouch;
	//fields
	ofTrueTypeFont font;
	string outputInfo;
	ofxButton loadScanButton;
	ofxButton saveScanButton;
	ofxFloatSlider fov;
	ofxFloatSlider pointSize;
	ofxColorSlider color;
	ofxVec3Slider center;
	ofxIntSlider circleResolution;
	ofxLabel cameraStatus;
	ofxPanel gui;
	ofSoundPlayer ring;
	Camera camera;
	ofRectangle viewport3D;
	ObjectsLib* objects;
	vector<string> transformationFiles;
	unordered_map<string, ofMatrix4x4*> transformations;
	vector<Cloud*> clouds;
	vector<ofxLabel*> cloudNames;
	vector<Cultural*> culturals;
	ofxToggle filteredImage;
	ofxButton loadStlButton;
	ofxToggle bush;
	ofxToggle tree;
	ofxToggle lamppost;
	ofxToggle construction;
	ofxToggle car;
	ofxToggle pole;
	ofxToggle TS;
	ofxToggle TM;
};
