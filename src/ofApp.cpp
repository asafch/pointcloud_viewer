#include "../include/ofApp.h"

//ofApp::~ofApp() {
//
//}

void ofApp::configViewportFullScreen(ofRectangle &viewport) {
	viewport.x = 0;
	viewport.y = 0;
	viewport.width = ofGetWidth();
	viewport.height = ofGetHeight();
}

void ofApp::parseCulturals() {
	cout << "Parsing culturals... ";
	ifstream input(CULTURALS_LIST);
	if (!input.good()) {
		cout << endl << "Error: opening culturals list" << endl;
		exit();
	}
	//the first line in the file is just column titles, so parse it out
	string line;
	getline(input, line);
	for (; getline(input, line);) {
		string filename;
		float translationX, translationY, translationZ, Q1, Q2, Q3, Q4, scaleX, scaleY, scaleZ;
		size_t start = 0;
		size_t end;
		end = line.find(",", start);
		filename = line.substr(0, end);
		start = end + 1;
		end = line.find(",", start + 1);
		translationX = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		translationY = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		translationZ = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		Q1 = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		Q2 = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		Q3 = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		Q4 = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		scaleX = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		scaleY = stof(line.substr(start, end - start));
		start = end + 1;
		end = line.find(",", start + 1);
		scaleZ = stof(line.substr(start, end - start));
		Cultural *cultural = new Cultural(filename, translationX, translationY, translationZ, Q1, Q2, Q3, Q4, scaleX, scaleY, scaleZ);
		culturals.push_back(cultural);
	}
	input.close();
	cout << "Done: total of " << culturals.size() << " culturals." << endl;
}

void ofApp::parseTransformations() {
	/*
	* The vector 'transformationFiles' holds the paths to all the files that contain transformations for the scans.
	* These file names are used in ofApp::parseTransformations() to enable the user flexibility to load whatever scan he chooses,
	* so the clouds and respective STLs would be rendered in their respective world coordinates.
	*/
	transformationFiles.push_back("C:\\scans\\transformations\\Area1LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area2LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area4LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area5LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area6LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area7LaserVsWorld.csv");
	transformationFiles.push_back("C:\\scans\\transformations\\Area8LaserVsWorld.csv");
	cout << "Parsing " << transformationFiles.size() << " transformation files... ";
	// x, y, z are just temps to calculate the average center of all scans
	//float x = 0;
	//float y = 0;
	//float z = 0;
	for (vector<string>::iterator file = transformationFiles.begin(); file != transformationFiles.end(); file++) {
		ifstream input(*file);
		if (!input.good()) {
			cout << endl << "Error: opening tansformation file" + *file << endl;
			exit();
		}
		for (string line; getline(input, line);) {
			string name;
			float a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;
			size_t start = 0;
			size_t end;
			end = line.find(",", start);
			name = line.substr(0, end);
			start = end + 1;
			end = line.find(",", start + 1);
			/*
			If the line defines a 'WorldToLaser' transformation, ignore it.
			*/
			if (line.at(start) == 'W') {
				continue;
			}
			start = end + 1;
			end = line.find(",", start + 1);
			a1 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			a2 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			a3 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			a4 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			b1 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			b2 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			b3 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			b4 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			c1 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			c2 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			c3 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			c4 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			d1 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			d2 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			d3 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			d4 = stof(line.substr(start, end - start));
			start = end + 1;
			end = line.find(",", start + 1);
			ofMatrix4x4 *transformation = new ofMatrix4x4(a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4);
			transformations.emplace(name, transformation);
			//x += a4;
			//y += b4;
			//z += c4;
		}
		input.close();
	}
	//x /= transformations.size();
	//y /= transformations.size();
	//z /= transformations.size();
	cout << "Done: total of " << transformations.size() << " transformations." << endl;
	//cout << "x: " << x << " , y: " << y << ", z: " << z << endl;
}

bool ofApp::isCategorySelected(string &category) {
	if (category.compare("Transportation") == 0) {
		return transportation == true;
	}
	if (category.compare("Street Objects") == 0) {
		return streetObjects == true;
	}
	if (category.compare("Bus Stations") == 0) {
		return busStations == true;
	}
	if (category.compare("Plants") == 0) {
		return plants == true;
	}
	if (category.compare("Construction & Buildings") == 0) {
		return constructionAndBuildings == true;
	}
	if (category.compare("Misc.") == 0) {
		return misc == true;
	}
	if (category.compare("Parks") == 0) {
		return parks == true;
	}
	if (category.compare("Furniture") == 0) {
		return furniture == true;
	}
	if (category.compare("Phone Booths") == 0) {
		return phoneBooths == true;
	}
	cout << "Error: a non-existent category was requested for toggle check" << endl;
	return false;
}

void ofApp::setup() {
	mappings = new Mappings();
	showModelsButtonPressed = false;
	parseTransformations();
	parseCulturals();
	mouseTouch = false;
	ofSetVerticalSync(true);
	ofBackground(0);
	loadScanButton.addListener(this, &ofApp::loadScanFunction);
	gui.setup();
	statusMoveEnum = GLOBAL;
	gui.add(cameraStatus.setup("Mode", (GLOBAL == statusMoveEnum) ? "Global" : "Local"));
	gui.add(loadScanButton.setup("Load point cloud", false));
	gui.add(showFilteredCloudsToggle.setup("Filter image", false));
	gui.add(showModelsButton.setup("Show models", false));
	showModelsButton.addListener(this, &ofApp::showModelsFunction);
	outputInfo = "Load scan";
	camera.setDistance(0.00);
	configViewportFullScreen(viewport3D);
	ofEnableSmoothing();
	camera.setNearClip(2);
	camera.setFarClip(6000);
	oldX = viewport3D.getWidth() / 2;
	oldY = viewport3D.getHeight() / 2;
}

void ofApp::showModelsFunction() {
	if (clouds.size() == 0)
		cout << "Load at least one cloud in order to display models" << endl;
	else {
		showModelsButtonPressed = true;
		gui.add(transportation.setup("Transportation", true));
		gui.add(streetObjects.setup("Street Objects", true));
		gui.add(busStations.setup("Bus Stations", true));
		gui.add(plants.setup("Plants", true));
		gui.add(constructionAndBuildings.setup("Construction & Buildings", true));
		gui.add(parks.setup("Parks", true));
		gui.add(furniture.setup("Furniture", true));
		gui.add(phoneBooths.setup("Phone Booths", true));
		gui.add(misc.setup("Misc.", true));
	}
}

void ofApp::loadScanFunction() {
	if (clouds.size() < MAX_NUM_OF_CONCURRENT_CLOUDS) {
		const char* filename = FileHandler::fileLoadDialog();
		cout << "Loading file: " << filename << endl;
		string filenameAsString(filename);
		size_t start = filenameAsString.find_last_of("\\");
		size_t end = filenameAsString.find_last_of(".");
		string justTheFile = filenameAsString.substr(start + 1, end - start - 1);
		ofMatrix4x4 *laserToWorld(transformations.at(justTheFile));
		Cloud *cloud = new Cloud(filename, laserToWorld, mappings, this);
		cout << "Matching culturals with the cloud... ";
		for (vector<Cultural*>::iterator cultural = culturals.begin(); cultural != culturals.end(); cultural++) {
			cloud->addModel(*cultural);
		}
		cout << "Done." << endl << endl;
		clouds.push_back(cloud);
	}
	else {
		cout << "Displaying maximum number of clouds: " << MAX_NUM_OF_CONCURRENT_CLOUDS << endl;
	}
}

void ofApp::exit() {

}

void ofApp::update() {

}

void ofApp::draw() {
	camera.begin(viewport3D);
	for (vector<Cloud*>::iterator cloud = clouds.begin(); cloud != clouds.end(); cloud++) {
		ofPushMatrix();
		ofMultMatrix(ofMatrix4x4::getTransposedOf((*cloud)->getLaserToWorld()));
		//if (!(mouseTouch))
		//	camera.setGlobalPosition(center);
		if (showModelsButtonPressed)
			if (clouds.size() > 0)
				(*cloud)->drawModels();
			else {
				cout << "You have to load a scan before rendering models" << endl;
			}
		if (showFilteredCloudsToggle)
			(*cloud)->getFilteredCloudMesh()->draw();
		else
			(*cloud)->getFullCloudMesh()->draw();
		ofPopMatrix();
	}
	camera.end();
	gui.draw();
}

void ofApp::keyPressed(int key) {
	switch (key) {
		//move forward
		case 'w':
			camera.dolly(-10);
			break;
		//move left
		case 'a':
			camera.truck(-10);
			break;
		//move backwards
		case 's':
			camera.dolly(10);
			break;
		//move right
		case 'd':
			camera.truck(10);
			break;
		//move upwards
		case 'W':
			camera.boom(10);
			break;
		//move downwards
		case 'S':
			camera.boom(-10);
			break;
		//pan counter-clockwise
		case 'D':
			camera.pan(3);
			break;
		//pan clockwise
		case 'A':
			camera.pan(-3);
			break;
	}
}

void ofApp::keyReleased(int key) {

}

void ofApp::mouseMoved(int x, int y) {

}

void ofApp::mouseScrolled(int x, int y, float scrollX, float scrollY) {
	//move forward
	if (scrollY > 0)
		camera.dolly(-10);
	//move backwards
	else
		camera.dolly(10);
}

void ofApp::mouseDragged(int x, int y, int button) {
	//if (abs(x - oldX) > MOUSE_SENSITIVITY) {
	//	camera.pan((x - oldX < 0 ? -1 : 1) * 3);
	//	x = oldX;
	//}
	//if (abs(y - oldY) > MOUSE_SENSITIVITY) {
	//	camera.tilt((y - oldY < 0 ? -1 : 1) * 3);
	//	y = oldY;
	//}
}

void ofApp::mousePressed(int x, int y, int button) {
	mouseTouch = true;
}

void ofApp::mouseReleased(int x, int y, int button) {
	mouseTouch = false;
}

void ofApp::windowResized(int w, int h) {
	configViewportFullScreen(ofGetCurrentViewport());
}

void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::changeMode() {

}
