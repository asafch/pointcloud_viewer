#include "../include/ofApp.h"

//ofMesh* meshCloudNormals = NULL;
//pcl::PointCloud<pcl::Normal>::Ptr	cloudNormals(new pcl::PointCloud<pcl::Normal>);
//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>());

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
		string name;
		float translationX, translationY, translationZ, Q1, Q2, Q3, Q4, scaleX, scaleY, scaleZ;
		size_t start = 0;
		size_t end;
		end = line.find(",", start);
		name = line.substr(0, end);
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
		Cultural *cultural = new Cultural(name, translationX, translationY, translationZ, Q1, Q2, Q3, Q4, scaleX, scaleY, scaleZ);
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

void ofApp::populateSelectedCategories() {
	selectedCategories.emplace("Transportation", true);
	selectedCategories.emplace("Street Objects", true);
	selectedCategories.emplace("Bus Stations", true);
	selectedCategories.emplace("Plants", true);
	selectedCategories.emplace("Construction & Buildings", true);
	selectedCategories.emplace("Misc.", true);
	selectedCategories.emplace("Parks", true);
	selectedCategories.emplace("Furniture", true);
	selectedCategories.emplace("Phone Booths", true);
}

void ofApp::setup() {
	mappings = new Mappings();
	populateSelectedCategories();
	showModelsButtonPressed = false;
	parseTransformations();
	parseCulturals();
	mouseTouch = false;
	ofSetVerticalSync(true);
	// we add this listener before setting up so the initial circle resolution is correct
	//	circleResolution.addListener(this, &ofApp::circleResolutionChanged);
	ofBackground(0);
	//loadStlButton.addListener(this, &ofApp::loadStlFunction);
	loadScanButton.addListener(this, &ofApp::loadScanFunction);
	saveScanButton.addListener(this, &ofApp::saveScanFunction);
	// Buttons::
	gui.setup(); // most of the time you don't need a name
	statusMoveEnum = GLOBAL;
	gui.add(cameraStatus.setup("Mode", (GLOBAL == statusMoveEnum) ? "Global" : "Local"));
	gui.add(loadScanButton.setup("Load point cloud", false));
	gui.add(saveScanButton.setup("Save visible point cloud", false));
	gui.add(fov.setup("FOV", 60, 0, 180));
	gui.add(pointSize.setup("Point size", 1, 1, 10));
	//camera.setPosition(3476, 2872, 18);
	gui.add(coordinate.setup("Coordinate", camera.getGlobalPosition(), ofVec3f(-5000, -5000, -5000), ofVec3f(5000, 5000, 5000)));
	gui.add(showFilteredCloudsToggle.setup("Filter image", false));
	gui.add(showModelsButton.setup("Show models", false));
	showModelsButton.addListener(this, &ofApp::showModelsFunction);
	outputInfo = "Load scan";
	//	camera.setAutoDistance(false);
	camera.setDistance(0.00);
	camera.setGlobalPosition(coordinate);
	//bHide = true;
	configViewportFullScreen(viewport3D);
	ofEnableSmoothing();
	//ring.loadSound("ring.wav");
	//objects = new ObjectsLib(ofVec3f(3495.679, 2892.808, 15.74835), ofVec3f(-3.30E-02, -4.67E-02, 3.428114), ofVec3f(-1.42576, 2.28E-03, -1.04E-02), ofVec3f(-3.583694, -0.266352218, 3.6992804));
	//objects->loadModels();
	camera.setNearClip(2);
	camera.setFarClip(6000);
	//mapCulturalsToCategories();
}

//void ofApp::mapCulturalsToCategories() {
//	culturalCategories.emplace("Ambulance", "Transportation");
//	culturalCategories.emplace("AP", "Street Objects");
//	culturalCategories.emplace("Barrier", "Street Objects");
//	culturalCategories.emplace("Boat", "Transportation");
//	culturalCategories.emplace("BR", "Street Objects");
//	culturalCategories.emplace("BS", "Bus Stations");
//	culturalCategories.emplace("Bush", "Plants");
//	culturalCategories.emplace("BushEnd", "Plants");
//	culturalCategories.emplace("BushPart", "Plants");
//	culturalCategories.emplace("Canoe", "Transportation");
//	culturalCategories.emplace("Car", "Transportation");
//	culturalCategories.emplace("Chair", "Furniture");
//	culturalCategories.emplace("ChairNew", "Furniture");
//	culturalCategories.emplace("Cone", "Street Objects");
//	culturalCategories.emplace("Construction", "Construction & Buildings");
//	culturalCategories.emplace("Conatainer", "Construction & Buildings");
//	culturalCategories.emplace("Crane", "Construction & Buildings");
//	culturalCategories.emplace("CraneHighGray", "Construction & Buildings");
//	culturalCategories.emplace("CraneHighRed", "Construction & Buildings");
//	culturalCategories.emplace("CraneHighYellow", "Construction & Buildings");
//	culturalCategories.emplace("CraneLowGray", "Construction & Buildings");
//	culturalCategories.emplace("CraneLowRed", "Construction & Buildings");
//	culturalCategories.emplace("CraneLowYellow", "Construction & Buildings");
//	culturalCategories.emplace("Debris", "Construction & Buildings");
//	culturalCategories.emplace("DW", "Street Objects");
//	culturalCategories.emplace("EB", "Street Objects");
//	culturalCategories.emplace("FH", "Street Objects");
//	culturalCategories.emplace("Flag", "Street Objects");
//	culturalCategories.emplace("Flower", "Plants");
//	culturalCategories.emplace("Forklift", "Construction & Buildings");
//	culturalCategories.emplace("FS", "Street Objects");
//	culturalCategories.emplace("GB", "Street Objects");
//	culturalCategories.emplace("GC", "Street Objects");
//	culturalCategories.emplace("GH", "Street Objects");
//	culturalCategories.emplace("Helicopter", "Transportation");
//	culturalCategories.emplace("kayak", "Transportation");
//	culturalCategories.emplace("LP", "Street Objects");
//	culturalCategories.emplace("MetalPile", "Construction & Buildings");
//	culturalCategories.emplace("misc", "Misc.");
//	culturalCategories.emplace("NB", "Misc.");
//	culturalCategories.emplace("PB", "Street Objects");
//	culturalCategories.emplace("PileOfTires", "Construction & Buildings");
//	culturalCategories.emplace("PL", "Street Objects");
//	culturalCategories.emplace("Plane", "Transportation");
//	culturalCategories.emplace("Plant", "Plants");
//	culturalCategories.emplace("Playground", "Parks");
//	culturalCategories.emplace("PM", "Street Objects");
//	culturalCategories.emplace("Pole", "Street Objects");
//	culturalCategories.emplace("PP", "Misc.");
//	culturalCategories.emplace("PR", "Misc.");
//	culturalCategories.emplace("Props_Bag", "Transportation");
//	culturalCategories.emplace("Props_Bicycle", "Transportation");
//	culturalCategories.emplace("Props_Box", "Misc.");
//	culturalCategories.emplace("Props_Chalkboard", "Misc.");
//	culturalCategories.emplace("PT", "Phone Booths");
//	culturalCategories.emplace("RecyclingBin", "Street Objects");
//	culturalCategories.emplace("SB", "Furniture");
//	culturalCategories.emplace("SBnew", "Furniture");
//	culturalCategories.emplace("Scooter", "Transportation");
//	culturalCategories.emplace("Sculpture", "Parks");
//	culturalCategories.emplace("SkyTrain", "Transportation");
//	culturalCategories.emplace("StoneRamp", "Parks");
//	culturalCategories.emplace("Sunshade", "Furniture");
//	culturalCategories.emplace("SunTent", "Furniture");
//	culturalCategories.emplace("Table", "Furniture");
//	culturalCategories.emplace("TableNew", "Furniture");
//	culturalCategories.emplace("TankerCar", "Transportation");
//	culturalCategories.emplace("TC", "Street Objects");
//	culturalCategories.emplace("TL", "Street Objects");
//	culturalCategories.emplace("Train_Car", "Transportation");
//	culturalCategories.emplace("Train_Engine", "Transportation");
//	culturalCategories.emplace("Tree_Maple", "Plants");
//	culturalCategories.emplace("Tree", "Plants");
//	culturalCategories.emplace("TS", "Misc.");
//	culturalCategories.emplace("WarningPost", "Street Objects");
//	culturalCategories.emplace("WL", "Misc.");
//}

//string ofApp::extractCulturalTypeFromFilename(const string& filename) {
//	size_t dot = filename.rfind(".");
//	string result = filename.substr(0, dot);
//	size_t lastSlash = result.rfind("\\");
//	result = result.substr(lastSlash + 1, result.length() - lastSlash);
//	size_t junk = result.rfind("-");
//	if (junk != string::npos) {
//		result = result.substr(0, junk);
//		return result;
//	}
//	junk = result.rfind("_");
//	if (junk != string::npos) {
//		result = result.substr(0, junk);
//		return result;
//	}
//	return result;
//}

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
		//deleteAllObject();
		//load pcd or gz file
		string filenameAsString(filename);
		size_t start = filenameAsString.find_last_of("\\");
		size_t end = filenameAsString.find_last_of(".");
		string justTheFile = filenameAsString.substr(start + 1, end - start - 1);
		ofMatrix4x4 *laserToWorld(transformations.at(justTheFile));
		Cloud *cloud = new Cloud(filename, laserToWorld, mappings, &selectedCategories);
		cout << "Matching culturals with the cloud...   ";
		for (vector<Cultural*>::iterator cultural = culturals.begin(); cultural != culturals.end(); cultural++) {
			cloud->addModel(*cultural);
		}
		cout << "Done." << endl;
		//ofMatrix4x4 convertMatrix = ofMatrix4x4(-0.710128, 0.699510, 0.080019, 3476.238525, -0.697860, -0.714366, 0.051697, 2872.002686, 0.093326, -0.019130, 0.995452, 18.049866, 0.000000, 0.000000, 0.000000, 1.000000);
		//convertMatrix = convertMatrix.getInverse();
		//*laserToWorld = laserToWorld->getInverse();
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-03.stl", *laserToWorld, 3495.51, 2916.46, 13.1929, 0, 0, 0.0553676, 0.998466, 2.99198, 1.15288, 3.11035));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-03.stl", *laserToWorld, 3510.25, 2902.25, 12.9986, 0, 0, 0.299134, 0.954211, 1.263, 0.923116, 1.45914));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-06.stl", *laserToWorld, 3332.81, 2753.45, 27.978, 0, 0, 0, 1, 0.923116, 0.769275, 2.08395));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-06.stl", *laserToWorld, 3324.08, 2753.77, 28.0656, 0, 0, -0.216439, 0.976296, 0.769275, 0.923116, 0.773067));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-06.stl", *laserToWorld, 3338.84, 2751.38, 27.978, 0, 0, 0, 1, 0.923116, 0.923115, 2.50307));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-06.stl", *laserToWorld, 3328.87, 2750.38, 27.7071, 0.000000592736, 0.000000921803, 0.707107, 0.707107, 0.923115, 0.481992, 2.08395));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-12.stl", *laserToWorld, 3415.53, 2851.4, 21.9735, 0, 0.000000119718, 0.5373, 0.843391, 0.481992, 0.692018, 0.814713));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-17.stl", *laserToWorld, 3326.95, 2753.91, 28.2844, 0, 0, 0, 1, 0.692018, 0.830408, 2.83354));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-17.stl", *laserToWorld, 3336.03, 2752.09, 27.9783, 0, 0, 0, 1, 0.830408, 0.830408, 1.80548));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-17.stl", *laserToWorld, 3336.47, 2748.98, 27.8467, 0, 0, 0, 1, 0.830408, 0.286896, 1.80548));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3412.39, 2834.48, 22.0429, 0.0209744, 0.00352302, 0.581101, 0.813554, 0.286896, 0.217706, 0.286896));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3405.42, 2841.9, 22.0522, 0.0189369, 0.0096816, 0.799306, 0.600548, 0.217706, 0.547814, 0.217706));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3407.02, 2810.8, 22.1889, 0.0164806, 0.0134436, 0.907626, 0.419241, 0.547815, 0.497852, 0.547814));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3415.6, 2819.35, 21.928, 0.0174816, 0.0121133, 0.872051, 0.488952, 0.497852, 0.547815, 0.497851));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3342.84, 2745.43, 26.5386, 0.0164806, 0.0134436, 0.907626, 0.419241, 0.547815, 0.547814, 0.547814));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3333.58, 2736.17, 26.9517, 0.0207269, -0.00476801, 0.225708, 0.973963, 0.547815, 0.475249, 0.547815));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3306.98, 2734.56, 27.7027, 0.0680521, 0.00900825, -0.610284, 0.789203, 0.475249, 0.318152, 0.382619));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3320.87, 2738, 27.412, 0.0211719, -0.00202181, 0.350905, 0.936169, 0.318152, 0.273881, 0.419216));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3320.85, 2733.71, 27.3537, 0.0128129, -0.0169755, -0.453148, 0.891181, 0.273881, 0.243102, 0.342988));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3320.82, 2729.39, 27.412, 0.0044383, -0.0208, -0.787322, 0.616176, 0.243102, 0.28855, 0.419216));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3316.55, 2737.94, 27.3537, 0.0044383, -0.0208, -0.787321, 0.616176, 0.288551, 0.318152, 0.342987));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3316.05, 2742.83, 27.412, 0.0211719, -0.00202181, 0.350905, 0.936169, 0.318152, 0.382618, 0.419216));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3315.44, 2734.1, 27.479, 0.0634017, 0.0263145, -0.385228, 0.920265, 0.382618, 0.382618, 0.382619));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3311.66, 2738.05, 27.5865, 0.060654, 0.0321454, -0.297212, 0.952341, 0.382618, 0.318152, 0.382619));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3320.83, 2741.11, 27.4297, 0.0199272, -0.00743263, 0.0966499, 0.995091, 0.318152, 0.156954, 0.419216));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3325.04, 2734.98, 27.3686, 0.0128129, -0.0169755, -0.453148, 0.891181, 0.156954, 0.336141, 0.196557));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3327.34, 2737.13, 25.7712, 0.0161834, -0.0137999, -0.24952, 0.968136, 0.336141, 0.336141, 0.420957));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3329.83, 2741.3, 25.7712, 0.0211719, -0.00202181, 0.350905, 0.93617, 0.336141, 0.336141, 0.420957));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3325.95, 2740.04, 25.7712, 0.00566755, 0.0204992, 0.982881, -0.18301, 0.336141, 0.336141, 0.420957));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3328.13, 2743.13, 25.7712, 0.00260857, -0.0211077, -0.838029, 0.545211, 0.336141, 0.42119, 0.420957));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-21.stl", *laserToWorld, 3320.12, 2746.35, 27.5442, 0.0193463, 0.00883504, 0.771977, 0.635294, 0.42119, 0.741675, 0.419216));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-23.stl", *laserToWorld, 3334.93, 2751.98, 27.7822, 0, 0, 0, 1, 0.741675, 0.741675, 1.29524));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-23.stl", *laserToWorld, 3336.27, 2752.69, 27.7822, 0, 0, 0, 1, 0.741675, 0.741675, 0.541346));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-23.stl", *laserToWorld, 3325.21, 2752.57, 28.0656, 0, 0, 0, 1, 0.741675, 0.741675, 0.463008));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-23.stl", *laserToWorld, 3325.59, 2750.04, 27.8671, 0, 0, 0, 1, 0.741675, 1.20859, 0.463008));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-23.stl", *laserToWorld, 3331.98, 2751.48, 27.7732, 0, 0, 0, 1, 1.20859, 0.699811, 1.09804));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-27.stl", *laserToWorld, 3475.48, 2912.98, 14.6335, 0.0185546, 0.0108069, 0.965703, -0.258759, 0.741675, 0.895977, 0.94319));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3346.16, 2705.76, 25.4006, 0, 0, 0.887011, -0.461749, 0.895977, 0.991332, 0.895977));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3346.8, 2706.86, 25.4006, 0, 0, -0.793353, 0.608761, 0.991332, 1.05847, 0.991331));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3347.55, 2707.48, 25.4006, 0, 0, 0.939693, -0.34202, 1.05847, 1.1404, 1.05847));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3348.4, 2708.55, 25.4006, 0, 0, -0.819152, 0.573576, 1.1404, 0.955505, 1.14039));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3349.86, 2709.69, 25.4006, 0, 0, 0.965926, -0.258819, 0.955505, 0.615304, 0.955505));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3352.25, 2709.77, 26.0366, 0, 0, 0.887011, -0.461749, 0.615304, 0.556119, 0.615304));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-28.stl", *laserToWorld, 3353.1, 2710.47, 25.7781, 0, 0, 0.887011, -0.461749, 0.556119, 1.63855, 0.556119));
		//cloud->addModel(new Object3dModel("C:\\scans\\culturals\\Tree-33.stl", *laserToWorld, 3330.4, 2751.06, 27.8279, 0, 0, 0, 1, 1.63855, 1, 1.63855));
		clouds.push_back(cloud);
		ofxLabel *label = new ofxLabel();
		cloudNames.push_back(label);
		gui.add(label->setup("Cloud", filename));
	}
	else {
		cout << "Displaying maximum number of clouds: " << MAX_NUM_OF_CONCURRENT_CLOUDS << endl;
	}
}

void ofApp::saveScanFunction() {
	//const char* filename = FileHandler::fileSaveDialog();
	//printf("Saving file\n");
	//pcl::io::savePCDFileBinary(filename, *cloud);
	//printf("Cloud save on %s", filename);
	//if (cloudNormals->points.size() > 0)
	//{
	//	const size_t lengthFileName = strlen(filename);
	//	char* fileNameNormal = new char[lengthFileName + 10];
	//	sprintf(fileNameNormal, "%s(normal)", filename);
	//	printf("Saving file Normals\n");
	//	pcl::io::savePCDFileBinary(fileNameNormal, *cloudNormals);
	//	printf("Cloud save on %s", fileNameNormal);
	//}
	//if (principal_curvatures->points.size() > 0)
	//{
	//	const size_t lengthFileName = strlen(filename);
	//	char* fileNameCurv = new char[lengthFileName + 10];
	//	sprintf(fileNameCurv, "%s(curvat)", filename);
	//	printf("Saving file Normals\n");
	//	pcl::io::savePCDFileBinary(fileNameCurv, *principal_curvatures);
	//	printf("Cloud save on %s", fileNameCurv);
	//}
}

void ofApp::exit() {
	//segmentPlaneButton.removeListener(this, &ofApp::segmentButtonPressed);
}

void ofApp::circleResolutionChanged(int & circleResolution) {
	//ofSetCircleResolution(circleResolution);
}

void ofApp::update() {
	//ofSetCircleResolution(circleResolution);
}

void ofApp::draw() {
	//Camera Configurations
	camera.begin(viewport3D);
	glPointSize(pointSize);
	camera.setFov(fov);
	//camera.setNearClip(2);
	//camera.setFarClip(6000);
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
				//showModelsButton = false;
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
	if (key == 'm')
	{
		if (GLOBAL == statusMoveEnum)
		{
			statusMoveEnum = LOCAL;
			cameraStatus = "Local";
		}
		else
		{
			statusMoveEnum = GLOBAL;
			cameraStatus = "Global";
		}
	}
	if (key == OF_KEY_F3)
	{
		fov = fov + (180 - fov) / 60;
	}

	if (key == OF_KEY_F2)
	{
		fov = fov - (fov) / 60;;
	}
	if (key == 'h') {
		bHide = !bHide;
	}
	//if (key == 's') {
	//	gui.saveToFile("settings.xml");
	//}
	if (key == 'l') {
		gui.loadFromFile("settings.xml");
	}
	if (key == ' ') {
		color = ofColor(255);
	}
}

void ofApp::keyReleased(int key) {
	switch (key) {
	case 'w':
		glTranslatef(0, 0, -10);
		cout << "w" << endl;
		break;
	case 'a':
		glTranslatef(-10, 0, 0);
		cout << "a" << endl;
		break;
	case 's':
		glTranslatef(0, 0, 10);
		cout << "s" << endl;
		break;
	case 'd':
		glTranslatef(10, 0, 0);
		cout << "d" << endl;
		break;
	}
}

void ofApp::mouseMoved(int x, int y) {

}

void ofApp::mouseScrolled(int x, int y, float scrollX, float scrollY) {
	if (scrollY < 0)
		fov = fov + (95 - fov) / (-scrollY * 10);

	if (scrollY > 0)
		fov = fov - (fov) / (scrollY * 10);
}

void ofApp::mouseDragged(int x, int y, int button) {
	//if (recPaint)
	//{
	//	cropSecond.x = x;
	//	cropSecond.y = y;
	//}
}

void ofApp::mousePressed(int x, int y, int button) {
	mouseTouch = true;
}

void ofApp::mouseReleased(int x, int y, int button) {
	coordinate = camera.getGlobalPosition();
	mouseTouch = false;

}

void ofApp::windowResized(int w, int h) {
	configViewportFullScreen(ofGetCurrentViewport());
}

void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::drawAxis() {
	ofVec3f vec[2 * 3];
	ofVec3f *xAxe = &vec[0], *yAxe = &vec[2], *zAxe = &vec[4];
	xAxe[0].set(0, 0, 0);
	xAxe[1].set(1, 0, 0);
	yAxe[0].set(0, 0, 0);
	yAxe[1].set(0, 1, 0);
	zAxe[0].set(0, 0, 0);
	zAxe[1].set(0, 0, 1);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(ofVec3f), &vec[0].x);
	glDrawArrays(GL_LINES, 0, 6);
}
void ofApp::changeMode() {

}

//ofMesh* ofApp::meshFromCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures) {
//	ofMesh* mesh = new ofMesh();
//	mesh->setMode(OF_PRIMITIVE_POINTS);
//	vector<double> vecPc1, vecPc2, vecNormalization;
//	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	float min = 50; float max = -50;
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		pcl::PrincipalCurvatures descriptor = principal_curvatures->points[i];
//
//		float k = (descriptor.pc1 + descriptor.pc2);
//		float newK = k * 0.1;
//		newK = -pow(newK, 4);
//		newK = exp(newK);
//		newK = 1 - newK;
//		if (newK < min)
//			min = newK;
//		if (newK > max)
//			max = newK;
//	}
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		pcl::PrincipalCurvatures descriptor = principal_curvatures->points[i];
//
//		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(normals, "curvature");
//
//		ofPoint point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//
//		//		double zeroToOne = 5 * normals->points[i].curvature;
//				//cout << zeroToOne << endl;
//		mesh->addVertex(point);
//		//mesh->addColor(ofFloatColor(descriptor.principal_curvature_x, descriptor.principal_curvature_y, descriptor.principal_curvature_z));
//		float k = (descriptor.pc1 + descriptor.pc2);
//		float newK = k * 0.1;
//		newK = -pow(newK, 4);
//		newK = exp(newK);
//		newK = 1 - newK;
//		float normlizationK = (newK - min) / (max - min);
//		//		cout << newK << endl;
//		mesh->addColor(ofFloatColor(normlizationK, 0.5, 1 - normlizationK));
//		vecPc1.push_back(descriptor.pc1);
//		vecPc2.push_back(descriptor.pc2);
//		vecNormalization.push_back(normlizationK);
//	}
//	return mesh;
//}
