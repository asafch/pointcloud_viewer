#include "../include/ofApp.h"

//ofMesh* meshCloudNormals = NULL;
//pcl::PointCloud<pcl::Normal>::Ptr	cloudNormals(new pcl::PointCloud<pcl::Normal>);
//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>());

void ofApp::configViewportFullScreen(ofRectangle & viewport) {
// utlitly function to randomise a rectangle
viewport.x = 0;
viewport.y = 0;
viewport.width = ofGetWidth();
viewport.height = ofGetHeight();
}

void ofApp::parseCulturals() {
	cout << "Parsing culturals..." << endl;
	ifstream input(CULTURALS_LIST);
	if (!input.good()) {
		cout << "Error: opening culturals list" << endl;
		exit();
	}
	//the first line in the file is junk, so parse it out
	string line;
	getline(input, line);
	for (; getline(input, line);) {
		string name;
		float translationX, translationY, translationZ, Q1, Q2, Q3, Q4, scaleX, scaleY, scaleZ;
		size_t start = -1;
		size_t end;
		end = line.find(",", start + 1);
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
	cout << "Done." << endl;
}

void ofApp::setup() {
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
	gui.add(center.setup("Coordinate", camera.getGlobalPosition(), ofVec3f(-100, -100, -100), ofVec3f(100, 100, 100)));
	gui.add(filteredImage.setup("Filter image", false));
	gui.add(loadStlButton.setup("Load STLs", false));
	outputInfo = "Load scan";
	//	camera.setAutoDistance(false);
	camera.setDistance(0.00);
	camera.setGlobalPosition(center);
	//bHide = true;
	configViewportFullScreen(viewport3D);
	ofEnableSmoothing();
	//ring.loadSound("ring.wav");
	objects = new ObjectsLib(ofVec3f(3495.679, 2892.808, 15.74835), ofVec3f(-3.30E-02, -4.67E-02, 3.428114), ofVec3f(-1.42576, 2.28E-03, -1.04E-02), ofVec3f(-3.583694, -0.266352218, 3.6992804));
	objects->loadModels();
}

void ofApp::loadStlFunction() {
	cout << "kaki" << endl;
}

void ofApp::loadScanFunction() {
	if (clouds.size() < MAX_NUM_OF_CONCURRENT_CLOUDS) {
		const char* filename = FileHandler::fileLoadDialog();
		cout << "Loading file: " << filename << endl;
		//deleteAllObject();
		//load pcd or gz file
		Cloud *cloud = new Cloud(filename);
		clouds.push_back(cloud);
		const size_t lengthFileName = strlen(filename);
		//normals load
		char* fileNameNormal = new char[lengthFileName + 10];
		sprintf(fileNameNormal, "%s(normal)", filename);
		//if (pcl::io::loadPCDFile<pcl::Normal>(fileNameNormal, *cloudNormals) >= 0 ) //try to open first
		//	 meshCloudNormals = meshFromCloudAndNormals(cloud, cloudNormals);
		//curvature load
		char* fileNameCurv = new char[lengthFileName + 10];
		sprintf(fileNameCurv, "%s(curvat)", filename);
		//if (pcl::io::loadPCDFile<pcl::PrincipalCurvatures>(fileNameCurv, *principal_curvatures) >= 0) //try to open first
		//	meshCloudNormals = meshFromCloudAndNormals(cloud->getFullCloud(), cloudNormals, principal_curvatures);
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
	camera.setNearClip(2);
	camera.setFarClip(600);
	if (!(mouseTouch))
		camera.setGlobalPosition(center);
	//		ofEnableDepthTest();
	//draw 3D objects
	if (loadStlButton)
		objects->draw();
	//if (!(meshCloud))
	//{
	//	camera.end();
	//	gui.draw();
	//	return;
	//}
	ofPushMatrix();
	for (vector<Cloud*>::iterator cloud = clouds.begin(); cloud != clouds.end(); cloud++) {
		if (filteredImage)
			(*cloud)->getFilteredCloudMesh()->draw();
		else
			(*cloud)->getFullCloudMesh()->draw();
	}
	ofPopMatrix();
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
	if (key == 's') {
		gui.saveToFile("settings.xml");
	}
	if (key == 'l') {
		gui.loadFromFile("settings.xml");
	}
	if (key == ' ') {
		color = ofColor(255);
	}

}

void ofApp::keyReleased(int key) {

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
	center = camera.getGlobalPosition();
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
