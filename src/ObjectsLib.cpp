#include "../include/ObjectsLib.h"

//transCam: x, y ,z of camera
//rotateCam: yaw, pitch, roll of camera
//routeCam: x, y ,z of route
//routeCam: yaw, pitch, roll of route
ObjectsLib::ObjectsLib(ofVec3f transCam, ofVec3f rotateCam, ofVec3f transRoute, ofVec3f rotateRoute) {
	aiMatrix4x4 W, C, yawMatCam, pitchMatCam, rollMatCam, yawMatRoute, pitchMatRoute, rollMatRoute;
	pitchMatCam.RotationX(rotateCam.x, pitchMatCam);
	rollMatCam.RotationY(rotateCam.y, rollMatCam);
	yawMatCam.RotationZ(rotateCam.z, yawMatCam);
	aiMatrix4x4 transCamMat;
	transCamMat.Translation(aiVector3D(transCam.x, transCam.y, transCam.z), transCamMat);
	W = transCamMat * rollMatCam * pitchMatCam * yawMatCam;
	pitchMatRoute.RotationX(rotateRoute.x, pitchMatRoute);
	rollMatRoute.RotationY(rotateRoute.y, rollMatRoute);
	yawMatRoute.RotationZ(rotateRoute.z, yawMatRoute);
	aiMatrix4x4 transRouteMat;
	transRouteMat.Translation(aiVector3D(transRoute.x, transRoute.y, transRoute.z), transRouteMat);
	C = transRouteMat * pitchMatRoute * rollMatRoute * yawMatRoute;
	aiMatrix4x4 mat = (W * C);
	//convertMatrix = (W * C).Inverse();
	//1
	//convertMatrix = aiMatrix4x4(-0.343392	,0.938489	,0.036329	,3495.941162	,-0.938753	,-0.341792	,-0.043824	,2893.695068	,-0.028712	,-0.049153	,0.998378	,15.707409	,0	,0	,0,1);
	//4
	//convertMatrix = aiMatrix4x4(-0.720864	,0.690157	,0.063548	,3440.559326	,-0.687352	,-0.723659	,0.062163	,2837.935791	,0.088889	,0.001131	,0.996041	,22.775545	,0	,0	,0,1);
	//5
	//convertMatrix = aiMatrix4x4(-0.722922	,0.689255	,0.04808	,3423.45166	,-0.689546	,-0.724127	,0.012896	,2819.274414	,0.043705	,-0.02383	,0.99876	,23.648308,0	,0	,0,1);
	//11
	//convertMatrix = aiMatrix4x4(-0.740422, 0.67212, 0.005439, 3329.346191, -0.672058, -0.740433, 0.009837, 2725.515869, 0.010639, 0.003628, 0.999937, 29.147188, 0, 0, 0, 1);
	convertMatrix = ofMatrix4x4(-0.710128, 0.699510, 0.080019, 3476.238525, -0.697860, -0.714366, 0.051697, 2872.002686, 0.093326, -0.019130, 0.995452, 18.049866, 0.000000, 0.000000, 0.000000, 1.000000);
	convertMatrix = convertMatrix.getInverse();
}

ObjectsLib::~ObjectsLib(void) {
	for (vector<Object3dModel*>::iterator iter = models.begin(); iter != models.end(); iter++) {
		delete (*iter);
	}
}

void ObjectsLib::draw() {
	for (vector<Object3dModel*>::iterator iter = models.begin(); iter != models.end(); iter++) {
		(*iter)->draw();
	}
}

void ObjectsLib::loadModels() {
	addModel("C:\\scans\\culturals\\Tree-03.stl", 3495.51, 2916.46, 13.1929, 0, 0, 0.0553676, 0.998466, 2.99198, 1.15288, 3.11035);
	addModel("C:\\scans\\culturals\\Tree-03.stl", 3510.25, 2902.25, 12.9986, 0, 0, 0.299134, 0.954211, 1.263, 0.923116, 1.45914);
	addModel("C:\\scans\\culturals\\Tree-06.stl", 3332.81, 2753.45, 27.978, 0, 0, 0, 1, 0.923116, 0.769275, 2.08395);
	addModel("C:\\scans\\culturals\\Tree-06.stl", 3324.08, 2753.77, 28.0656, 0, 0, -0.216439, 0.976296, 0.769275, 0.923116, 0.773067);
	addModel("C:\\scans\\culturals\\Tree-06.stl", 3338.84, 2751.38, 27.978, 0, 0, 0, 1, 0.923116, 0.923115, 2.50307);
	addModel("C:\\scans\\culturals\\Tree-06.stl", 3328.87, 2750.38, 27.7071, 0.000000592736, 0.000000921803, 0.707107, 0.707107, 0.923115, 0.481992, 2.08395);
	addModel("C:\\scans\\culturals\\Tree-12.stl", 3415.53, 2851.4, 21.9735, 0, 0.000000119718, 0.5373, 0.843391, 0.481992, 0.692018, 0.814713);
	addModel("C:\\scans\\culturals\\Tree-17.stl", 3326.95, 2753.91, 28.2844, 0, 0, 0, 1, 0.692018, 0.830408, 2.83354);
	addModel("C:\\scans\\culturals\\Tree-17.stl", 3336.03, 2752.09, 27.9783, 0, 0, 0, 1, 0.830408, 0.830408, 1.80548);
	addModel("C:\\scans\\culturals\\Tree-17.stl", 3336.47, 2748.98, 27.8467, 0, 0, 0, 1, 0.830408, 0.286896, 1.80548);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3412.39, 2834.48, 22.0429, 0.0209744, 0.00352302, 0.581101, 0.813554, 0.286896, 0.217706, 0.286896);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3405.42, 2841.9, 22.0522, 0.0189369, 0.0096816, 0.799306, 0.600548, 0.217706, 0.547814, 0.217706);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3407.02, 2810.8, 22.1889, 0.0164806, 0.0134436, 0.907626, 0.419241, 0.547815, 0.497852, 0.547814);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3415.6, 2819.35, 21.928, 0.0174816, 0.0121133, 0.872051, 0.488952, 0.497852, 0.547815, 0.497851);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3342.84, 2745.43, 26.5386, 0.0164806, 0.0134436, 0.907626, 0.419241, 0.547815, 0.547814, 0.547814);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3333.58, 2736.17, 26.9517, 0.0207269, -0.00476801, 0.225708, 0.973963, 0.547815, 0.475249, 0.547815);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3306.98, 2734.56, 27.7027, 0.0680521, 0.00900825, -0.610284, 0.789203, 0.475249, 0.318152, 0.382619);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3320.87, 2738, 27.412, 0.0211719, -0.00202181, 0.350905, 0.936169, 0.318152, 0.273881, 0.419216);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3320.85, 2733.71, 27.3537, 0.0128129, -0.0169755, -0.453148, 0.891181, 0.273881, 0.243102, 0.342988);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3320.82, 2729.39, 27.412, 0.0044383, -0.0208, -0.787322, 0.616176, 0.243102, 0.28855, 0.419216);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3316.55, 2737.94, 27.3537, 0.0044383, -0.0208, -0.787321, 0.616176, 0.288551, 0.318152, 0.342987);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3316.05, 2742.83, 27.412, 0.0211719, -0.00202181, 0.350905, 0.936169, 0.318152, 0.382618, 0.419216);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3315.44, 2734.1, 27.479, 0.0634017, 0.0263145, -0.385228, 0.920265, 0.382618, 0.382618, 0.382619);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3311.66, 2738.05, 27.5865, 0.060654, 0.0321454, -0.297212, 0.952341, 0.382618, 0.318152, 0.382619);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3320.83, 2741.11, 27.4297, 0.0199272, -0.00743263, 0.0966499, 0.995091, 0.318152, 0.156954, 0.419216);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3325.04, 2734.98, 27.3686, 0.0128129, -0.0169755, -0.453148, 0.891181, 0.156954, 0.336141, 0.196557);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3327.34, 2737.13, 25.7712, 0.0161834, -0.0137999, -0.24952, 0.968136, 0.336141, 0.336141, 0.420957);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3329.83, 2741.3, 25.7712, 0.0211719, -0.00202181, 0.350905, 0.93617, 0.336141, 0.336141, 0.420957);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3325.95, 2740.04, 25.7712, 0.00566755, 0.0204992, 0.982881, -0.18301, 0.336141, 0.336141, 0.420957);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3328.13, 2743.13, 25.7712, 0.00260857, -0.0211077, -0.838029, 0.545211, 0.336141, 0.42119, 0.420957);
	addModel("C:\\scans\\culturals\\Tree-21.stl", 3320.12, 2746.35, 27.5442, 0.0193463, 0.00883504, 0.771977, 0.635294, 0.42119, 0.741675, 0.419216);
	addModel("C:\\scans\\culturals\\Tree-23.stl", 3334.93, 2751.98, 27.7822, 0, 0, 0, 1, 0.741675, 0.741675, 1.29524);
	addModel("C:\\scans\\culturals\\Tree-23.stl", 3336.27, 2752.69, 27.7822, 0, 0, 0, 1, 0.741675, 0.741675, 0.541346);
	addModel("C:\\scans\\culturals\\Tree-23.stl", 3325.21, 2752.57, 28.0656, 0, 0, 0, 1, 0.741675, 0.741675, 0.463008);
	addModel("C:\\scans\\culturals\\Tree-23.stl", 3325.59, 2750.04, 27.8671, 0, 0, 0, 1, 0.741675, 1.20859, 0.463008);
	addModel("C:\\scans\\culturals\\Tree-23.stl", 3331.98, 2751.48, 27.7732, 0, 0, 0, 1, 1.20859, 0.699811, 1.09804);
	addModel("C:\\scans\\culturals\\Tree-27.stl", 3475.48, 2912.98, 14.6335, 0.0185546, 0.0108069, 0.965703, -0.258759, 0.741675, 0.895977, 0.94319);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3346.16, 2705.76, 25.4006, 0, 0, 0.887011, -0.461749, 0.895977, 0.991332, 0.895977);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3346.8, 2706.86, 25.4006, 0, 0, -0.793353, 0.608761, 0.991332, 1.05847, 0.991331);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3347.55, 2707.48, 25.4006, 0, 0, 0.939693, -0.34202, 1.05847, 1.1404, 1.05847);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3348.4, 2708.55, 25.4006, 0, 0, -0.819152, 0.573576, 1.1404, 0.955505, 1.14039);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3349.86, 2709.69, 25.4006, 0, 0, 0.965926, -0.258819, 0.955505, 0.615304, 0.955505);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3352.25, 2709.77, 26.0366, 0, 0, 0.887011, -0.461749, 0.615304, 0.556119, 0.615304);
	addModel("C:\\scans\\culturals\\Tree-28.stl", 3353.1, 2710.47, 25.7781, 0, 0, 0.887011, -0.461749, 0.556119, 1.63855, 0.556119);
	addModel("C:\\scans\\culturals\\Tree-33.stl", 3330.4, 2751.06, 27.8279, 0, 0, 0, 1, 1.63855, 1, 1.63855);
}

void ObjectsLib::addModel(const char* filename, float x, float y, float z, float qx, float qy, float qz, float qw, float sX, float sY, float sZ) {
	Object3dModel* obj = new Object3dModel(filename, convertMatrix, x, y, z, qx, qy, qz, qw, sX, sY, sZ);
	models.push_back(obj);
}
