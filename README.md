# Visualiztion of point clouds with OpenFrameworks and PCL
This app is part of a project that is conducted at the Computer Graphics Laboratory, Department of Computer Science, Ben-Gurion University, Israel.

In this project, a cloud that consists of millions of points in 3D space are rendered on screen. The input file ("pointcloud") is a proprietary file of a commercial company, and is the result of a high-accuracy laser scanner.
On this scene, 3D objects (defined in `.stl` format) are rendered. The company provides for each object a transformation that defines its location in relation to local space (the laser scanner is situated at this space's origin).
The final step is to dynamically define an epsilon-environment (via a simple slider). Each point from the cloud that is in the epsilon-environment of the `STL` will be rendered, so the end result is a scene that consists of `STL`s with points superimposed. This will enable future applications in the field of machine learning.

A summary f the project can be read in the file `Final Report - Topics in computer graphics.pdf`.

# Technical Information

This project is intended to run on Windows. It is tested and functioning properly on Windows 10.

## Dependencies
1. Visual Studio 2015
2. [PCL 1.7.2](http://unanancyowen.com/?p=1255&lang=en)
3. [OpenFrameworks](http://www.openframeworks.cc/)

## System Configuration
Since the app is written in VS2015, it has an attached `.vcxproj` file that manages its Configuration. In order for the compiler and linker to execute properly, the following environment variables should be define:
1. `PCL_ROOT` - PCL's installation root. This is set during the installation of PCL.
2. `OPEN_FRAMEWORKS` - set it manually to point to OF's installation root.

## Project Configuration
1. In the window `Property Manager`, open the properties of `Debug | x64`. On the left side, paste into `C/C++ -> Additional Include Directories` the following:
```
$(PCL_ROOT)\3rdParty\Boost\include\boost-1_57;
$(PCL_ROOT)\include\pcl-1.7;
$(PCL_ROOT)\3rdParty\FLANN\include;
$(PCL_ROOT)\3rdParty\Eigen\eigen3;
$(PCL_ROOT)\3rdParty\VTK\include\vtk-6.2;
%(AdditionalIncludeDirectories);
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\include;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\include\assimp;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\include\assimp\Compiler;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib\emscripten;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib\vs;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib\vs\Win32;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib\vs\x64;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\license;
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\src;
$(OPEN_FRAMEWORKS)\addons\ofxGui\src
```
2. Under `General-> Project Defaults`, set `Character Set` to `Not Set`.

3. Under `Linker -> Additional Library Directories`, paste:
```
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\libs\assimp\lib\vs\x64;
$(PCL_ROOT)\3rdParty\;
$(PCL_ROOT)\lib;
%(AdditionalLibraryDirectories)
```
4. Drag-n-drop the file `PCL.props` into `Debug | x64` in the property manager. This will make the file the 5th property sheet under `Debug | x64`. Open it, and paste the following to `C/C++ -> Additional Include Directories`:
```
$(PCL_ROOT)\include\pcl-1.7;
$(PCL_ROOT)\3rdParty\Boost\include\boost-1_57;
$(PCL_ROOT)\3rdParty\Eigen\eigen3;
$(PCL_ROOT)\3rdParty\FLANN\include;
$(PCL_ROOT)\3rdParty\QHull\include;
$(PCL_ROOT)\3rdParty\VTK\include\vtk-6.2;
%(AdditionalIncludeDirectories)
```

5. ...and the following to `Linker -> Additional Library Directories`:
```
$(PCL_ROOT)\lib;
$(PCL_ROOT)\3rdParty\Boost\lib;
$(PCL_ROOT)\3rdParty\FLANN\lib;
$(PCL_ROOT)\3rdParty\QHull\lib;
$(PCL_ROOT)\3rdParty\VTK\lib;
%(AdditionalLibraryDirectories)
```

6. Furthermore, the project should be launched from within VS2015 in `x64, Debug` configuration.

7. Add to the file
```
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\src\ofxAssimpModelLoader.h
```
the following method declaration:
```
void updateMatrix(ofMatrix4x4 mat);
```
8. ...and add its implementation to the file
```
$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\src\ofxAssimpModelLoader.cpp
```
:
```
void ofxAssimpModelLoader::updateMatrix(ofMatrix4x4 mat) {
	modelMatrix.makeFromMultiplicationOf(modelMatrix, mat);
}
```
9. You'll have to change some hard-coded paths in certain files in order for this to work properly:
```
Object3dModel.h
ofApp.h
ofApp.cpp
```
10. Cloud input files are assumed to be in `.gszf` format.
11. Cultural models transformation files are assumed to be in `.csv` format, and their actual content in a certain format.
12. Same goes for the transformation files that depict the clouds' transformations from local spaces to global space.

## Usage
1. Load the app.
2. Wait 1-2 minutes for all of the transformation files and cultural models to be loaded.
3. Load a cloud, this takes 1-2 minutes.
4. You can change the camera's orientation with the mouse while left-clicking.
5. You can move the camera's position with w-a-s-d like in an FPS game. You can move the camera up\down with W\S and pan it counter\clockwise with A\D.

# Example screenshots
3 cloud rendered concurrently, bird's-eye view
![3scans](https://github.com/asafch/pointcloud_viewer/blob/master/3_scans.JPG)
Cloud and models vs. Google Earth
![appvsge](https://github.com/asafch/pointcloud_viewer/blob/master/kewl_beanz.JPG)
Street level view, note the benches
![benches](https://github.com/asafch/pointcloud_viewer/blob/master/with_benches.JPG)
...and without benches: an example of model category filtration per user reqeuest.
![nobenches](https://github.com/asafch/pointcloud_viewer/blob/master/without_benches.JPG)

## Credits
The app is based on [Natan Elul](mailto:eluln@post.bgu.ac.il )'s original project, with assistance from [Fanglin Gu](mailto:gfl699468@gmail.com).


## License
The MIT License (MIT)

Copyright (c) 2016 Asaf Chelouche, Ben Ben-Chaya

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.