# Visualiztion of point clouds with OpenFrameworks and PCL
This app is part of a project that is conducted at the Computer Graphics Laboratory, Department of Computer Science, Ben-Gurion University, Israel.

In this project, a cloud that consists of millions of points in 3D space are rendered on screen. The input file ("pointcloud") is a proprietary file of a commercial company, and is the result of a high-accuracy laser scanner.
On this scene, 3D objects (defined in `.stl` format) are rendered. The company provides for each object a transformation that defines its location in relation to local space (the laser scanner is situated at this space's origin).
The final step is to dynamically define an epsilon-environment (via a simple slider). Each point from the cloud that is in the epsilon-environment of the `STL` will be rendered, so the end result is a scene that consists of `STL`s with points superimposed. This will enable future applications in the field of machine learning.

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

7. Add to the file `$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\src\ofxAssimpModelLoader.h` the following method declaration:

    `void updateMatrix(ofMatrix4x4 mat);`

8. ...and add its implementation to the file `$(OPEN_FRAMEWORKS)\addons\ofxAssimpModelLoader\src\ofxAssimpModelLoader.cpp`:

```
void ofxAssimpModelLoader::updateMatrix(ofMatrix4x4 mat) {
	modelMatrix.makeFromMultiplicationOf(modelMatrix, mat);
}
```

## Credits
The app is based on [Natan Elul](mailto:eluln@post.bgu.ac.il )'s original project, with assistance from [Fanglin Gu](mailto:gfl699468@gmail.com).