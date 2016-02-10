#include "../include/FileHandler.h"
#include "../include/ofApp.h"
#include <stdlib.h>

#include <Windows.h>
#include <Commdlg.h>
#include <tchar.h>
#include <fileapi.h>

FileHandler::FileHandler(void) {

}

FileHandler::~FileHandler(void) {

}

 const char* FileHandler::fileLoadDialog() {
	OPENFILENAME ofn;       // common dialog box structure
//	TCHAR  szFile[260];       // buffer for file name
	TCHAR* szFile = (TCHAR*) malloc(sizeof(TCHAR) * 260);
	const TCHAR* ret = szFile;
	HWND hwnd;              // owner window
	HANDLE hf;              // file handle
	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
//	ofn.hwndOwner = hwnd;
	ofn.lpstrFile = szFile;
	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof(TCHAR) * 260;
	//ofn.lpstrFilter = "All\0*.*\0Text\0*.TXT\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
	// Display the Open dialog box. 
	GetOpenFileName(&ofn) ;
	return szFile;
}

const char* FileHandler::fileSaveDialog() {
	OPENFILENAME ofn;       // common dialog box structure
//	TCHAR  szFile[260];       // buffer for file name
	TCHAR* szFile = (TCHAR*) malloc(sizeof(TCHAR) * 260);
	const TCHAR* ret = szFile;
	HWND hwnd;              // owner window
	HANDLE hf;              // file handle
	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
//	ofn.hwndOwner = hwnd;
	ofn.lpstrFile = szFile;
	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof(TCHAR) * 260;
	ofn.lpstrFilter = "PointCloud\0*.pcd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
	ofn.lpstrDefExt = _T("pcd");
	// Display the Open dialog box. 
	GetSaveFileName(&ofn) ;
	return szFile;
}


