#pragma once

class FileHandler {
public:
	FileHandler(void);
	~FileHandler(void);
	static const char*  FileHandler::fileLoadDialog();
	static const char*  FileHandler::fileSaveDialog();

};

