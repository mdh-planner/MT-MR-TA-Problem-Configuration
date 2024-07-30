#pragma once
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <experimental/filesystem>


using namespace std;
namespace filesys = std::experimental::filesystem;

class LOG
{
public:
	
	LOG(int instance);

	string getPath() { return path; }

private:

	string path;
	int _nRun;
	char dir_path[14] = "Test Results\\";

	void createDir(const char *name, char *dir_path);
	void createDirS(string bla);
	bool checkIfDirectory(std::string &filePath);
	string pathCreator(int instance);

};

