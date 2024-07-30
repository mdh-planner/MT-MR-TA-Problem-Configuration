#include "Logger.h"

LOG::LOG(int instance)
{
	string directory_path(dir_path);
	bool resDir = checkIfDirectory(directory_path);

	if (!resDir) {
		createDir("", dir_path);
	}

	path = pathCreator(instance);
}

void LOG::createDir(const char *name, char *dir_path) {

	strcat(dir_path, name);
	filesys::path dir(dir_path);

	if (filesys::create_directory(dir)) {
		cout << endl;
		std::cout << "Directory has been Successfully created" << "\n";
		cout << endl;
	}
}

void LOG::createDirS(string bla) {
	filesys::path dir(bla);

	if (filesys::create_directory(dir)) {
		cout << endl;
		std::cout << "Directory has been Successfully created" << "\n";
		cout << endl;
	}
}

bool LOG::checkIfDirectory(std::string &filePath)
{
	try {
		// Create a Path object from given path string
		filesys::path pathObj(filePath);
		// Check if path exists and is of a directory file
		if (filesys::exists(pathObj) && filesys::is_directory(pathObj))
			return true;
	}
	catch (filesys::filesystem_error & e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

string LOG::pathCreator(int instance) {

	int count;
	ifstream infile("counter.txt");
	infile >> count;

	string path = dir_path + to_string(count) + "\\";

	if (!checkIfDirectory(path)) {
		createDirS(path);
	}

	path += "Instance_" + to_string(instance);

	if (!checkIfDirectory(path)) {
		createDirS(path);
	}

	return path + "\\";
};