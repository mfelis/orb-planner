#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>

#include "kukaplan.h"

using namespace std;
using namespace KukaPlan;

string robot_filename;
string scene_filename;
string path_in_filename;

void usage (char *program_name) {
	cout << "Usage: " << program_name << "<robot_file.kxml> <scene_file.kxml> <path_in.txt>" << std::endl;
	exit(1);
}

bool file_exists (const char* filename) {
	ifstream test_stream (filename);

	if (!test_stream) {
		cerr << "Error: could not open file " << filename << endl;
		abort();
	}

	test_stream.close();

	return true;
}

int main (int argc, char* argv[]) {
	if (argc != 4) {
		usage(argv[0]);
	}

	for (int i = 1; i < argc; i++) {
		if (i == 1)
			robot_filename = argv[i];
		if (i == 2)
			scene_filename = argv[i];
		if (i == 3)
			path_in_filename = argv[i];
		else
			break;
	}
	
	file_exists (robot_filename.c_str());
	file_exists (scene_filename.c_str());
	file_exists (path_in_filename.c_str());

	return 0;
}
