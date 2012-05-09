#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "kukaplan.h"

using namespace std;
using namespace KukaPlan;

string robot_filename;
string scene_filename;
string path_in_filename;
vector<vector<double> > path_values;

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

vector<vector<double> > parse_path (const char* filename) {
	ifstream path_file (filename);

	string line;
	vector<vector<double> > return_values;
	while (getline(path_file, line)) {
		istringstream line_stream (line);
		double value;
		vector<double> values;
		while ((line_stream >> value)) {
			values.push_back(value);
		}
		return_values.push_back(values);
	}

	path_file.close();

	return return_values;
}

void print_path (const vector<vector<double> > &values) {
	cout << "Path:" << endl;

	for (unsigned int i = 0; i < values.size(); i++) {
		cout << "  ";
		for (unsigned int j = 0; j < values[i].size(); j++) {
			cout << values[i][j] << ", ";
		}
		cout << endl;
	}
}

int main (int argc, char* argv[]) {
	if (argc != 4) {
		usage(argv[0]);
	}

	for (int i = 1; i < argc; i++) {
		if (i == 1)
			robot_filename = argv[i];
		else if (i == 2)
			scene_filename = argv[i];
		else if (i == 3)
			path_in_filename = argv[i];
		else
			cerr << "Error: invalid number of arguments." << endl;
	}

	file_exists (robot_filename.c_str());
	file_exists (scene_filename.c_str());
	file_exists (path_in_filename.c_str());

	path_values = parse_path (path_in_filename.c_str());
	print_path (path_values);

	kukaplan_initialize (robot_filename.c_str(), scene_filename.c_str());

	cout << "Checking path...";
	cout.flush();
	if (!kukaplan_check_path(path_values)) {
		cout << " found collision: success!" << endl;
	} else {
		cout << " no collision: fail!" << endl;
	}

	return 0;
}
