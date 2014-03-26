// author : guido manfred
// mail : gmanfredi.mail@gmail.com
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace std;

#include "rpnp.h"

void read_file (string filepath);

const double uc = 329.032488151495;
const double vc = 241.997953783025;
const double fu = 750.032554928978;
const double fv = 747.332385418021;

rpnp PNP;

int number_of_points = 0;

int main(int argc, char **argv)
{
    assert (argc == 2 && "Usage : test_rpnp number_of_points");
    number_of_points = atoi(argv[1]);

    PNP.set_internal_parameters(uc, vc, fu, fv);
    PNP.set_maximum_number_of_correspondences(number_of_points);
    read_file ("/home/gmanfred/debug.txt");
    double R[3][3];
    double t[3];
    cout << PNP.compute_pose(R, t) << endl;
    PNP.print_pose(R, t);

    return 0;
}

void read_file (string filepath)
{
    ifstream file;
    file.open (filepath.c_str(), ifstream::in);
    string line;
    vector<string> coordinates;
    if (file.is_open()) {
        for (uint i = 0; i < number_of_points; ++i) {
            getline(file, line);
            boost::split (coordinates, line, boost::is_any_of (" "));
            PNP.add_correspondence(atof(coordinates[0].c_str()),
                                     atof(coordinates[1].c_str()),
                                     atof(coordinates[2].c_str()),
                                     atof(coordinates[3].c_str()),
                                     atof(coordinates[4].c_str()));
        }
    }
    else
        cerr << "couldn't open file" << endl;
}
