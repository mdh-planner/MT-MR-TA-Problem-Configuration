#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "definitions.h"

using namespace std;

class MODEL {
public:
	// Number of cities
	int nbCities;

	/* Number od Salesmen per Source Depot */
	vector<int> spsd;

	/* Salesmen Color */
	vector<vector<int>> scolor;

	/* City Color */
	vector<int> ccolor;

	/* Number of Salesmen */
	int mtsp;

	/* Number of Source Depot */
	int sdepot;

	/* Number of Destination Depot */
	int ddepot;

	/* Vector of distance between two cities */
	vector<vector<vector<int>>> distanceWeight;

	/* Vector of city durations */
	vector<int> cityDur;

	/* Precedence Vector */
	vector<int> pv;

	/* Virtual Tasks Vector */
	vector<int> vt;

	/* Physical Task with No Location Vector */
	vector<int> pnl;

	/* Concurrency Task Matrix */
	vector<vector<int>> R;

	/* U Matrix */
	vector<vector<int>> U;

	/* Agents Per Tasks vector apt */
	vector<int> APT;

	double pV; double pP;

	vector<vector<int>> P; /* precedance matrix */
	void parseGenData(Problem context, int instance);

};

