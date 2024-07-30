#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

//#include "graph.h"

using namespace std;


	class TSP {
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
		vector<vector<int> > distanceWeight;

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

		/* Agents Per Tasks vector apt */
		vector<int> APT;

		void readInstanceTSP(const string& fileName);
	
//		void parseOutput(const swarms::Mission  & Context, vector<vector<vector<uint32_t>>> &tours, cplex_solver::Graph & graph);


	};


