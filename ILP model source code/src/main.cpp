#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <io.h>
#include <experimental/filesystem>
#include "definitions.h"
#include "solver.h"
#include "NoThriftGraph.h"
#include "NoThriftModel.h"
#include "Logger.h"

using namespace cplex_solver;
using namespace std;
namespace filesys = std::experimental::filesystem;

#ifndef _UNISTD_H
#define _UNISTD_H    1

#define access _access_s
#else
#include <unistd.h>
#endif

vector<Problem> problemGenerator();

int main(int argc, char** argv) {

	/*Update counter file*/
	int count = 0;
	ifstream infile("counter.txt"); infile >> count;
	ofstream count_file; count_file.open("counter.txt"); count_file << count + 1; count_file.close();

	clock_t tmpTime = clock();
	clock_t startTime = clock();

	// Generates test instances (IJCAI 2024)
	auto problem = problemGenerator();

	int start, end;
	string text = "Enter start instance (0-29): ";
	cout << text << flush;
	cin >> start;
	text = "Enter end instance (0-29): ";
	cout << text << flush;
	cin >> end;

	

	int nThreads;
	string text2 = "Enter the number of threads to use: ";
	cout << text2 << flush;
	cin >> nThreads; cout << endl;

	for (int instance = start; instance < end; instance++) {
		cout << endl;
		cout << "Instance: " << instance << endl;

		MODEL model;
		try {
			cout << "Parsing mission... " << flush;
			model.parseGenData(problem[instance], instance);
			cout << "Done!" << endl;
		}
		catch (const exception& e) {
			cerr << "Error occurred: " << e.what() << endl;
			throw;
		}

		/*--------------------------------------*/
		cplex_solver::Graph graph = cplex_solver::Graph(model);
		/*--------------------------------------*/

		graph.nThreads = nThreads;
		LOG logger2(instance);
		graph.path = logger2.getPath();

		cout << "Done!" << endl;
		printf("Graph created in: %.2fs\n", (double)(clock() - tmpTime) / CLOCKS_PER_SEC);
		tmpTime = clock();
		cout << "Initializing solver... " << flush;

		/*--------------------------------------*/
		auto solver = Solver(graph);
		/*--------------------------------------*/

		cout << "Done!" << endl;
		cout << endl;
		printf("Solver initialized in: %.2fs\n", (double)(clock() - tmpTime) / CLOCKS_PER_SEC);
		tmpTime = clock();

		cout << "Creating model... " << flush;

		/*--------------------------------------*/
		auto tours = solver.solve_and_print();
		/*--------------------------------------*/

		solver.~Solver();
		graph.~Graph();
	}
	int a;
	cin >> a;
	return 0;
}
