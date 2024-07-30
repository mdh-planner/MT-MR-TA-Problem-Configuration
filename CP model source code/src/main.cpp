#include <cstring>
#include <vector>
#include "ilcp/cp.h"
#include "definitions.h"
#include "Logger.h"
#include "NoThriftModel.h"
#include "NoThriftGraph.h"

ILOSTLBEGIN

vector<Problem> problemGenerator();
void multiTaskMultiRobot(MODEL& mod, cplex_solver::Graph& graph, int instance);

int main(int argc, const char* argv[]) {

	/*Update counter file*/
	int count = 0;
	ifstream infile("counter.txt"); infile >> count;
	ofstream count_file; count_file.open("counter.txt"); count_file << count + 1; count_file.close();
	/*-------------------*/
	
	auto problem = problemGenerator();	
	/*int instance;
	string text = "Enter instance to use from 0 to " + to_string(problem.size() - 1) + " : ";
	cout << text << flush;
	cin >> instance; cout << endl;*/

	int start,end;
	string text = "Enter start instance :";
	cout << text << flush;
	cin >> start;
	 text = "Enter end instance :";
	cout << text << flush;
	cin >> end;

	for (int instance = start; instance < end; instance++) {
		cout << endl;
		cout << "Instance: " << instance << endl;
		MODEL model;
		// Create model from instance
		model.parseGenData(problem[instance]);
		cplex_solver::Graph graph = cplex_solver::Graph(model);
		//graph.nThreads = nThreads;
		// Log
		LOG logger(instance);
		graph.path = logger.getPath();
		// Solve
		multiTaskMultiRobot(model, graph, instance);
	}
}