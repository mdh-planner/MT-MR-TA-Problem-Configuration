// Magic tricks to have CPLEX behave well:
#ifndef IL_STD
#define IL_STD
#endif
// End magic tricks

#include <cstring>
#include "ilcp/cp.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include "definitions.h"
#include "NoThriftModel.h"
#include "NoThriftGraph.h"

class IncumbentSolutionCallback : public IloCP::Callback {
private:
	ILOSTD(ostream&) _out;
	string path;
	clock_t t;

public:
	IncumbentSolutionCallback(ostream& out, string& incumbPath, clock_t& timer) : _out(out), path(incumbPath), t(timer) { }

	void invoke(IloCP cp, IloCP::Callback::Reason reason) {

		if (reason == IloCP::Callback::Solution) {
			ofstream myfile(path, ios_base::app | ios_base::out);
			// End the timer after solving
			clock_t time = clock();
			// Get the total time spent in solving (including CPLEX time)
			double incumbTime = ((double)(time - t)) / CLOCKS_PER_SEC;
			double gap = cp.getObjGap();
			
			//myfile << cp.getObjValue() << "\t" << cp.getObjBound() << "\t" << incumbTime << endl;
			myfile << cp.getObjValue() << "\t" << (double)gap << "\t" << incumbTime << endl;
			myfile.close();
		}
	}
};

void multiTaskMultiRobot(MODEL& mod, cplex_solver::Graph& graph, int instance) {

	/* CP MODEL */
	int sigma = mod.mtsp; // the number of robots
	int delta = mod.ddepot; // the number of destination depots
	int V = mod.nbCities - sigma - delta; // the number of tasks
	int Vtilde = V + sigma + delta; // the number of nodes
	int n = accumulate(mod.APT.begin(), mod.APT.end(), 0); // the total number of tasks (MR are split to SR)
	int nvt = accumulate(mod.vt.begin(), mod.vt.end(), 0);  // the number of virtual tasks

	IloEnv env;
	IloModel model(env);
	string name;

	IloIntervalVarArray overT(env, n), D(env, sigma), SrcDepot(env, sigma);
	IloIntervalVarArray2 A(env, n), AD(env, sigma);;
	IloIntArray POS(env), POS_pt(env), POS_s(env), POS_d(env);

#pragma region -- DIVs --

	int count = 0;
	/* Create array of Physical Tasks + depots */
	for (int i = 0; i < Vtilde; i++) {
		if (graph.VT()[i] != 1 || i < sigma || i >= V + sigma) {
			POS.add(count++);
		}
	}

	/* Create index array of Source Depots */
	for (int i = 0; i < sigma; i++) {
			POS_s.add(i);
	}

	count = 0;
	/* Create index array of Physical Tasks */
	for (int i = sigma; i < Vtilde-delta; i++) {
		if (graph.VT()[i] != 1) {
			POS_pt.add(count++);
		}
	}

	/* Create index array of Destination Depots */
	for (int i = V + sigma; i < Vtilde; i++) {
		POS_d.add(i);
	}

	/* Intervals representing the source depot of agents */
	for (int i = 0; i < sigma; i++) {
		IloIntervalVar tmp(env);
		SrcDepot[i] = tmp;
		SrcDepot[i].setSizeMin(1); SrcDepot[i].setSizeMax(1);
		SrcDepot[i].setStartMin(-1); SrcDepot[i].setStartMax(-1);

		name = "S_" + to_string(i);
		SrcDepot[i].setName(name.c_str());
	}

	/* Allocation matrix A */
	for (int i = 0; i < sigma; i++) {
		AD[i] = IloIntervalVarArray(env, delta);
		for (int j = 0; j < delta; j++) {
			IloIntervalVar tmp(env);
			AD[i][j] = tmp;
			AD[i][j].setSizeMin(1); AD[i][j].setSizeMax(1);
			AD[i][j].setOptional();
			name = "DestDepot_" + to_string(i) + to_string(j);
			AD[i][j].setName(name.c_str());
		}
	}

	/* Intervals representing the destination depot of agents */
	for (int i = 0; i < sigma; i++) {
		IloIntervalVar tmp(env);
		D[i] = tmp;
		D[i].setSizeMin(1); D[i].setSizeMax(1);
		D[i].setPresent();
		name = "D_" + to_string(i);
		D[i].setName(name.c_str());
	}

	vector<int> trmInd;
	/* Create Tasks and set their duration */
	int idx = 0;
	for (int i = 0; i < n; i++) {
		auto taskDuration = graph.cd(idx + sigma);
		IloIntervalVar tmp(env);
		name = "Task^1_" + to_string(idx);
		tmp.setName(name.c_str());
		tmp.setSizeMin(taskDuration); tmp.setSizeMax(taskDuration);
		overT[i] = tmp; overT[i].setPresent();
		trmInd.emplace_back(idx);
		for (int ii = 1; ii < mod.APT[idx]; ii++) {
			IloIntervalVar tmp2(env);
			name = "Task^" + to_string(ii + 1) + "_" + to_string(idx);
			tmp2.setName(name.c_str());
			tmp2.setSizeMin(taskDuration); tmp2.setSizeMax(taskDuration);
			i++;
			overT[i] = tmp2;		
			overT[i].setName(name.c_str());
			trmInd.emplace_back(idx);
		}
		idx++;
	}

	/* Create double array of possible allocation of tasks to agents */
	for (int i = 0; i < n; i++) {
		A[i] = IloIntervalVarArray(env, sigma);
		for (int j = 0; j < sigma; j++) {
			IloIntervalVar tmp(env);
			A[i][j] = tmp;
			name = "a_" + to_string(i) + "," + to_string(j);
			A[i][j].setName(name.c_str());

			auto it = find(graph._agentsPerTasks[trmInd[i] + sigma].begin(), graph._agentsPerTasks[trmInd[i] + sigma].end(), j);
			if (it != graph._agentsPerTasks[trmInd[i] + sigma].end()) {
				A[i][j].setOptional();
			}
			else {
				A[i][j].setAbsent();
			}
		}
	}

	//A[0][0].setPresent();
	//A[1][1].setPresent();
	//A[2][0].setPresent(); A[2][0].setStartMin(297); A[2][0].setStartMax(297);
	//A[3][1].setPresent(); A[3][0].setStartMin(297); A[3][0].setStartMax(297);
	//A[4][0].setPresent();
	//A[5][1].setPresent();
	//A[6][0].setPresent();
	//A[7][1].setPresent();
	//A[8][0].setPresent(); A[8][0].setStartMin(261); A[8][0].setStartMax(261);
	//A[8][1].setAbsent();
	/* Transition Distances - source tasks destination. It is symmetric*/
	int ii = 0, jj = 0, s = 0;
	IloTransitionDistance M(env, POS.getSize());

	for (int i = 0; i < Vtilde; i++) {
		if (graph.VT()[i] == 0) {
			for (int j = 0; j < Vtilde; j++) {
				if (graph.VT()[j] == 0) {
					M.setValue(ii, jj, round(graph.cost(i, j, s)));
					//cout << round(graph.cost(i, j, s)) << "\t" << flush;
					jj++;
				}
			}
			jj = 0; ii++;
			//cout << endl;
		}
	}

	/*IloStateFunction - State function for each agent based on transition distances */
	IloStateFunctionArray f(env);

	for (int i = 0; i < sigma; i++) {
		IloStateFunction tmpPOSV(env, M);
		f.add(tmpPOSV);
	}

#pragma endregion

#pragma region -- Constraints --

	// Equation (15) -  Enforce constraints based on A
	for (int i = 0; i < overT.getSize(); i++) {
		model.add(IloAlternative(env, overT[i], A[i]));
	}

	// Equation (16) -  Enforce constraints based on AD
	for (int i = 0; i < sigma; i++) {
		model.add(IloAlternative(env, D[i], AD[i]));
	}

	///* Equation (17) -  Prevents tasks from overlapping unless it is allowed by matrix R */
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			for (int k = 0; k < sigma; k++) {
				//if (mod.R[trmInd[i]][trmInd[j]] != 1 && i != j) {
				if (mod.R[trmInd[i]+sigma][trmInd[j]+sigma] != 1 && i != j) {
					IloIntervalVarArray tmpOverlap(env);
 					tmpOverlap.add(A[i][k]); tmpOverlap.add(A[j][k]);
					//cout << "No overlap between " << A[i][k].getName() << " and " << A[j][k].getName() << endl;
					model.add(IloNoOverlap(env, tmpOverlap));
				}
			}
		}
	}

	/* Equation (18) -  All tasks must finish before the agent reach destination depot */
	for (int i = 0; i < overT.getSize(); i++) {
		for (int j = 0; j < sigma; j++) {
			for (int k = 0; k < delta; k++) {
				model.add(IloEndBeforeStart(env, overT[i], AD[j][k]));
			}
		}
	}

	/* Equation (19) - Precedence Constraints */
	for (int i = 0; i < n; i++) {
		if (graph.PV()[trmInd[i] + sigma] != 0) {			
			auto it = find(trmInd.begin(), trmInd.end(), graph.PV()[trmInd[i] + sigma] - sigma);
			auto index = std::distance(trmInd.begin(), it);
			model.add(IloEndBeforeStart(env, overT[i], overT[index]));
		}
	}

	/* Equation (20) - Source Depots */
	for (int i = 0; i < f.getSize(); i++) {
		model.add(IloAlwaysEqual(env, f[i], SrcDepot[i], POS_s[i]));
	}

	/* Equation (21) - Destination Depots */
	for (int i = 0; i < f.getSize(); i++) {
		for (int j = 0; j < delta; j++) {
			model.add(IloAlwaysEqual(env, f[i], AD[i][j], POS_d[j]-nvt));
		}
	}

	/* Equation (22) -  Physical Tasks */
	for (int i = 0; i < f.getSize(); i++) {
		for (int j = 0; j < n - 1; j++) {
			if (mod.vt[trmInd[j] + sigma] == 0) {
				model.add(IloAlwaysEqual(env, f[i], A[j][i], POS_pt[trmInd[j]] + sigma));
			}
		}
	}

	// Equation (24) - All robots must start at the same time an MR task
	for (auto i = 0; i < n - 1; i++) {
		if (trmInd[i] == trmInd[i + 1]) { //k_i = mod.APT[trmInd[i]]
			model.add(IloStartOf(overT[i]) == IloStartOf(overT[i + 1]));
		}	
	}

	// Optional -> Equation (23) - Two tasks on the same robot
	/* Matrix U must be defined in "noThriftGraph.cpp */
	//for (int i = 0; i < n; i++) {
	//	for (int j = 0; j < n; j++) {
	//		for (int k = 0; i < sigma; i++) {
	//			if (mod.cityDur[i][j] == 1) {
	//				model.add(IloPresenceOf(env, A[i][k]) == IloPresenceOf(env, A[j][k]));
	//			}
	//		}
	//	}
	//}

#pragma endregion

	IloIntExprArray endTimes(env);
	for (int i = 0; i < sigma; i++) {
		endTimes.add(IloStartOf(D[i]));
	}

	// Add Objective function the model
	model.add(IloMinimize(env, IloMax(endTimes)));

	IloCP cp(model);
	
	cp.exportModel("model.cpo");
	cp.setParameter(IloCP::LogSearchTags, IloCP::On);
	cp.setParameter(IloCP::WarningLevel, 0);
	cp.setParameter(IloCP::PrintModelDetailsInMessages, IloCP::On);
	cp.setParameter(IloCP::TimeLimit, 3600);
	cp.setParameter(IloCP::Presolve, IloCP::On);
	cp.setParameter(IloCP::LogVerbosity, IloCP::Terse);


	clock_t timer = clock();

	string incumbPath = graph.path + "incumbentSolution_" + to_string(instance) + ".txt";
	remove(incumbPath.c_str());
	IncumbentSolutionCallback cb(cp.out(), incumbPath, timer);
	cp.addCallback(&cb);

	bool sol = false;
	double solveTime;

	try {
		// Start the timer before solving the model
		clock_t startTime = clock();
		sol = cp.solve();
		// End the timer after solving
		clock_t endTime = clock();
		// Get the total time spent in solving (including CPLEX time)
		solveTime = ((double)(endTime - startTime)) / CLOCKS_PER_SEC;
	}
	catch (const IloException& e) {
		std::cerr << std::endl << std::endl;
		std::cerr << "CP Raised an exception:" << std::endl;
		std::cerr << e << std::endl;
		env.end();
		throw;
	}

	if (sol) {
		
		cp.out() << "Makespan \t: " << cp.getObjValue() << std::endl;
		std::cout << "Time spent in solve: " << solveTime << " seconds." << std::endl;

		// Show results
		for (int i = 0; i < overT.getSize(); i++) {
			for (int j = 0; j < sigma; j++) {
				if (cp.isPresent(A[i][j])) {
					cp.out() << overT[i].getName() << " on robot " << j << ": start time: " << cp.getStart(overT[i]) << " | end time : " << cp.getEnd(overT[i]) << endl;		
				}
			}
		}
		cout << endl; cout << endl;

		//Write output schedule to a file
		string path = graph.path + "output_" + to_string(instance) + ".txt";
		remove(path.c_str());
		ofstream myfile2(path, ios_base::app | ios_base::out);

		for (int i = 0; i < overT.getSize(); i++) {
			myfile2 << trmInd[i] << "\t" << cp.getStart(overT[i]) << "\t" << cp.getEnd(overT[i]) << "\t" << flush;
			for (int j = 0; j < sigma; j++) {
				if (cp.isPresent(A[i][j])) {
					myfile2 << j << "\t" << flush;
				}
			}
			myfile2 << "\t" << graph.VT()[trmInd[i] + sigma] << endl;
		}
		myfile2.close();

		// write results to a file
		// 1st row end time of robot 1
		// 2nd row end time of robot 2
		// ...
		// Total time spent in solve
		// Gap
		// Bound value
		path = graph.path + "depot2_" + to_string(instance) + ".txt";
		remove(path.c_str());	
		ofstream myfile3(path, ios_base::app | ios_base::out);

		for (int i = 0; i < sigma; i++) {
			for (int j = 0; j < delta; j++) {
				if (cp.isPresent(AD[i][j])) {

					myfile3 << cp.getStart(AD[i][j]) << endl;
					cout << "D[" << to_string(i) << "] " << cp.getStart(AD[i][j]) << "\t" << flush;

				}
			}
		}
		myfile3 << solveTime << endl;
		myfile3 << cp.getObjGap() << endl;
		myfile3 << cp.getObjBound() << endl;
		myfile3.close();
	}
	else {
		// If the problem was shown to be infeasible, 
		//    find a minimal explanation for infeasibility
		if (cp.refineConflict()) {
			cp.writeConflict(cp.out());
		}
		cp.out() << "No solution found." << std::endl;
	}
	return;
}


