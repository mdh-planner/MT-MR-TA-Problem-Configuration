#include "solver.h"
#include "NoThriftGraph.h"
#include <cmath>
#include <limits>
#include <algorithm> 
#include <iterator>
#include <string.h>
#include <experimental/filesystem>

namespace filesys = std::experimental::filesystem;

vector<int> findElements(vector<uint32_t>& vec, int j) {
	vector<int> index;

	for (int i = 0; i < vec.size(); i++) {
		if (i != j) {
			if (vec[i] == vec[j]) {
				index.emplace_back(i);
			}
		}
	}

	return index;
}

// Tells if two floating-point numbers are equal (for all practical purposes)
auto almost_equal = [](double x, double y) {
	float magnitude = 10e5;
	float wtf = 0.1;
	return std::abs(x - y) < magnitude * std::numeric_limits<float>::epsilon() * std::abs(x + y) || std::abs(x - y) < wtf;
};

ILOMIPINFOCALLBACK6(MIPInfoCallback,
	IloNumArray, val,
	IloIntVarArray, vars,
	cplex_solver::Graph, g,
	IloNum, lastIncumbent,
	long, timestep,
	string, path) {

	if (hasIncumbent() &&
		fabs(lastIncumbent - getIncumbentObjValue())
	> 1e-5 * (1.0 + fabs(getIncumbentObjValue()))) {

		lastIncumbent = getIncumbentObjValue();

		getIncumbentValues(val, vars);

		string pathi = path + "IncumbentSolutions.txt";
		ofstream myfile(pathi, ios_base::app | ios_base::out);
		double getGap = getMIPRelativeGap();
		myfile << lastIncumbent << "\t" << (double)getGap << "\t" << (double)(clock() - timestep) / CLOCKS_PER_SEC << endl;
		myfile.close();
	}
}

namespace cplex_solver {

	vector<vector<vector<uint32_t>>> Solver::solve_and_print() const {

		clock_t tmpTime = clock();

#pragma region -- MODEL --

		// CPLEX environment. Takes care of everything, including memory management for CPLEX objects.
		IloEnv env;

		auto Vtilde = g.size();
		auto m = g.nSalesmen();
		auto sigma = g.SD();
		auto delta = g.DD();
		auto B = g.SDSP();
		auto V = Vtilde - sigma - delta;
		// K = g.APT() -> k_i = g.APT()[i]

		// CPLEX model. We put variables and constraints in it!
		IloModel model(env);

		// Decision Variable
		IloArray<IloArray<IloNumVarArray>> x(env, Vtilde); // Decision Variable x {0,1}
		IloArray<IloNumVarArray> y(env, Vtilde); // Decision Variable y {0,1}
		IloArray<IloArray<IloNumVarArray>> z(env, Vtilde); // int >= 0
		IloNumVarArray T(env, Vtilde);
#pragma endregion

#pragma region == Constraints Definition == 
		auto M = 10000; // PROBLEM DEPENDENT, general rule -> M > maximum possible cost of a plan;

/*  CONSTRAINTS  */
		IloArray<IloArray<IloRangeArray>> pc1(env, Vtilde);  // EQUATION(1)
		IloArray<IloArray<IloArray<IloRangeArray>>> pc3(env, Vtilde);  // EQUATION(3)										
		IloArray<IloArray<IloRangeArray>> cc(env, Vtilde);	// EQUATION(3)  A salesperson s is allowed to only visit the cities specified in its extended color matrix As */
		IloRangeArray mc(env, Vtilde);  // EQUATION(4) There can exist only one entry to a specific city.
		IloRangeArray vc(env, Vtilde);  // EQUATION(5) There can exist only exit from the city.
		IloRangeArray fd(env, Vtilde);  // EQUATION(6) The final location of a salesperson s is always a destination depot delta.
		IloRangeArray sd(env, Vtilde);  // EQUATION(7) The starting location of a salesperson s is always a source depot sigma.
		IloRangeArray ds(env, Vtilde);  // EQUATION(8) The number of salespersons per source depot.
		IloArray<IloRangeArray> test(env, Vtilde); // EQUATION (MISSING) Ensures that the same agent enters and exits a city.
		IloArray<IloRangeArray> TT(env, Vtilde);
		IloArray<IloRangeArray> PC(env, Vtilde);
		IloArray<IloRangeArray> TT7(env, Vtilde);
		IloArray<IloRangeArray> sameRobot(env, Vtilde); // Equation (10)

		// Initialize expr variables
		IloExpr expr(env); IloExpr expr2(env); IloExpr expr3(env); IloExpr expr4(env);

		// We use this stringstream to create variable and constraint names
		std::stringstream name; std::stringstream name2;

#pragma endregion

		/* ----------== Define x_ijs ==--------------- */
		for (auto i = 0u; i < Vtilde; ++i) {
			x[i] = IloArray<IloNumVarArray>(env, Vtilde);
			for (auto j = 0u; j < Vtilde; ++j) {
				x[i][j] = IloNumVarArray(env, Vtilde);
				for (auto s = 0u; s < Vtilde; s++) {
					if (s < m) {
						name << "x_" << i << "_" << j << "_" << s;

						/* ----------== Equation (1) ==--------------- */
						if ((i >= Vtilde - delta) || (j < sigma && i < sigma) || (i >= sigma && j < sigma) || i == j) {
							x[i][j][s] = IloNumVar(env, 0, 0, IloNumVar::Bool, name.str().c_str());
						}
						else {
							x[i][j][s] = IloNumVar(env, 0, 1, IloNumVar::Bool, name.str().c_str());
						}
					}
					else {
						x[i][j][s] = IloNumVar(env, 0, 0, IloNumVar::Bool, name.str().c_str());
					}
					name.str(""); // Clean name
					model.add(x[i][j][s]); 
				}
			}
		}

		/* ----------== Define z_ijs ==--------------- */
		for (auto i = 0u; i < Vtilde; ++i) {
			z[i] = IloArray<IloNumVarArray>(env, Vtilde);
			for (auto j = 0u; j < Vtilde; ++j) {
				z[i][j] = IloNumVarArray(env, Vtilde);
				for (auto s = 0u; s < m; s++) {

					name << "z_" << i << "_" << j << "_" << s;

					if ((i >= Vtilde - delta) || (j < sigma && i < sigma) || (i >= sigma && j < sigma) || i == j) {
						z[i][j][s] = IloNumVar(env, 0, 0, IloNumVar::Bool, name.str().c_str());
					}
					else {
						z[i][j][s] = IloNumVar(env, 0, 1, IloNumVar::Bool, name.str().c_str());
					}
					name.str(""); // Clean name
					model.add(z[i][j][s]); 
				}
			}
		}

		/* ----------== Define T_i ==--------------- */
		for (auto i = 0u; i < Vtilde; ++i) {

			name << "T_" << i;
			T[i] = IloNumVar(env, 0, INFINITY, IloNumVar::Float, name.str().c_str());

			name.str(""); // Clean name
			model.add(T[i]);
		}

		/* ----------== Equation (2) -  A robot s is allowed to only do tasks specified in its equipment matrix As ==--------------- */
		for (auto i = 0u; i < Vtilde - delta; i++) {
			cc[i] = IloArray<IloRangeArray>(env, Vtilde);
			for (auto j = sigma; j < Vtilde; j++) {
				cc[i][j] = IloRangeArray(env, Vtilde);
				for (auto s = 0u; s < m; s++) {
					if (g.A()[i][j][s] == 0) {

						expr = x[i][j][s];

						name << "cc_" << i << "_" << j << "_" << s;
						cc[i][j][s] = IloRange(env, 0, expr, 0, name.str().c_str());
						name.str(""); expr.clear(); // Clean expr
					}
				}
				model.add(cc[i][j]);
			}
		}

		/* ----------== Equation (3) - Establishing relation between x_ijs and z_ijs ==--------------- */
		for (auto i = 0; i < Vtilde - delta; ++i) {
			pc1[i] = IloArray<IloRangeArray>(env, Vtilde);
			for (auto j = sigma; j < Vtilde; ++j) {
				pc1[i][j] = IloRangeArray(env, Vtilde);
				for (auto s = 0; s < m; ++s) {
					if (i != j) {
						expr = z[i][j][s] - x[i][j][s];

						name << "ZZ_" << i << "_" << j << "_" << s;

						pc1[i][j][s] = IloRange(env, 0, expr, 1, name.str().c_str());

						name.str(""); expr.clear(); // Clean expr and name
					}
				}
				model.add(pc1[i][j]);
			}
		}

		/* ----------== Equation (4) -  The number of roborts starting a task ==--------------- */
		for (auto j = sigma; j < V + sigma; j++) {
			for (auto s = 0u; s < m; s++) {
				for (auto i = 0u; i < V + sigma; i++) {
					expr += x[i][j][s];
				}
			}
			name << "mc_" << j;
			mc[j] = IloRange(env, g.APT()[j], expr, g.APT()[j], name.str().c_str());
			name.str(""); expr.clear(); // Clean expr
		}

		model.add(mc);

		/* ----------== Equation (5) -  The number of roborts ending a task ==--------------- */
		for (auto i = sigma; i < V + sigma; i++) {
			if (i < g.SDSP().size() && g.SDSP()[i] != 0) {
				for (auto s = 0u; s < m; s++) {
					for (auto j = sigma; j < Vtilde; j++) {
						expr += x[i][j][s];
					}
				}

				name << "vc_" << i;
				vc[i] = IloRange(env, g.APT()[i], expr, g.APT()[i], name.str().c_str());
				name.str(""); expr.clear(); // Clean expr
			}
		}
		model.add(vc);

		/* ----------== Equation (6) -  forcing robots that started a task to be the ones to finish it ==--------------- */
		for (auto s = 0u; s < m; s++) {
			test[s] = IloRangeArray(env, Vtilde);
			for (auto j = sigma; j < V + sigma; j++) {

				for (auto i = 0u; i < V + sigma; i++) {
					expr3 += x[i][j][s];
				}
				for (auto k = sigma; k < Vtilde; k++) {
					expr2 += x[j][k][s];
				}

				expr = expr2 - expr3;

				name << "test_" << s << "_" << j; // city visit
				test[s][j] = IloRange(env, 0, expr, 0, name.str().c_str());
				name.str(""); // Clean name
				expr.clear(); expr2.clear(); expr3.clear();
			}
			model.add(test[s]);
		}

		/* ----------== Equation (7) -  forcing robots to start a mission at a source depot ==--------------- */
		for (auto s = 0u; s < m; s++) {
			for (auto i = 0u; i < sigma; i++) {
				for (auto j = sigma; j < Vtilde; j++) {
					expr += x[i][j][s];
				}
			}

			name << "sd_" << s; // city visit
			sd[s] = IloRange(env, 1, expr, 1, name.str().c_str());
			name.str(""); expr.clear(); // Clean expr
		}
		model.add(sd);

		/* ----------== Equation (8) -  forcing robots to go to destination depot at the end of a mission ==--------------- */
		for (auto s = 0u; s < m; s++) {
			for (auto i = 0; i < V + sigma; i++) {
				for (auto j = V + sigma; j < Vtilde; j++) {
					expr += x[i][j][s];
				}
			}

			name << "fd_" << s; // city visit
			fd[s] = IloRange(env, 1, expr, 1, name.str().c_str());
			name.str(""); expr.clear(); // Clean expr
		}
		model.add(fd);

		/* ----------== Equation (9) - Forbids cycles in z ==--------------- */
		for (auto i = 0; i < Vtilde - delta; ++i) {
			TT7[i] = IloRangeArray(env, Vtilde);
			for (auto j = sigma; j < Vtilde; ++j) {

				for (auto s = 0u; s < m; ++s) {
					expr += (z[i][j][s] + z[j][i][s]);
				}

				name << "TT7_" << i << "_" << j;

				TT7[i][j] = IloRange(env, 0, expr, g.APT()[i], name.str().c_str());
				name.str(""); expr.clear(); // Clean name		 
			}
			model.add(TT7[i]);
		}

		/* OPTIONAL ----------== Equation (10) - force the same robot to do multiple tasks ==--------------- */
		/* Matrix U must be defined in "noThriftGraph.cpp */
		//for (auto i = 0; i < Vtilde - delta; ++i) {
		//	sameRobot[i] = IloRangeArray(env, Vtilde);
		//	for (auto j = sigma; j < Vtilde; ++j) {

		//		if (g.U()[i][j] == 1) {
		//			for (auto s = 0u; s < m; ++s) {
		//				expr += (z[i][j][s] + z[j][i][s]);
		//			}

		//			name << "sameRobot_" << i << "_" << j;

		//			sameRobot[i][j] = IloRange(env, g.APT()[i], expr, g.APT()[i], name.str().c_str());
		//			name.str(""); expr.clear(); // Clean name	
		//			model.add(sameRobot[i][j]);
		//		}
		//	}	
		//}

		/* ----------== Equation (11) - Ensures transitive property on z ==--------------- */
		for (auto i = 0; i < Vtilde - delta; ++i) {
			pc3[i] = IloArray<IloArray<IloRangeArray>>(env, Vtilde);
			for (auto j = sigma; j < Vtilde; ++j) {
				pc3[i][j] = IloArray<IloRangeArray>(env, Vtilde);
				for (auto k = sigma; k < Vtilde - delta; ++k) {
					pc3[i][j][k] = IloRangeArray(env, Vtilde);
					for (auto s = 0; s < m; s++) {
						if (i != j && j != k && i != k) {

							expr = z[i][k][s] + z[k][j][s] - z[i][j][s];

							name << "pc3_" << i << "_" << j << "_" << k << "_" << s;
							pc3[i][j][k][s] = IloRange(env, -1, expr, 1, name.str().c_str());
							name.str(""); // Clean name
							expr.clear(); expr2.clear(); expr3.clear(); expr4.clear(); // Clean expressions
						}
					}
					model.add(pc3[i][j][k]);
				}
			}
		}

		/* ----------== Equation (12) - Cross-Schedule Precedence Constraint ==--------------- */
		for (auto i = 0; i < Vtilde - delta; ++i) {
			TT[i] = IloRangeArray(env, Vtilde);
			for (auto j = sigma; j < Vtilde; ++j) {

				if (g.P()[i][j] == 1) {

					for (auto s = 0; s < m; s++) {
						expr += z[i][j][s];
					}

					name << "xPC0_" << i << "_" << j; 
					name2 << "xPC1_" << i << "_" << j; 

					model.add(IloIfThen(env, (expr == 0), (T[j] - T[i] >= g.cd(i)), name.str().c_str()));
					model.add(IloIfThen(env, (expr >= 1), (T[j] - T[i] >= g.cd(i) + g.cost(i, j, 0)), name2.str().c_str()));

					name.str(""); name2.str(""); // Clean name
					expr.clear(); // Clean expressions
				}
			}
			model.add(TT[i]);
		}

		/* ----------== Equation (13 & 14) - Disjunctive Constraint ==--------------- */
		for (auto i = 0; i < Vtilde - delta; ++i) {
			for (auto j = sigma; j < Vtilde; ++j) {
				if (i != j) {

					for (auto s = 0u; s < m; ++s) {
						expr += z[i][j][s];
					}

					name << "z0_" << i << "_" << j; 
					name2 << "z1_" << i << "_" << j; 

					model.add(IloIfThen(env, (expr == 0), ((T[j] - T[i]) >= (g.cd(j) * (1 - g.R()[i][j])) - M), name.str().c_str()));
					model.add(IloIfThen(env, (expr >= 1), (T[j] - T[i] >= (g.cost(i, j, 0) + g.cd(i)) * (1 - g.R()[i][j])), name2.str().c_str()));

					name.str(""); name2.str(""); // Clean name
					expr.clear(); // Clean expressions
				}
			}
		}


#pragma region -- Objective Function --

		cout << "--\\-- Modeling Objective Function --//--" << endl;
		IloRangeArray dd2(env, Vtilde); // Array of tasks {0,inf}

		// minMax, using Q
		IloNumVar Q(env, 0, INFINITY, IloNumVar::Int, "Q");
		model.add(Q);

		for (auto i = sigma + V; i < Vtilde; ++i) {

			expr = Q - T[i];

			name << "dd_" << i;
			dd2[i] = IloRange(env, 0, expr, INFINITY, name.str().c_str());
			name.str(""); // Clean name
			expr.clear(); // Clean expr
		}
		model.add(dd2);
		IloObjective obj(env, Q, IloObjective::Minimize);

		// Add the objective function to the model
		model.add(obj);

#pragma endregion

		try {
			IloCplex cplex(model);
		}
		catch (const IloException& e) {
			std::cerr << std::endl << std::endl;
			std::cerr << "CPLEX Raised an exception:" << std::endl;
			std::cerr << e << std::endl;
			env.end();
			throw;
		}

		IloCplex cplex(model);

		std::cout << "Done!" << endl;
		std::cout << endl;
		printf("Model created in: %.2fs\n", (double)(clock() - tmpTime) / CLOCKS_PER_SEC);
		std::cout << endl;
		std::cout << "Starting solver... " << endl;
		std::cout << endl;

		// Export model to file (useful for debugging!)
		cplex.exportModel("model.lp");

		vector<vector<vector<uint32_t>>> subtours;
		bool solved = false;
		double callbackTime = 0;

#pragma region -- SOLVER -- 

		clock_t tmpTimeaa = clock();

		//CPLEX parameters
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, 1);
		cplex.setParam(IloCplex::Param::Threads, g.nThreads);
		cplex.setParam(IloCplex::Param::Parallel, 1);
		cplex.setParam(IloCplex::Param::Advance, 1);
		cplex.setParam(IloCplex::Param::Emphasis::MIP, 5);
		cplex.setParam(IloCplex::Param::Preprocessing::Symmetry, 5);
		cplex.setParam(IloCplex::Param::TimeLimit, 3600);
		cplex.setParam(IloCplex::Param::ClockType, 2);

		IloNum lastObjVal = IloInfinity;

		IloIntVarArray vars(env);
		IloNumArray vals(env);

		long timestep = clock();
		cplex.use(MIPInfoCallback(env, vals, vars, g, lastObjVal, timestep, g.path));

		// Try to solve CPLEX (and hope it does not raise an exception!)
		tmpTimeaa = clock();
		solved = cplex.solve();

		if (solved) {
			// If CPLEX successfully solved the model, print the results

			string path = g.path + "IncumbentSolutions.txt";

			ofstream myfile(path, ios_base::app | ios_base::out);

			myfile << cplex.getObjValue() << "\t" << cplex.getMIPRelativeGap() << "\t" << (double)(clock() - tmpTimeaa) / CLOCKS_PER_SEC << endl;
			myfile.close();

			std::cout << std::endl << std::endl;
			std::cout << "Cplex success!" << std::endl;
			std::cout << "\tStatus: " << cplex.getStatus() << std::endl;
			std::cout << "\tObjective value: " << cplex.getObjValue() << std::endl;

			vector<uint32_t> oneCount;
			oneCount.resize(sigma);
			vector<vector<uint32_t>> salesmen;
			salesmen.resize(sigma);
			cout << endl;
			std::cout << "X: " << endl;
			std::cout << endl;
			for (auto s = 0u; s < sigma; s++) {
				for (auto i = 0u; i < Vtilde; ++i) {
					/*if (i == 0) {
					cout << "  " << flush;
					for (auto j = 0; j < n; j++) {
					cout << setw(3) << j << flush;
					}
					cout << endl;
					}*/
					assert(x[i].getSize() == n);
					bool flagBreak = false;
					cout << setw(2) << i << flush;
					for (auto j = 0u; j < Vtilde; ++j) {
						//If variable x[i][j] is 1, the arc (i,j) was included in the optimal solution
						if (almost_equal(cplex.getValue(x[i][j][s]), 1)) {
							oneCount[s]++;
						}
						cout << setw(3) << static_cast<int>(round(cplex.getValue(x[i][j][s]))) << flush;
						//cout << setw(3) << (cplex.getValue(x[i][j][s])) << flush;

					}
					cout << endl;
				}
				cout << endl;
				cout << endl;
			}

			for (auto s = 0u; s < sigma; s++) {
				for (auto i = 0; i < Vtilde; ++i) {
					for (auto j = 0; j < Vtilde; ++j) {

						if (almost_equal(cplex.getValue(x[i][j][s]), 1)) {

							salesmen[s].emplace_back(i);
							salesmen[s].emplace_back(j);

						}

					}
				}
			}


			subtours.resize(sigma);


			for (auto i = 0u; i < sigma; i++) {

				uint32_t j = 0;
				uint32_t counter = 0, count = 0;
				uint32_t city = 0;
				vector<uint32_t> subtemp;
				vector<int> index;
				vector<uint32_t>  v;


				auto salestemp = salesmen[i];


				sort(salestemp.begin(), salestemp.end());
				auto last = unique(salestemp.begin(), salestemp.end());
				salestemp.erase(last, salestemp.end());

				while (count <= oneCount[i]) {
					//cout << "i " << i << " j " << j << endl;
					subtours[i].resize(counter + 1);

					if (subtours[i][counter].empty()) {
						subtours[i][counter].emplace_back(salesmen[i][city]);
						city++;
						subtours[i][counter].emplace_back(salesmen[i][city]);
					}
					else {
						index = findElements(salesmen[i], city);
						if (!index.empty()) {
							auto it = find(subtours[i][counter].begin(), subtours[i][counter].end(), salesmen[i][index.back() + 1]);
							if (it != subtours[i][counter].end()) {
								index.clear();
							}
						}
					}


					if (index.empty() && j != 0) {
						count += subtours[i][counter].size();
						counter++;

						copy(subtours[i][counter - 1].begin(), subtours[i][counter - 1].end(), back_inserter(subtemp));

						sort(subtemp.begin(), subtemp.end());

						set_intersection(salestemp.begin(), salestemp.end(), subtemp.begin(), subtemp.end(), back_inserter(v));
						copy(v.begin(), v.end(), back_inserter(subtemp));
						vector<uint32_t>  v;
						sort(subtemp.begin(), subtemp.end());
						auto last = unique(subtemp.begin(), subtemp.end());
						subtemp.erase(last, subtemp.end());

						set_difference(salestemp.begin(), salestemp.end(), subtemp.begin(), subtemp.end(), back_inserter(v));

						if (!v.empty()) {
							auto it = find(salesmen[i].begin(), salesmen[i].end(), v[0]);
							auto ind = distance(salesmen[i].begin(), it);
							if (*it == *(it + 1))
								city = ind + 1;
							else
								city = ind;
						}
						j = -1;

					}

					else if (j != 0) {
						if (index.size() == 1) {

							city = index.back() + 1;
							subtours[i][counter].emplace_back(salesmen[i][city]);

						}
						else {
							cout << "duplicate cities" << endl;
						}

					}

					j++;
				}
			}

			int N = 50;

			for (auto i = 0u; i < subtours.size(); i++) {

				cout << "Salesman " << i << ": " << endl;

				for (auto j = 0u; j < subtours[i].size(); j++) {

					auto sortList = subtours[i][j];

					cout << "Task " << sortList.back() << "|" << flush;
					auto startValue = cplex.getValue(T[sortList.back()]);
					for (auto b = 0; b < round(startValue / N); b++) {
						cout << " " << flush;
					}
					cout << "*" << flush;

					for (auto k = subtours[i][j].size() - 2; k > 0; k--) {
						cout << endl;
						cout << "Task " << sortList[k] << "|" << flush;
						auto startValue = cplex.getValue(T[sortList[k]]);

						for (auto b = 0; b < ceil(startValue / N); b++) {
							cout << " " << flush;
						}

						auto dur = g.cd(sortList[k]);

						for (auto b = 0; b < ceil(dur / N); b++) {
							cout << "=" << flush;
						}
					}


					cout << endl;
				}
				cout << endl;
				cout << endl;
			}

			cout << endl;
			cout << endl;

			cout << endl;
			std::cout << "T: " << endl;
			std::cout << endl;

			for (auto i = 0u; i < Vtilde; ++i) {
				/*if (i == 0) {
				cout << "  " << flush;
				for (auto j = 0; j < n; j++) {
				cout << setw(3) << j << flush;
				}
				cout << endl;
				}*/
				//assert(T[i].getSize() == n);
				bool flagBreak = false;
				cout << setw(2) << i << flush;


				cout << setw(12) << cplex.getValue(T[i]) << flush;
				//cout << setw(3) << (cplex.getValue(x[i][j][s])) << flush;


				cout << endl;
			}

			return subtours;
		}
		else {
			std::cerr << "Cplex error!" << std::endl;
			std::cerr << "\tStatus: " << cplex.getStatus() << std::endl;
			std::cerr << "\tSolver status: " << cplex.getCplexStatus() << std::endl;
		}

		env.end();

#pragma endregion

	}
}


