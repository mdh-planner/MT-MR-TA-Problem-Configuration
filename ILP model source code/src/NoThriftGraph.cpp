#include "NoThriftGraph.h"
#include <cmath>
#include <random>
#include <iterator>
#include <algorithm>

namespace cplex_solver {
	Graph::Graph(MODEL & model)
	{
		/* Number of salesman */
		mtsp = model.mtsp;

		pVirtual = model.pV;
		pParallel = model.pP;

		/* number of cities */
		n_nodes = model.nbCities;

		/* Number of Source Depot */
		sdepot = model.sdepot;

		/* Number of Destination Depot */
		ddepot = model.ddepot;

		/* Number od Salesmen per Source Depot */
		spsd = model.spsd;

		/* Salesmen Color */
		sc = model.scolor;

		/* City Color */
		cc = model.ccolor;

		/* precedence vector */
		pv = model.pv;

		/* City duration */
		cDur = model.cityDur;

		/* Virtual Tasks Vector */
		vt = model.vt;

		/* Concurrency Matrix */
		r = model.R;

		/* U Matrix */
		u = model.U;

		/*physical task with no location vector*/
		pnl = model.pnl;

		/* the number of agents per task required vector */
		apt = model.APT;

		int add = mtsp - 1;

		costs = model.distanceWeight;

		a = vector<vector<vector<int>>>(model.nbCities, vector<vector<int>>(model.nbCities, vector<int>(model.mtsp)));

		p = model.P;

	/* Color Matrix A */

		int lb = 0;
		int counter = 0;

		for (auto i = 0u; i < a.size(); ++i) {

			bool flag = true;

			if (i >= sdepot) {
				counter = mtsp;
				lb = 0;
			}
			else {
				if (i > 0) {
					lb += spsd[i - 1];
				}
				counter = lb + spsd[i];
			}

			for (auto j = 0; j < a[i].size(); ++j) {

				if (i == j) {
					continue;
				}

				for (auto s = lb; s < counter; ++s) {
					if (j < a[i].size() - ddepot) {

						bool colorFlag = false;

						if (i < sdepot) {

							auto it = find(sc[s].begin(), sc[s].end(), cc[j]);

							if (it != sc[s].end()) {
								colorFlag = true;
							}
						}
						else {
							auto it = find(sc[s].begin(), sc[s].end(), cc[i]);
							if (it != sc[s].end()) {
								auto it2 = find(sc[s].begin(), sc[s].end(), cc[j]);

								if (it2 != sc[s].end()) {
									colorFlag = true;
								}
							}
						}

						if (p[i][j] == 1 && colorFlag) {
							a[i][j][s] = 1;
							a[j][i][s] = 0;
						}
						if (p[i][j] == 0 && colorFlag) {
							a[i][j][s] = 1;
						}
					}
					else  if (i >= a[i].size() - ddepot && j >= a[i].size() - ddepot) {
						a[i][j][s] = 0;
					}
					else {
						a[i][j][s] = 1;
					}
				}
			}
		}

		_agentsPerTasks.resize(n_nodes);
		_tasksPerAgents.resize(mtsp);

		for (int i = 0; i < n_nodes; i++) {
			for (int j = 0; j < mtsp; j++) {
				for (int k = 0; k < sc[j].size(); k++) { 
					if (cc[i] == sc[j][k]) {
						_agentsPerTasks[i].emplace_back(j);
						break;
					}
				}
			}
		}

		for (int i = 0; i < mtsp; i++) {
			for (int j = 0; j < n_nodes; j++) {
				for (int k = 0; k < sc[i].size(); k++) {
					if (cc[j] == sc[i][k]) {
						_tasksPerAgents[i].emplace_back(j);
						break;
					}
				}
			}
		}
	}
}