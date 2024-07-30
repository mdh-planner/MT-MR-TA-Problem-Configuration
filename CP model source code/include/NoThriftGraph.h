#pragma once
#ifndef GRAPH_H
#define GRAPH_H
#include "NoThriftModel.h"
#include <vector>
#include <iostream>

using namespace std;

namespace cplex_solver {
	class Graph {

		uint32_t n_nodes; /* Number of nodes in the graph. Nodes go from 0 to n_nodes - 1.*/
		int mtsp; /* Number of Salesmen */
		int sdepot; /* Number of Source Depot */
		int ddepot; /* Number of Destination Depot */
		vector<int> spsd; /* Number od Salesmen per Source Depot */
		vector<int> cc; /* City Color */
		vector<vector<int>> sc; /* Salesmen Color */
		vector<int> cDur; /* City Duration */
		vector<vector<vector<int>>> costs; /* Arc costs matrix. */
		vector<vector<vector<int>>> a; /* shows openness of cities towards salespersons */
		vector<vector<int>> p; /* Precedence Matrix p */
		vector<int> pv; /* precedence vector pv */
		vector<int> vt; /* virtual task vector pt */
		vector<int> pnl; /* Physical Task with No Location Vector */
		vector<vector<int>> r; /* concurrency matrix r */
		vector<int> apt; /* agents per task vector apt */

	public:

		/* Creats a new (random) graph from a model. */
		explicit Graph(MODEL& model);

		int nThreads; /* Number of threads to be used in the optimization process */
		bool useMIPstart; /*Use warm start*/
		int model; /*Choosing a model*/
		int tuneTime; /*Tuning Time*/
		int Emphasis; /*Search Emphasis*/
		string path;/*Path String */
		double pVirtual; /* Virtual task probability */
		double pParallel; /* Parallel task probability */

		vector<vector<int>> _agentsPerTasks;
		vector<vector<int>> _tasksPerAgents;

		uint32_t size() const { return n_nodes; } /* Size of the graph (i.e. number of nodes). */
		double cost(uint32_t i, uint32_t j, uint32_t s) const { return costs[i][j][s]; } /* Cost of arc (i,j,s). */
		int SD() const { return sdepot; } /* Returns the number of source depots*/
		int DD() const { return ddepot; } /* Returns the number of destination depots*/
		vector<int> SDSP() const { return spsd; } /* Returns a vector containig number of salesmen per source depot, size of the vector is equal to the number of source depots */
		vector<vector<int>> scolor() const { return sc; } /* Returns a 2D - vector containing Salesmen Colors */
		vector<int> ccolor() const { return cc; } /* Return a vector containing Cities Color */
		double cd(uint32_t i) const { return cDur[i]; } /* Returns  City duration */
		vector<vector<vector<int>>> A() const { return a; } /* Returns Tenzor A that shows openness of cities towards salespersons */
		vector<vector<int>> P() const { return p; } /*Returns Precedence Matrix P */
		vector<int> PV() const { return pv; } /* Returns Precedence vector pv */
		vector<int> VT() const { return vt; } /* Returns virtual tasks vector pt */
		vector<int> PnL() const { return pnl; } /* Returns physical tasks with no location vector PNL */
		vector<vector<int>> R() const { return r; } /* Returns parallel task Matrix P */
		vector<int> APT() const { return apt; } /* Returns parallel task Matrix P */
		int nSalesmen() const { return mtsp; } /* Returns the number of salesmen */

		friend class MODEL2;
	};
}

#endif