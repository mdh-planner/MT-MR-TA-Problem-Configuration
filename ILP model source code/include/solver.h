#ifndef SOLVER_H
#define SOLVER_H

#include "NoThriftGraph.h"
#include <iostream>

// Magic tricks to have CPLEX behave well:
#ifndef IL_STD
#define IL_STD
#endif
#include <cstring>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
// End magic tricks

namespace cplex_solver {
  class Solver {
    // The graph on which we are solving the TSP.
    const Graph& g;
  public:
    
    // Builds a solver for graph g.
    explicit Solver(const Graph& g) : g{g} {}
	
    // Solves the TSP with CPLEX and prints the result.
	vector<vector<vector<uint32_t>>> solve_and_print() const;
	~Solver() { cout << "Solver object has been destroyed" << endl; }; // destructor
  };
}

#endif