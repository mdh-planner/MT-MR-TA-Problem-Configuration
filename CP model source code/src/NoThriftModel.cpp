#include "NoThriftModel.h"
#include "randnumgenFixed.h"
#include <algorithm>

void MODEL::parseGenData(Problem context)
{
	pV = context.pV;
	pP = context.pP;
	mtsp = context.A.size();
	int V = context.T.size();
	nbCities = context.T.size() + context.dest.size() + context.A.size(); // I need to see if vehicles start from the same position or not.
	scolor.resize(mtsp);

	for (auto i = 0u; i < mtsp; ++i) {
		for (auto j = 0u; j < context.A[i].color.size(); j++) {
			scolor[i].emplace_back(context.A[i].color[j]);
		}
		spsd.emplace_back(1);
	}

	std::vector<std::vector<int>> tasks2agents(V);
	for (int t = 0; t < V; t++) {
		for (int i = 0; i < mtsp; i++) {
			for (int j = 0; j < scolor[i].size(); j++) {
				if (context.T[t].color == scolor[i][j]) {
					tasks2agents[t].emplace_back(i);
					break;
				}
			}
		}
	}

	vector<int> cccolor(nbCities, -1);

	for (auto i = 0u; i < context.T.size(); i++) {
		cccolor[i + context.A.size()] = context.T[i].color; // check this if it is always zero
	}

	ccolor = cccolor;

	sdepot = context.A.size();
	ddepot = context.dest.size();

	distanceWeight.resize(nbCities);
	R.resize(nbCities);

	vt.resize(nbCities);
	for (int i = 0; i < nbCities; i++) {
		for (int j = 0; j < nbCities; j++) {

			// Virtual tasks
			/*=========================*/
			if (i < sdepot || i >= (nbCities - ddepot)) {
				vt[i] = 0;
			}
			else {
				vt[i] = context.T[i - sdepot].virt;
			}
		}
	}

	for (int i = 0; i < nbCities; i++) {

		// Getting Task Duration from the benchmarkMission
		/*=========================*/
		if (i < sdepot || i >= (nbCities - ddepot)) {
			cityDur.emplace_back(0);
		}
		else {
			cityDur.emplace_back(context.T[i - sdepot].duration);
		}
		/*=========================*/

		distanceWeight[i].resize(nbCities);
		/* Read Parallel Tasks */
		R = vector<vector<int>>(nbCities, vector<int>(nbCities));
		for (int i = 0; i < nbCities; i++) {
			if (i >= sdepot && i < V + sdepot) {
				context.T[i - sdepot].index = i - sdepot;
				for (int j = 0; j < context.T[i - sdepot].para.size(); j++) {
					R[i][context.T[i - sdepot].para[j] + sdepot] = 1;
					R[context.T[i - sdepot].para[j] + sdepot][i] = 1;
				}
			}
		}

		/* Weight matrix */
		vector<pair<double, double>> Vtilde2;
		Vtilde2.resize(nbCities);

		for (int i = 0; i < Vtilde2.size(); i++) {
			if (i < sdepot) {
				Vtilde2[i].first = context.A[i].x;
				Vtilde2[i].second = context.A[i].y;
			}
			else if (i >= sdepot && i < sdepot + V) {
				Vtilde2[i].first = context.T[i - sdepot].x;
				Vtilde2[i].second = context.T[i - sdepot].y;
			}
			else {
				Vtilde2[i].first = context.dest[i - sdepot - V].x;
				Vtilde2[i].second = context.dest[i - sdepot - V].y;
			}
		}

		distanceWeight = vector<vector<vector<int>>>(Vtilde2.size(), vector<vector<int>>(Vtilde2.size(), vector<int>(sdepot)));

		for (int i = 0; i < Vtilde2.size(); i++) {
			for (int j = i; j < Vtilde2.size(); j++) {
				for (int s = 0; s < sdepot; s++) {

					if (j >= sdepot && j < sdepot + V && vt[j] == 1) {
						distanceWeight[i][j][s] = 0;
						distanceWeight[j][i][s] = 0;
					}
					else if (vt[i] == 1 && j >= V + sdepot) {
						distanceWeight[i][j][s] = 0;
						distanceWeight[j][i][s] = 0;
					}
					else {
						distanceWeight[i][j][s] = sqrt((Vtilde2[i].first - Vtilde2[j].first) * (Vtilde2[i].first - Vtilde2[j].first) + (Vtilde2[i].second - Vtilde2[j].second) * (Vtilde2[i].second - Vtilde2[j].second)) / context.A[s].v;
						distanceWeight[j][i][s] = distanceWeight[i][j][s];
					}
				}
			}
		}
	}
	
	/* Adding Precedence Constraints to the Model*/
	pv.resize(nbCities);

	for (int i = 0; i < context.T.size(); i++) {
		if (context.T[i].prec > 0) {
			pv[i + sdepot] = context.T[i].prec + sdepot; // CORRECT WAY
		}
	}

	P = vector<vector<int>>(nbCities, vector<int>(nbCities));

	for (int i = sdepot; i < V + sdepot; i++) {
		for (int j = sdepot; j < V + sdepot; j++) {
			if (context.T[i - sdepot].prec == (j - sdepot)) {
				P[i][j] = 1;
			}
		}
	}

	APT = context.APT;
}


