#include "definitions.h"
#include "randnumgenFixed.h"
#include <numeric>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string_view>
#include <vector>

using namespace std;

void pushVirtual(Problem& object);
void pushParallel(Problem& object);
int selectNumTask(vector<double>& probV);
int findMaxAgentNumber(int color, Problem& object);

vector<double> parallelTasksProbability(vector<int>& paratemp);

vector<Problem> problemGenerator() {

	Problem object;

	

	/* Max number of Colors */

	constexpr auto PHYSICAL_COLOR_NUMBER = 2;
	constexpr auto VIRTUAL_COLOR_NUMBER = 2;
	int COLOR_NUMBER = PHYSICAL_COLOR_NUMBER + VIRTUAL_COLOR_NUMBER;
	std::vector<int> phy_colors(PHYSICAL_COLOR_NUMBER), virt_colors(VIRTUAL_COLOR_NUMBER), tempPara;;
	std::iota(phy_colors.begin(), phy_colors.end(), 0);
	std::iota(virt_colors.begin(), virt_colors.end(), PHYSICAL_COLOR_NUMBER);

	/* Probabilities for tasks having precedence constraint "pcProb", for task being eligable for parallel execution "paraProb", and for task being virtual "virtProb" */
	constexpr auto pcProb = 0;

	double virtProb = 0.15;
	//cout << "Enter % of Virtual Tasks (0.0 - 1.0):  " << flush;
	//cin >> virtProb;

	double paraProb = 0.5;
	//cout << "Enter % of Parallel Tasks (0.1 - 0.9):  " << flush;
	//cin >> paraProb;

	object.pP = paraProb;	object.pV = virtProb;

	///* Data Set; This can be read from a file as well */
	//vector<int> SourceDepots = { 1,1,2,2,3,3,4,4,5,5 };
	//vector<int> DestinationDepots = { 1,1,2,2,3,3,4,4,5,5 };
	//vector<int> Agents = { 2,2,3,3,3,3,4,4,5,6 };
	//vector<int> Tasks = { 8,10,12,14,16,18,20,25,30, 35 };
		
	/*vector<int> SourceDepots = { 1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5 };
	vector<int> DestinationDepots = { 1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5 };
	vector<int> Agents = { 2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6 };
	vector<int> Tasks = { 8,10,12,14,14,16,18,20,20,22,24,26,26,28,30,32,32,34,36,40};*/


	//vector<int> SourceDepots = { 1,1,2,2,3,3,3,3,3,3 };
	//vector<int> DestinationDepots = { 1,1,1,2,2,2,3,3,3,4 };
	//vector<int> Agents = { 2,2,2,3,3,3,4,4,4,4 };
	//vector<int> Tasks = { 6,7,8,8,9,10,10,11,13,15 };

	/* Data Set ISRR 2024 */
	vector<int> SourceDepots = { 1,1,1,1,1,1,1,1,1,1,
								 2,2,2,2,2,2,2,2,2,2,
								 3,3,3,3,3,3,3,3,3,3 };
	vector<int> DestinationDepots = { 1,1,1,1,1,1,1,1,1,1,
								 2,2,2,2,2,2,2,2,2,2,
								 3,3,3,3,3,3,3,3,3,3 };
	vector<int> Agents = { 2,2,2,2,2,2,
						   3,3,3,3,3,3,
						   4,4,4,4,4,4,
						   5,5,5,5,5,5,
						   6,6,6,6,6,6 };
	vector<int> Tasks = { 6,6,7,7,8,8,
						  8,8,10,10,12,12,
						  10,10,12,12,14,14,
						  12,12,14,14,16,16,
						  14,14,16,16,18,18 };


	/* Number of Test Instances*/
	int N = Tasks.size();
	vector<Problem> _return(N);

	/* Navigation Area x,y [meters] */
	double xMin = 0, xMax = 200, yMin = 0, yMax = 200;

	/*Task duration min max [seconds] */
	double taskMin = 10, taskMax = 50;

	/* Max velocity of an agent */
	double velMax = 5.0;

	/* Instance Generation Loop*/
	for (int inst = 0; inst < N; inst++) {


		//Initialize vectors
		object.src.resize(SourceDepots[inst]);
		object.dest.resize(DestinationDepots[inst]);
		object.A.resize(Agents[inst]);
		object.T.resize(Tasks[inst]);

		/* Create Source Depots with randomized X and Y coordiantes */
		for (int sdepot = 0; sdepot < SourceDepots[inst]; sdepot++) {
			object.src[sdepot].x = getRandomRealInRangeF(xMin, xMax);
			object.src[sdepot].y = getRandomRealInRangeF(yMin, yMax);
		}

		/* Create Destination Depots with randomized X and Y coordiantes */
		for (int ddepot = 0; ddepot < DestinationDepots[inst]; ddepot++) {
			object.dest[ddepot].x = getRandomRealInRangeF(xMin, xMax);
			object.dest[ddepot].y = getRandomRealInRangeF(yMin, yMax);
		}

		/* Create Agents with randomized X and Y coordiantes and randomized colors */
		for (int agent = 0; agent < Agents[inst]; agent++) {
			auto phy_tempColors = phy_colors;
			auto virt_tempColors = virt_colors;

			auto rndSource = getRandomIntegerInRangeF(0, static_cast<int>(SourceDepots[inst] - 1));
			object.A[agent].x = object.src[rndSource].x;
			object.A[agent].y = object.src[rndSource].y;
			object.A[agent].v = 2.0;

			//physical colors
			int phy_rnd = getRandomIntegerInRangeF(1, PHYSICAL_COLOR_NUMBER);
			int virt_rnd = getRandomIntegerInRangeF(1, VIRTUAL_COLOR_NUMBER);
			object.A[agent].color.resize(phy_rnd + virt_rnd);

			for (int i = 0; i < phy_rnd; i++) {
				int tmp = phy_tempColors.size() - 1;
				auto rnd2 = getRandomIntegerInRangeF(0, tmp);
				object.A[agent].color[i] = phy_tempColors[rnd2];
				phy_tempColors.erase(phy_tempColors.begin() + rnd2);
			}

			for (int i = 0; i < virt_rnd; i++) {
				int tmp = virt_tempColors.size() - 1;
				auto rnd2 = getRandomIntegerInRangeF(0, tmp);
				object.A[agent].color[i + phy_rnd] = virt_tempColors[rnd2];
				virt_tempColors.erase(virt_tempColors.begin() + rnd2);
			}

		}

		//in order to make sure at least 2 agents share the same color
		auto r = getRandomIntegerInRangeF(0, Agents[inst] - 1);
		auto rnd = getRandomIntegerInRangeExcludingF(0, Agents[inst] - 1, r);

		object.A[rnd].color.emplace_back(object.A[r].color[0]);
		sort(object.A[rnd].color.begin(), object.A[rnd].color.end());
		auto last = std::unique(object.A[rnd].color.begin(), object.A[rnd].color.end());
		// v now holds {1 2 1 3 4 5 4 x x x}, where 'x' is indeterminate
		object.A[rnd].color.erase(last, object.A[rnd].color.end());

		/* Create Agents with randomized X and Y coordiantes and randomized colors */
		int cc = 0;
		bool virtualFlag = false;

		for (int task = 0; task < Tasks[inst]; task++) {
			object.T[task].x = getRandomRealInRangeF(xMin, xMax);
			object.T[task].y = getRandomRealInRangeF(yMin, yMax);
			object.T[task].duration = getRandomRealInRangeF(taskMin, taskMax);
			object.T[task].color = getRandomIntegerInRangeF(0, PHYSICAL_COLOR_NUMBER - 1);

			/* Find the maximum number of agents with certain color. If no agent is found, the color is added to one of the agents randomly */
			int MAX = findMaxAgentNumber(object.T[task].color, object);

			/* Randomly set the maximum number of agents required per task */
			object.T[task].reqA = getRandomIntegerInRangeF(1, MAX);

			// Probability of task being Virtual
			auto limit = getRandomRealInRangeF(0.0, 1.0);

			if (limit < virtProb) {
				virtualFlag = true;
				object.T[task].virt = 1;
				object.T[task].color = getRandomIntegerInRangeF(PHYSICAL_COLOR_NUMBER, COLOR_NUMBER - 1);;
				//tempPara.emplace_back(inst);
				//tempPara.emplace_back(task);
				tempPara.emplace_back(Tasks[inst] - cc - 1);
				cc++;
			}
			//
			//// Probability of having Precedence Constraint
			//limit = getRandomRealInRangeF(0.0, 1.0);

			//if (limit < pcProb) {
			//	object.T[task].prec = getRandomIntegerInRangeExcludingF(0, Tasks[inst] - 1, task);
			//}

		}
		if (!virtualFlag) {
			object.T.back().virt = 1;
			object.T.back().color = getRandomIntegerInRangeF(PHYSICAL_COLOR_NUMBER, COLOR_NUMBER - 1);;
			//tempPara.emplace_back(inst);
			//tempPara.emplace_back(task);
			tempPara.emplace_back(Tasks[inst] - cc - 1);
		}
		/* Push Virtual Tasks to the end of the Task array */
		pushVirtual(object);


		//vector<double> probV = parallelTasksProbability(tempPara);

		//int start = Tasks[inst] - round(Tasks[inst] * paraProb);

		//for (int task = start; task < Tasks[inst]; task++) {

		//	if (tempPara.size() > 1) {

		//		auto tempPara2 = tempPara;
		//		int rnd = selectNumTask(probV);

		//		// remove task -> it cannot be parallel with itself
		//		auto it = find(tempPara2.begin(), tempPara2.end(), task);
		//		if (it != tempPara2.end()) {
		//			tempPara2.erase(it);
		//		}

		//		//remove tasks that have PC relations, they cannot be parallel at the same time while one precedes the other
		//		it = find(tempPara2.begin(), tempPara2.end(), object.T[task].prec);
		//		if (it != tempPara2.end()) {
		//			tempPara2.erase(it);
		//		}

		//		for (int t = 0; t < object.T.size(); t++) {
		//			if (object.T[t].prec == task) {
		//				it = find(tempPara2.begin(), tempPara2.end(), t);
		//				if (it != tempPara2.end()) {
		//					tempPara2.erase(it);
		//				}
		//			}
		//		}

		//		for (int ij = 0; ij < rnd; ij++) {
		//			//cout << ij << endl;
		//			auto rnd2 = getRandomIntegerInRangeF(0, (int)tempPara2.size() - 1);

		//			object.T[task].para.emplace_back(tempPara2[rnd2]);

		//			auto it = find(tempPara2.begin(), tempPara2.end(), tempPara2[rnd2]);
		//			if (it != tempPara2.end()) {
		//				tempPara2.erase(it);
		//			}

		//		}
		//	}
		//}
		vector<int> taskList(Tasks[inst]);
		iota(taskList.begin(), taskList.end(), 0);
		//		cout << inst << endl;
		vector<double> probV = parallelTasksProbability(taskList);

		for (int task = 0; task < Tasks[inst]; task++) {

			object.T[task].index = task;
			// Probability of parallel Tasks
			auto limit = getRandomRealInRangeF(0.0, 1.0);

			if (limit < paraProb) {

				auto tempPara2 = tempPara;

				// remove task -> it cannot be parallel with itself
				auto it = find(tempPara2.begin(), tempPara2.end(), task);
				if (it != tempPara2.end()) {
					tempPara2.erase(it);
				}

				//remove tasks that have PC relations, they cannot be parallel at the same time while one precedes the other
				it = find(tempPara2.begin(), tempPara2.end(), object.T[task].prec);
				if (it != tempPara2.end()) {
					tempPara2.erase(it);
				}
				for (int t = 0; t < object.T.size(); t++) {
					if (object.T[t].prec == task) {
						it = find(tempPara2.begin(), tempPara2.end(), t);
						if (it != tempPara2.end()) {
							tempPara2.erase(it);
						}
					}
				}

				if (!tempPara2.empty()) {
					int rnd = getRandomIntegerInRangeF(0, (int)tempPara2.size() - 1); //bilo je pocinjalo od 1 ne od 0. ne znam zasto.
					object.T[task].para.emplace_back(tempPara2[rnd]);
				}
			}

		}

		pushParallel(object);

		std::vector<int> l(Tasks[inst]);
		std::iota(l.begin(), l.end(), 0);

		int N = object.T.size();

		/* Read Parallel Tasks */
		std::vector<std::vector<int>> R = vector<vector<int>>(N, vector<int>(N));
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < object.T[i].para.size(); j++) {
				//cout << i << " " << object.T[i].para[j] << endl;
				R[i][object.T[i].para[j]] = 1;
				R[object.T[i].para[j]][i] = 1;
			}

		}

		for (int task = 0; task < Tasks[inst]; task++) {
			object.T[task].prec = -1;
		}

		/*------------ If pcprob is enabled -------------*/
		if (pcProb > 0) {
			for (int task = 0; task < Tasks[inst]; task++) {
				// Probability of having Precedence Constraint
				auto limit = getRandomRealInRangeF(0.0, 1.0);

				if (limit < pcProb) {
					vector<int> tmpTasksForPrec;

					if (!object.T[task].para.empty()) {
						vector<int> jediGovna;
						for (int govno = 0; govno < R[task].size(); govno++) {
							if (R[task][govno] == 1) {
								jediGovna.emplace_back(govno - Agents[inst]);
							}
						}

						sort(jediGovna.begin(), jediGovna.end());
						set_difference(l.begin(), l.end(), jediGovna.begin(), jediGovna.end(), inserter(tmpTasksForPrec, tmpTasksForPrec.begin()));
					}
					else {
						tmpTasksForPrec = l;
					}

					object.T[task].prec = getRandomIntegerInRangeExcludingF(0, (int)(tmpTasksForPrec.size() - 1), task);
				}
				else {
					object.T[task].prec = -1;
				}

			}
		}

		/*-----------make every virtual task to precede a physical task ---------------*/
		//create a list of only physical tasks
		vector<int> vPhysicalTasks;
		for (int phy = 0; phy < Tasks[inst]; phy++) {

			if (object.T[phy].virt == 0) {
				vPhysicalTasks.push_back(phy);
			}
		}

		for (int task = 0; task < Tasks[inst]; task++) {
			// Probability of having Precedence Constraint
			if (object.T[task].virt == 1) {
				vector<int> tmpTasksForPrec;

				if (!object.T[task].para.empty()) {
					vector<int> jediGovna;
					for (int govno = 0; govno < R[task].size(); govno++) {
						if (R[task][govno] == 1) {
							jediGovna.emplace_back(govno - Agents[inst]);
						}
					}

					sort(jediGovna.begin(), jediGovna.end());
					set_difference(vPhysicalTasks.begin(), vPhysicalTasks.end(), jediGovna.begin(), jediGovna.end(), inserter(tmpTasksForPrec, tmpTasksForPrec.begin()));
				}
				else {
					tmpTasksForPrec = vPhysicalTasks;
				}
				int tmppc = tmpTasksForPrec[getRandomIntegerInRangeF(0, (int)(tmpTasksForPrec.size() - 1))];
				object.T[tmppc].prec = task;
				//cout << object.T[tmppc].prec << "\t" << task << "\t" << tmppc << endl;

			}
			else {
				//cout << object.T[task].prec << "\t" << task << "\t" << "else" << endl;
				object.T[task].prec = -1;
			}
		}

		///* Creating Vector of APT randomly */
		std::vector<std::vector<int>> tasks2agents(Tasks[inst]);
		for (int t = 0; t < Tasks[inst]; t++) {
			for (int i = 0; i < Agents[inst]; i++) {
				for (int j = 0; j < object.A[i].color.size(); j++) {
					if (object.T[t].color == object.A[i].color[j]) {
						tasks2agents[t].emplace_back(i);
						break;
					}
				}
			}
		}

		object.APT.resize(Tasks[inst]);
		//object.APT[0] = 3;
		/*for (int i = 0; i < Tasks[inst]; i++) {
			object.APT[i] = 3;
		}*/
		for (int i = 0; i < Tasks[inst]; i++) {

			int rnd = 0;
			if (object.T[i].virt == 0) {
				if (tasks2agents[i].size() > 1) {
					//float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					float r = getRandomRealInRangeF(0.0, 1.0);
					if (r > 0.5) {
						rnd = getRandomIntegerInRangeF(1, (int)tasks2agents[i].size() - 1);
						if (rnd > 2) {
							rnd = 2;
						}
						//rnd = 0;
					}
				}
			}
			object.APT[i] = 1 + rnd;
		}

		_return[inst] = object;
		object.src.clear();;
		object.dest.clear();
		object.A.clear();
		object.T.clear();
		tempPara.clear();
	}

	return _return;
}

int findMaxAgentNumber(int color, Problem& object) {
	int _return = 0;

	/* Count how many agents have the certain color */
	for (int i = 0; i < object.A.size(); i++) {
		for (int j = 0; j < object.A[i].color.size(); j++) {
			if (color == object.A[i].color[j]) {
				_return++;
				break;
			}
		}
	}

	/* A fix if no agent can perform the task */
	if (_return == 0) {
		int tmpA = object.A.size() - 1;
		int agentIdx = getRandomIntegerInRangeF(0, tmpA);
		object.A[agentIdx].color.emplace_back(color);
		return 1;
	}

	return _return;
}

void pushVirtual(Problem& object) {
	vector<Task> T(object.T.size());
	int _physical = 0;
	int _virtual = T.size() - 1;

	for (int i = 0; i < T.size(); i++) {
		if (object.T[i].virt == 0) {
			T[_physical] = object.T[i];
			_physical++;
		}
		else {
			T[_virtual] = object.T[i];
			_virtual--;
		}
	}

	object.T = T;

	return;
}

void pushParallel(Problem& object) {
	int N = object.T.size();

	///* Read Parallel Tasks */
	std::vector<std::vector<int>> R = vector<vector<int>>(N, vector<int>(N));
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < object.T[i].para.size(); j++) {
			//	cout << i << " " << object.T[i].para[j] << endl;
			R[i][object.T[i].para[j]] = 1;
			R[object.T[i].para[j]][i] = 1;
		}

	}

	vector<Task> T(N);
	int _notParallel = 0;
	int _Parallel = T.size() - 1;

	for (int i = 0; i < T.size(); i++) {

		int sum = accumulate(R[i].begin(), R[i].end(), 0);
		if (sum == 0 || object.T[i].virt == 0) {
			T[_notParallel] = object.T[i];

			_notParallel++;
		}
		else {
			T[_Parallel] = object.T[i];
			_Parallel--;
		}
	}

	for (int i = 0; i < T.size(); i++) {
		if (!T[i].para.empty()) {
			for (int j = 0; j < T[i].para.size(); j++) {
				if (T[T[i].para[j]].index == T[i].para[j]) {

				}
				else {
					for (int idx = 0; idx < T.size(); idx++) {
						//cout << i << "  " << j << "  " << idx << endl;
						if (T[idx].index == T[i].para[j]) {
							T[i].para[j] = idx;
							break;
						}
					}
				}
			}
		}
	}


	object.T = T;

	/* Read Parallel Tasks */

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < object.T[i].para.size(); j++) {
			//	cout << i << " " << object.T[i].para[j] << endl;
			R[i][object.T[i].para[j]] = 1;
			R[object.T[i].para[j]][i] = 1;
		}

	}
	return;
}

vector<double> parallelTasksProbability(vector<int>& paratemp) {

	vector<double> tmpP(paratemp.size());
	//double sum = (paratemp.size() + 1) * paratemp.size() / 2;
	//tmpP[0] = paratemp.size() / sum;
	//for (int i = paratemp.size() - 1; i > 0; i--) {
	//	tmpP[paratemp.size() - i] = tmpP[paratemp.size() - i - 1] + i / sum;
	//}



	for (int i = 0; i < paratemp.size(); i++) {
		tmpP[i] = 1.0 / paratemp.size();
	}

	return tmpP;

}

int selectNumTask(vector<double>& probV) {
	int binNum = 0;
	double p = getRandomRealInRangeF(0.0, 1.0);
	if (0 <= p && p < probV[binNum])
		return binNum + 1;

	for (; binNum < probV.size() - 1; ++binNum) {
		if (probV[binNum] <= p && p < probV[binNum + 1]) {
			return binNum + 1;
		}
	}

	return probV.size() - 1;

}
