#pragma once
#include <vector>

//#define THRIFT

struct Source {
	double x, y;
};

struct Destination {
	double x, y;
};

struct Task {
	double x, y, duration;
	int reqA, color, prec, virt, index;
	std::vector<int> para;

};

struct Agent {
	double x, y, v;
	std::vector<int> color;
	//std::vector<std::vector<float>> performance; // Performance of each agent per task

};

struct gantt {
	int startTime, endTime, agentIdx, virtualTask, taskIdx;
};

struct Problem {
	std::vector<Source> src;
	std::vector<Destination> dest;
	std::vector<Task> T;
	std::vector<Agent> A;
	std::vector<double> seed;
	std::vector<int> APT;
	double pV; double pP;
};


#pragma once
