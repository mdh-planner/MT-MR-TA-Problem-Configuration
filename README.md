# MT-MR-TA Problem Configuration

We are providing both the source code and the compiled code for Windows (tested on win10).

The source code can be compiled easily in Visual Studio 2019 and C++14.
In addition what is need is to install cplex and cp optimizer on you local machine and properly link the libraries.

The code is setup to re-create 30 instances from the paper and solve them. 
This can be changed in problemGenerator.cpp. 

CP (MT-MR-TA) is a zip file containing CP implementation. Definition of variables and constraints can be found in
CP_Model.cpp. We tried to comment the constrains so they match the ones in the paper. In a similar manner, in ILP (MT-MR-TA) zip file,
ILP implementation can be found. Definition of variables and constraints can be found in solver_mr_mt.cpp.

The compiled version you can run by simply starting the *.exe file and entering the number from 0-29 (the 30 instances in the paper, the counting starts from 0,
so 0 maps to instance 1 and so on). 

To successfully use a test instance, you need to use three matching files (tasks,agents,weights). For example tasks_2_8.txt, agents_2_8.txt, and weights_2_8.txt are one instance.


# Test Instances guide
Agents file explanation

1st column - Agent's index

2nd column - Agent's equipment

Tasks file explanation

1st column - Task's index

2nd column - The number of agents required for the task

3rd column - Task's required equipment

4st column - 1 if the task is virtual (no location), 0 if it is physical

5th column - Precedence relations, the index of the task from the 1st column precedes the task that is here

6th column - Task duration

7th column - List of tasks that can run in parallel

Keep in mind that it works both ways, although it is not explicitly stated here. For example, if task 3 can run in parallel with task 7, that means that task 7 can run in parallel with task 3 even though in 7th column of task 7 there might not be task 3.

-1 is just int that fills in the empty space.

Weights file explanation

It is an adjacency matrix. Rows go as following: First m rows are agents. (every file has "name_m_n_.txt". The first number (m) is the number of agents in that instance, the second number (n) is the number of tasks in that instance) After m rows for agents, the next n rows are tasks. Lastly, the remaining rows represent the destination depots.

Note that virtual tasks have value 0 when traveling to or from, it means you don't need to travel to execute them.

If you have any questions you can contact me via Email branko.miloradovic@mdu.se
