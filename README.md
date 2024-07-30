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
