Background:
this project try to modify CBS algorithm to reduce the CPU time and memory required for operation.

Environment:
python3 version

Usage；
using following command get the output of all the test instances:
$ python run_experiments.py --instance test/* --solver CBS --batch

using following command get the output in 5.4 (Compare different number of agents case for BCBS and ECBS.)
$ python run_experiments.py --instance test/largeTest* --solver CBS --batch

cbs.py and single_agent_planner.py
It contain the main modification of CBS.
In cbs.py, it implement the high level search.
In single_agent_planner.py, it implement the low level search.