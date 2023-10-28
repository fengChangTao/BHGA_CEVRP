
#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <iostream>
#include <string>
#include <climits>
#include "AlgorithmParameters.h"
using namespace std;
class CommandLine
{
public:
	AlgorithmParameters ap = default_algorithm_parameters();	// 算法参数

	int nbVeh		 = INT_MAX;		// 车的数量. 默认值: 无穷
	std::string pathInstance;		// vrp实例的路径
    std::string evrpInstance;		// evrp实例的路径
	std::string pathSolution;		// 解的路径
	bool verbose     = true; 		// 日志的详细级别
	bool isRoundingInteger = true;	// 是否四舍五入

	// 读取命令行并提取可用选项
	CommandLine(int argc, char* argv[])
	{
		if (argc % 2 != 1 || argc > 35 || argc < 3)
		{
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			display_help(); throw std::string("Incorrect line of command");
		}
		else
		{
			pathInstance = std::string(argv[1]);
            
            evrpInstance= pathInstance.substr(0, pathInstance.length() - 4) + ".evrp";
			pathSolution = std::string(argv[2]);
			for (int i = 3; i < argc; i += 2)
			{
				if (std::string(argv[i]) == "-t")
					ap.timeLimit = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-it")
					ap.nbIter  = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-seed")
					ap.seed    = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-veh")
					nbVeh = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-round")
					isRoundingInteger = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-log")
					verbose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbGranular")
					ap.nbGranular = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-mu")
					ap.mu = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-lambda")
					ap.lambda = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbElite")
					ap.nbElite = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbClose")
					ap.nbClose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterPenaltyManagement")
					ap.nbIterPenaltyManagement = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterTraces")
					ap.nbIterTraces = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-targetFeasible")
					ap.targetFeasible = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyIncrease")
					ap.penaltyIncrease = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyDecrease")
					ap.penaltyDecrease = atof(argv[i+1]);
				else
				{
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					display_help(); throw std::string("Incorrect line of command");
				}
			}
		}
	}


// 参数说明：
// [-it <int>]：设置在没有进步的情况下的最大迭代次数。默认为20,000。
// [-t <double>]：设定时间限制（单位为秒）。如果设置此参数，代码将持续迭代直至达到时间限制。
// [-seed <int>]：设定一个固定的随机数种子。默认为0。
// [-veh <int>]：设置预设的车队规模。如果不设置，会计算一个合理的车队规模上界。
// [-round <bool>]：是否将距离四舍五入至最接近的整数。可选0（不四舍五入）或1（四舍五入）。默认为1。
// [-log <bool>]：设置算法日志的详细级别。可以是0或1。默认为1。
// 附加参数：
// [-nbIterTraces <int>]：在HGS执行期间，两次显示轨迹之间的迭代次数。默认为500。
// [-nbGranular <int>]：粒度搜索参数，限制RI局部搜索中的移动次数。默认为20。
// [-mu <int>]：最小种群规模。默认为25。
// [-lambda <int>]：在达到最大种群规模之前所创建的解决方案数量（即，一代的规模）。默认为40。
// [-nbElite <int>]：精英个体数量。默认为5。
// [-nbClose <int>]：在计算多样性贡献时，考虑的最近解决方案/个体的数量。默认为4。
// [-nbIterPenaltyManagement <int>]：两次更新罚分之间的迭代次数。默认为100。
// [-targetFeasible <double>]：在两次罚分更新之间，可行个体的目标比率。默认为0.2。
// [-penaltyIncrease <double>]：在两次罚分更新之间，如果可行个体数量不足，罚分会增加的比率。默认为1.2。
// [-penaltyDecrease <double>]：在两次罚分更新之间，如果可行个体数量足够，罚分会减少的比率。默认为0.85。

	// 输出使用说明
	void display_help()
	{
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- HGS-CVRP algorithm (2020) ---------------------------------------------------" << std::endl;
		std::cout << "Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-seed mySeed] [-veh nbVehicles] [-log verbose]               " << std::endl;
		std::cout << "[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                         " << std::endl;
		std::cout << "[-t <double>] sets a time limit in seconds. If this parameter is set the code will be run iteratively until the time limit      " << std::endl;
		std::cout << "[-seed <int>] sets a fixed seed. Defaults to 0                                                                                  " << std::endl;
		std::cout << "[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the the fleet size is calculated                        " << std::endl;
		std::cout << "[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. " << std::endl;
		std::cout << "[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                     " << std::endl;
		std::cout << std::endl;
		std::cout << "Additional Arguments:                                                                                                           " << std::endl;
		std::cout << "[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500                         " << std::endl;
		std::cout << "[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20                " << std::endl;
		std::cout << "[-mu <int>] Minimum population size. Defaults to 25                                                                             " << std::endl;
		std::cout << "[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40 " << std::endl;
		std::cout << "[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                     " << std::endl;
		std::cout << "[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4      " << std::endl;
		std::cout << "[-nbIterPenaltyManagement <int>] Number of iterations between penalty updates. Defaults to 100                                  " << std::endl;
		std::cout << "[-targetFeasible <double>] target ratio of feasible individuals between penalty updates. Defaults to 0.2                        " << std::endl;
		std::cout << "[-penaltyIncrease <double>] penalty increase if insufficient feasible individuals between penalty updates. Defaults to 1.2      " << std::endl;
		std::cout << "[-penaltyDecrease <double>] penalty decrease if sufficient feasible individuals between penalty updates. Defaults to 0.85       " << std::endl;
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
};
#endif
