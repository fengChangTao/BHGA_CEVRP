#include "case.h"
#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
using namespace std;




int main(int argc, char *argv[])
{
	try
	{
		// 读取程序参数
		CommandLine commandline(argc, argv);

		// 输出所有算法参数值，如果日志为“详细”
		if (commandline.verbose) print_algorithm_parameters(commandline.ap);

		// 读取数据文件和初始化一些数据结构
		if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		// 读取文件
		InstanceCVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);
		// 问题参数+算法参数
		Params params(cvrp.x_coords,cvrp.y_coords,cvrp.dist_mtx,cvrp.service_time,cvrp.demands,
			          cvrp.vehicleCapacity,cvrp.durationLimit,commandline.nbVeh,cvrp.isDurationConstraint,commandline.verbose,commandline.ap);
		// 补充，将evrp实例定义进去
        params.c_evrp=Case(commandline.evrpInstance,2);
		params.s1=commandline.pathSolution;
		// 运行 HGS
		Genetic solver(params);
		solver.run();
		
		// 输出最优结果
		if (solver.population.getBestFound() != NULL)
		{
			if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
			solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),commandline.pathSolution);
			solver.population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance);
		}
	}
	catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
	return 0;
}
