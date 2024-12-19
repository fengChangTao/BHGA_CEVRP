#include "Individual.h" 


void Individual::evaluateCompleteCost(Params & params)
{
	eval = EvalIndiv();
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			double distance = params.timeCost[0][chromR[r][0]];// 已走距离
			double load = params.cli[chromR[r][0]].demand;// 负载
			double service = params.cli[chromR[r][0]].serviceDuration;// 服务时间
			predecessors[chromR[r][0]] = 0;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				distance += params.timeCost[chromR[r][i-1]][chromR[r][i]];
				load += params.cli[chromR[r][i]].demand;
				service += params.cli[chromR[r][i]].serviceDuration;
				predecessors[chromR[r][i]] = chromR[r][i-1];
				successors[chromR[r][i-1]] = chromR[r][i];
			}
			successors[chromR[r][chromR[r].size()-1]] = 0;
			distance += params.timeCost[chromR[r][chromR[r].size()-1]][0];
			eval.distance += distance;
			eval.nbRoutes++;
			if (load > params.vehicleCapacity) eval.capacityExcess += load - params.vehicleCapacity;
			if (distance + service > params.durationLimit) eval.durationExcess += distance + service - params.durationLimit;
		}
	}
    // 修改评估的部分，改为适用于EVRP
    
    if(params.mode==0)
    {
        eval.distance=chroSplit_new5(chromT,params.c_evrp).first;
    }
    else if(params.mode==1)
    {
        eval.distance=calCost(chromR,params.c_evrp);
    }
    else if(params.mode==2)
    {
        // 如果是可行解，尝试用split优化
        if(eval.capacityExcess<MY_EPSILON)
        {
            auto temp_2=chroSplit_new5(chromT,params.c_evrp);
            chromR=temp_2.second;
            //有一些空的配送序列，也应该补充上的
            eval.distance=temp_2.first;
        }
        else
            eval.distance=calCost(chromR,params.c_evrp);
        
    }
    else
        throw std::string("Mode is not specified in individual.");
	eval.penalizedCost = eval.distance + eval.capacityExcess*params.penaltyCapacity + eval.durationExcess*params.penaltyDuration;
	eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);
    

    // double temp_distance=0;
    // for (int r = 0; r < params.nbVehicles; r++)
    // {
    //     if (!chromR[r].empty())
    //     {
    //         auto temp_route=chromR[r];
    //         temp_route.insert(temp_route.begin(),0);
    //         temp_route.push_back(0);
            
    //         temp_distance+=insertStationByRemove(temp_route,params.c_evrp).second;
    //     }
    // }
    // eval.distance=temp_distance;
    // eval.penalizedCost = eval.distance + eval.capacityExcess*params.penaltyCapacity + eval.durationExcess*params.penaltyDuration;
	// eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);
}

Individual::Individual(Params & params)
{
	successors = std::vector <int>(params.nbClients + 1);
	predecessors = std::vector <int>(params.nbClients + 1);
	chromR = std::vector < std::vector <int> >(params.nbVehicles);  //每辆车的配送序列
	chromT = std::vector <int>(params.nbClients);   //基因序列
	for (int i = 0; i < params.nbClients; i++) chromT[i] = i + 1;
	std::shuffle(chromT.begin(), chromT.end(), params.ran); //打乱顺序
	eval.penalizedCost = 1.e30;	
}

// 根据文件来构造个体
Individual::Individual(Params & params, std::string fileName) : Individual(params)
{
	double readCost;
	chromT.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops in the input file as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			while (ss >> inputCustomer) // Loops as long as there is an integer to read in this route
			{
				chromT.push_back(inputCustomer);
				chromR[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost") inputFile >> readCost;
		else throw std::string("Unexpected token in input solution");

		// Some safety checks and printouts
		evaluateCompleteCost(params);
		if ((int)chromT.size() != params.nbClients) throw std::string("Input solution does not contain the correct number of clients");
		if (!eval.isFeasible) throw std::string("Input solution is infeasible");
		if (eval.penalizedCost != readCost)throw std::string("Input solution has a different cost than announced in the file");
		if (params.verbose) std::cout << "----- INPUT SOLUTION HAS BEEN SUCCESSFULLY READ WITH COST " << eval.penalizedCost << std::endl;
	}
	else 
		throw std::string("Impossible to open solution file provided in input in : " + fileName);
}
