#include "Individual.h" 

//pair<vector<int>, double> insertStationByFeng(vector<int> route, Case& instance)
//{
//    list<pair<int, int>> stationInserted;
//    for(int i = 0; i < (int)route.size() - 1; i++)
//    {
//
//    }
//}

// 移除启发
pair<vector<int>, double> insertStationByRemove(vector<int> route, Case& instance) {
	list<pair<int, int>> stationInserted;
	for (int i = 0; i < (int)route.size() - 1; i++) {
		double allowedDis = instance.maxDis;
		if (i != 0) {
			allowedDis = instance.maxDis - instance.distances[stationInserted.back().second][route[i]];
		}
		int onestation = instance.findNearestStationFeasible(route[i], route[i + 1], allowedDis);
		if (onestation == -1) return make_pair(route, -1);
		stationInserted.push_back(make_pair(i, onestation));
	}
	/*for (int i = 0; i < (int)route.size() - 1; i++) {
		stationInserted.push_back(make_pair(i, instance.bestStation[route[i]][route[i + 1]]));
	}*/
	while (!stationInserted.empty())
	{
		bool changed = false;
		list<pair<int, int>>::iterator delone = stationInserted.begin();
		double savedis = 0;
		list<pair<int, int>>::iterator itr = stationInserted.begin();
		list<pair<int, int>>::iterator next = itr;
		next++;
		if (next != stationInserted.end()) {
			int endInd = next->first;
			int endstation = next->second;
			double sumdis = 0;
			for (int i = 0; i < endInd; i++) {
				sumdis += instance.distances[route[i]][route[i + 1]];
			}
			sumdis += instance.distances[route[endInd]][endstation];
			if (sumdis <= instance.maxDis) {
				savedis = instance.distances[route[itr->first]][itr->second] + instance.distances[itr->second][route[itr->first + 1]]
					- instance.distances[route[itr->first]][route[itr->first + 1]];
			}
		}
		else {
			double sumdis = 0;
			for (int i = 0; i < (int)route.size() - 1; i++) {
				sumdis += instance.distances[route[i]][route[i + 1]];
			}
			if (sumdis <= instance.maxDis) {
				savedis = instance.distances[route[itr->first]][itr->second] + instance.distances[itr->second][route[itr->first + 1]]
					- instance.distances[route[itr->first]][route[itr->first + 1]];
			}
		}
		itr++;
		while (itr != stationInserted.end())
		{
			int startInd, endInd;
			next = itr;
			next++;
			list<pair<int, int>>::iterator prev = itr;
			prev--;
			double sumdis = 0;
			if (next != stationInserted.end()) {
				startInd = prev->first + 1;
				endInd = next->first;
				sumdis += instance.distances[prev->second][route[startInd]];
				for (int i = startInd; i < endInd; i++) {
					sumdis += instance.distances[route[i]][route[i + 1]];
				}
				sumdis += instance.distances[route[endInd]][next->second];
				if (sumdis <= instance.maxDis) {
					double savedistemp = instance.distances[route[itr->first]][itr->second] + instance.distances[itr->second][route[itr->first + 1]]
						- instance.distances[route[itr->first]][route[itr->first + 1]];
					if (savedistemp > savedis) {
						savedis = savedistemp;
						delone = itr;
					}
				}
			}
			else {
				startInd = prev->first + 1;
				sumdis += instance.distances[prev->second][route[startInd]];
				for (int i = startInd; i < (int)route.size() - 1; i++) {
					sumdis += instance.distances[route[i]][route[i + 1]];
				}
				if (sumdis <= instance.maxDis) {
					double savedistemp = instance.distances[route[itr->first]][itr->second] + instance.distances[itr->second][route[itr->first + 1]]
						- instance.distances[route[itr->first]][route[itr->first + 1]];
					if (savedistemp > savedis) {
						savedis = savedistemp;
						delone = itr;
					}
				}
			}
			itr++;
		}
		if (savedis != 0) {
			stationInserted.erase(delone);
			changed = true;
		}
		if (!changed) {
			break;
		}
	}
	while (!stationInserted.empty())
	{
		route.insert(route.begin() + stationInserted.back().first + 1, stationInserted.back().second);
		stationInserted.pop_back();
	}
	double summ = 0;
	for (int i = 0; i < (int)route.size() - 1; i++) {
		summ += instance.distances[route[i]][route[i + 1]];
	}
	return make_pair(route, summ);
}
// 传统的Split分割算法
pair<double, vector<vector<int>>> chroSplit_new(vector<int> x, Case instance) {
	x.insert(x.begin(), 0);
	int lens = instance.depotNumber + instance.customerNumber;
	vector<int> pp(lens, 0);
	vector<double> vv(lens, DBL_MAX);

	vv[0] = 0;
	for (int i = 1; i < (int)x.size(); i++) {
		double load = 0;
		double cost = 0;
		int j = i;
		for (;;)
		{
			load += instance.demand[x[j]];
			if (i == j) {
				cost = instance.distances[0][x[j]] * 2;
			}
			else {
				cost = cost - instance.distances[x[j - 1]][0];
				cost = cost + instance.distances[x[j - 1]][x[j]];
				cost = cost + instance.distances[0][x[j]];
			}
			if (load <= (double)instance.maxC) {
				if (vv[i - 1] + cost < vv[j]) {
					vv[j] = vv[i - 1] + cost;
					pp[j] = i - 1;
				}
				j++;
			}
			if (j > instance.customerNumber || load > instance.maxC)
				break;
		}
	}

	vector<vector<int>> allroutes;
	int j = x.size() - 1;
	while (true) {
		int i = pp[j];
		vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
		allroutes.push_back(temp);
		j = i;
		if (i == 0) {
			break;
		}
	}

	double totalDistance = 0.0;
	for (auto route : allroutes)
	{
		// 在route的开头插入一个0
		route.insert(route.begin(), 0);
		// 在route的末尾添加一个0
		route.push_back(0);
        
        totalDistance +=insertStationByRemove(route, instance).second;
	}

	return pair<double, vector<vector<int>>>(totalDistance, allroutes);
}

void Individual::evaluateCompleteCost(const Params & params)
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
    eval.distance=chroSplit_new(chromT,params.c_evrp).first;
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
