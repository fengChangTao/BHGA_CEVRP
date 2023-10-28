#include "Params.h"

// 可执行库和共享库的通用构造函数
// 当从命令行运行可执行文件时,
// 它首先从.vrp文件生成一个CVRPLIB实例，然后提供必要的信息。
Params::Params(
	const std::vector<double>& x_coords,
	const std::vector<double>& y_coords,
	const std::vector<std::vector<double>>& dist_mtx,
	const std::vector<double>& service_time,
	const std::vector<double>& demands,
	double vehicleCapacity,
	double durationLimit,
	int nbVeh,
	bool isDurationConstraint,
	bool verbose,
	const AlgorithmParameters& ap
)
	: ap(ap), isDurationConstraint(isDurationConstraint), nbVehicles(nbVeh), durationLimit(durationLimit),
	  vehicleCapacity(vehicleCapacity), timeCost(dist_mtx), verbose(verbose)
{
	// 算法开始时间
	startTime = clock();
    
	nbClients = (int)demands.size() - 1; // Need to substract the depot from the number of nodes
	totalDemand = 0.;
	maxDemand = 0.;

	// 初始化RNG随机数种子
	ran.seed(ap.seed);

	// 检查是否提供了有效的坐标
	areCoordinatesProvided = (demands.size() == x_coords.size()) && (demands.size() == y_coords.size());

	cli = std::vector<Client>(nbClients + 1);
	for (int i = 0; i <= nbClients; i++)
	{
		// If useSwapStar==false, x_coords and y_coords may be empty.
		if (ap.useSwapStar == 1 && areCoordinatesProvided)
		{
			cli[i].coordX = x_coords[i];
			cli[i].coordY = y_coords[i];
			cli[i].polarAngle = CircleSector::positive_mod(
				32768. * atan2(cli[i].coordY - cli[0].coordY, cli[i].coordX - cli[0].coordX) / PI);
		}
		else
		{
			cli[i].coordX = 0.0;
			cli[i].coordY = 0.0;
			cli[i].polarAngle = 0.0;
		}

		cli[i].serviceDuration = service_time[i];
		cli[i].demand = demands[i];
		if (cli[i].demand > maxDemand) maxDemand = cli[i].demand;
		totalDemand += cli[i].demand;
	}

	if (verbose && ap.useSwapStar == 1 && !areCoordinatesProvided)
		std::cout << "----- NO COORDINATES HAVE BEEN PROVIDED, SWAP* NEIGHBORHOOD WILL BE DEACTIVATED BY DEFAULT" << std::endl;

	// 如果用户没有提供车辆数量，则默认初始化
	if (nbVehicles == INT_MAX)
	{
		nbVehicles = (int)std::ceil(1.3*totalDemand/vehicleCapacity) + 3;  // Safety margin: 30% + 3 more vehicles than the trivial bin packing LB
		if (verbose) 
			std::cout << "----- FLEET SIZE WAS NOT SPECIFIED: DEFAULT INITIALIZATION TO " << nbVehicles << " VEHICLES" << std::endl;
	}
	else
	{
		if (verbose)
			std::cout << "----- FLEET SIZE SPECIFIED: SET TO " << nbVehicles << " VEHICLES" << std::endl;
	}

	// 计算最大距离
	maxDist = 0.;
	for (int i = 0; i <= nbClients; i++)
		for (int j = 0; j <= nbClients; j++)
			if (timeCost[i][j] > maxDist) maxDist = timeCost[i][j];

	// 计算每个客户的相关顶点(对于粒度限制)
	correlatedVertices = std::vector<std::vector<int> >(nbClients + 1);
	std::vector<std::set<int> > setCorrelatedVertices = std::vector<std::set<int> >(nbClients + 1);
	std::vector<std::pair<double, int> > orderProximity;
	for (int i = 1; i <= nbClients; i++)
	{
		orderProximity.clear();
		for (int j = 1; j <= nbClients; j++)
			if (i != j) orderProximity.emplace_back(timeCost[i][j], j);
		std::sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(ap.nbGranular, nbClients - 1); j++)
		{
			// If i is correlated with j, then j should be correlated with i
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
	}

	// 填充相关顶点的vector
	for (int i = 1; i <= nbClients; i++)
		for (int x : setCorrelatedVertices[i])
			correlatedVertices[i].push_back(x);

	// 在包含任意小或任意大数值的情况下，避免可能的数值不稳定的保护措施
	if (maxDist < 0.1 || maxDist > 100000)
		throw std::string(
			"The distances are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (maxDemand < 0.1 || maxDemand > 100000)
		throw std::string(
			"The demand quantities are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (nbVehicles < std::ceil(totalDemand / vehicleCapacity))
		throw std::string("Fleet size is insufficient to service the considered clients.");

	// 惩罚初始值的合理比例
	penaltyDuration = 1;
	penaltyCapacity = std::max<double>(0.1, std::min<double>(1000., maxDist / maxDemand));

	if (verbose)
		std::cout << "----- INSTANCE SUCCESSFULLY LOADED WITH " << nbClients << " CLIENTS AND " << nbVehicles << " VEHICLES" << std::endl;
}


