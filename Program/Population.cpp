#include "Population.h"
// 创建初始个体种群
void Population::generatePopulation()
{
	if (params.verbose) std::cout << "----- BUILDING INITIAL POPULATION" << std::endl;
	for (int i = 0; i < 4*params.ap.mu && (i == 0 || params.ap.timeLimit == 0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit) ; i++)
	{
		Individual randomIndiv(params);
		split.generalSplit(randomIndiv, params.nbVehicles);
		localSearch.run(randomIndiv, params.penaltyCapacity, params.penaltyDuration);
		addIndividual(randomIndiv, true);
		if (!randomIndiv.eval.isFeasible && params.ran() % 2 == 0)  // Repair half of the solutions in case of infeasibility
		{
			localSearch.run(randomIndiv, params.penaltyCapacity*10., params.penaltyDuration*10.);
			if (randomIndiv.eval.isFeasible) addIndividual(randomIndiv, false);
		}
	}
}

// 在种群中添加个体（当种群达到最大大小时，自动触发生存者选择）
// 如果找到了新的最佳解决方案，则返回TRUE
bool Population::addIndividual(const Individual & indiv, bool updateFeasible)
{
	if (updateFeasible)
	{
		listFeasibilityLoad.push_back(indiv.eval.capacityExcess < MY_EPSILON);
		listFeasibilityDuration.push_back(indiv.eval.durationExcess < MY_EPSILON);
		listFeasibilityLoad.pop_front();
		listFeasibilityDuration.pop_front();
	}

	// 根据个体的可行性找到相应的子种群
	SubPopulation & subpop = (indiv.eval.isFeasible) ? feasibleSubpop : infeasibleSubpop;

	// 创建个体的副本，并更新相似度结构，计算个体之间的距离
	Individual * myIndividual = new Individual(indiv);
	for (Individual * myIndividual2 : subpop)
	{
		double myDistance = brokenPairsDistance(*myIndividual,*myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	// 在子种群中找到正确的位置，并插入个体
	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->eval.penalizedCost > indiv.eval.penalizedCost - MY_EPSILON) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);

	// 如果子种群大小超过最大值，则进行生存者选择
	if ((int)subpop.size() > params.ap.mu + params.ap.lambda)
		while ((int)subpop.size() > params.ap.mu)
			removeWorstBiasedFitness(subpop);

	// 跟踪最佳解决方案
	if (indiv.eval.isFeasible && indiv.eval.penalizedCost < bestSolutionRestart.eval.penalizedCost - MY_EPSILON)
	{
		bestSolutionRestart = indiv; // Copy
		if (indiv.eval.penalizedCost < bestSolutionOverall.eval.penalizedCost - MY_EPSILON)
		{
			bestSolutionOverall = indiv;
			//searchProgress.push_back({ clock() - params.startTime , bestSolutionOverall.eval.penalizedCost });
			searchProgress.push_back({ params.dai , bestSolutionOverall.eval.penalizedCost });
		}
		return true;
	}
	else
		return false;
}
// 评估种群中所有个体的偏置适应度
void Population::updateBiasedFitnesses(SubPopulation & pop)
{
	// 根据个体的多样性贡献（距离的降序）对个体进行排名
	std::vector <std::pair <double, int> > ranking;
	for (int i = 0 ; i < (int)pop.size(); i++) 
		ranking.push_back({-averageBrokenPairsDistanceClosest(*pop[i],params.ap.nbClose),i});
	std::sort(ranking.begin(), ranking.end());

	// 更新有偏差的适应度值
	if (pop.size() == 1) 
		pop[0]->biasedFitness = 0;
	else
	{
		for (int i = 0; i < (int)pop.size(); i++)
		{
			double divRank = (double)i / (double)(pop.size() - 1); // Ranking from 0 to 1
			double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
			if ((int)pop.size() <= params.ap.nbElite) // 精英个体不能小于种群大小
				pop[ranking[i].second]->biasedFitness = fitRank;
			else 
				pop[ranking[i].second]->biasedFitness = fitRank + (1.0 - (double)params.ap.nbElite / (double)pop.size()) * divRank;
		}
	}
}
// 移除偏置适应度最差的个体
void Population::removeWorstBiasedFitness(SubPopulation & pop)
{
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in HGS");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 1; i < (int)pop.size(); i++)
	{
		bool isClone = (averageBrokenPairsDistanceClosest(*pop[i],1) < MY_EPSILON); // A distance equal to 0 indicates that a clone exists
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFitness > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFitness;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}
	// 在种群中，清除该个体，释放内存
	// Removing the individual from the population and freeing memory
	pop.erase(pop.begin() + worstIndividualPosition); 

	// 在种群中，清除其他个体到该个体的距离
	// Cleaning its distances from the other individuals in the population
	for (Individual * indiv2 : pop)
	{
		auto it = indiv2->indivsPerProximity.begin();
		while (it->second != worstIndividual) ++it;
		indiv2->indivsPerProximity.erase(it);
	}

	// 释放内存
	delete worstIndividual; 
}
// 清除所有解决方案并生成新的初始种群（仅在运行HGS直到达到时间限制时使用，此时算法会一直重启直到达到时间限制）
void Population::restart()
{
	if (params.verbose) std::cout << "----- RESET: CREATING A NEW POPULATION -----" << std::endl;
	for (Individual * indiv : feasibleSubpop) delete indiv ;
	for (Individual * indiv : infeasibleSubpop) delete indiv;
	feasibleSubpop.clear();
	infeasibleSubpop.clear();
	bestSolutionRestart = Individual(params);
	generatePopulation();
}
// 惩罚参数的调整，后期可以优化为快速排序
void Population::managePenalties()
{
	// 为了安全起见，将惩罚值设置在[0.1,100000]之间
	double fractionFeasibleLoad = (double)std::count(listFeasibilityLoad.begin(), listFeasibilityLoad.end(), true) / (double)listFeasibilityLoad.size();
	if (fractionFeasibleLoad < params.ap.targetFeasible - 0.05 && params.penaltyCapacity < 100000.)
		params.penaltyCapacity = std::min<double>(params.penaltyCapacity * params.ap.penaltyIncrease, 100000.);
	else if (fractionFeasibleLoad > params.ap.targetFeasible + 0.05 && params.penaltyCapacity > 0.1)
		params.penaltyCapacity = std::max<double>(params.penaltyCapacity * params.ap.penaltyDecrease, 0.1);

	// 为了安全起见，将惩罚值设置在[0.1,100000]之间
	double fractionFeasibleDuration = (double)std::count(listFeasibilityDuration.begin(), listFeasibilityDuration.end(), true) / (double)listFeasibilityDuration.size();
	if (fractionFeasibleDuration < params.ap.targetFeasible - 0.05 && params.penaltyDuration < 100000.)
		params.penaltyDuration = std::min<double>(params.penaltyDuration * params.ap.penaltyIncrease, 100000.);
	else if (fractionFeasibleDuration > params.ap.targetFeasible + 0.05 && params.penaltyDuration > 0.1)
		params.penaltyDuration = std::max<double>(params.penaltyDuration * params.ap.penaltyDecrease, 0.1);

	// 更新评估值
	for (int i = 0; i < (int)infeasibleSubpop.size(); i++)
		infeasibleSubpop[i]->eval.penalizedCost = infeasibleSubpop[i]->eval.distance
		+ params.penaltyCapacity * infeasibleSubpop[i]->eval.capacityExcess
		+ params.penaltyDuration * infeasibleSubpop[i]->eval.durationExcess;

	// 如果需要，重新排序不可行个体子种群中的个体，因为惩罚值已经改变（为了简单起见，使用简单的冒泡排序）
	for (int i = 0; i < (int)infeasibleSubpop.size(); i++)
	{
		for (int j = 0; j < (int)infeasibleSubpop.size() - i - 1; j++)
		{
			if (infeasibleSubpop[j]->eval.penalizedCost > infeasibleSubpop[j + 1]->eval.penalizedCost + MY_EPSILON)
			{
				Individual * indiv = infeasibleSubpop[j];
				infeasibleSubpop[j] = infeasibleSubpop[j + 1];
				infeasibleSubpop[j + 1] = indiv;
			}
		}
	}
}
// 通过二元锦标赛在可行和不可行子种群的联合中选择个体
const Individual & Population::getBinaryTournament ()
{
	// 从可行和不可行个体子种群的并集中均匀分布地选择两个个体
	std::uniform_int_distribution<> distr(0, feasibleSubpop.size() + infeasibleSubpop.size() - 1);
	int place1 = distr(params.ran);
	int place2 = distr(params.ran);
	Individual * indiv1 = (place1 >= (int)feasibleSubpop.size()) ? infeasibleSubpop[place1 - feasibleSubpop.size()] : feasibleSubpop[place1];
	Individual * indiv2 = (place2 >= (int)feasibleSubpop.size()) ? infeasibleSubpop[place2 - feasibleSubpop.size()] : feasibleSubpop[place2];
	
	// 保留两个个体中偏向适应度更好的一个
	updateBiasedFitnesses(feasibleSubpop);
	updateBiasedFitnesses(infeasibleSubpop);
	if (indiv1->biasedFitness < indiv2->biasedFitness) return *indiv1 ;
	else return *indiv2 ;		
}
// 访问最佳可行个体
const Individual * Population::getBestFeasible ()
{
	if (!feasibleSubpop.empty()) return feasibleSubpop[0] ;
	else return NULL ;
}
// 访问最佳不可行个体
const Individual * Population::getBestInfeasible ()
{
	if (!infeasibleSubpop.empty()) return infeasibleSubpop[0] ;
	else return NULL ;
}
// 访问所有时间内找到的最佳解决方案
const Individual * Population::getBestFound()
{
	if (bestSolutionOverall.eval.penalizedCost < 1.e29) return &bestSolutionOverall;
	else return NULL;
}
// 打印种群状态
void Population::printState(int nbIter, int nbIterNoImprovement)
{
	if (params.verbose)
	{
		std::printf("It %6d %6d | T(s) %.2f", nbIter, nbIterNoImprovement, (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC);

		if (getBestFeasible() != NULL) std::printf(" | Feas %zu %.2f %.2f", feasibleSubpop.size(), getBestFeasible()->eval.penalizedCost, getAverageCost(feasibleSubpop));
		else std::printf(" | NO-FEASIBLE");

		if (getBestInfeasible() != NULL) std::printf(" | Inf %zu %.2f %.2f", infeasibleSubpop.size(), getBestInfeasible()->eval.penalizedCost, getAverageCost(infeasibleSubpop));
		else std::printf(" | NO-INFEASIBLE");

		std::printf(" | Div %.2f %.2f", getDiversity(feasibleSubpop), getDiversity(infeasibleSubpop));
		std::printf(" | Feas %.2f %.2f", (double)std::count(listFeasibilityLoad.begin(), listFeasibilityLoad.end(), true) / (double)listFeasibilityLoad.size(), (double)std::count(listFeasibilityDuration.begin(), listFeasibilityDuration.end(), true) / (double)listFeasibilityDuration.size());
		std::printf(" | Pen %.2f %.2f", params.penaltyCapacity, params.penaltyDuration);
		std::cout << std::endl;
	}
}
// 计算两个个体之间的距离，用于多样性计算
double Population::brokenPairsDistance(const Individual & indiv1, const Individual & indiv2)
{
	int differences = 0;
	for (int j = 1; j <= params.nbClients; j++)
	{
		if (indiv1.successors[j] != indiv2.successors[j] && indiv1.successors[j] != indiv2.predecessors[j]) differences++;
		if (indiv1.predecessors[j] == 0 && indiv2.predecessors[j] != 0 && indiv2.successors[j] != 0) differences++;
	}
	return (double)differences / (double)params.nbClients;
}
// 返回此个体与种群中最接近的nbClosest个体的平均破碎对距离
double Population::averageBrokenPairsDistanceClosest(const Individual & indiv, int nbClosest)
{
	double result = 0.;
	int maxSize = std::min<int>(nbClosest, indiv.indivsPerProximity.size());
	auto it = indiv.indivsPerProximity.begin();
	for (int i = 0; i < maxSize; i++)
	{
		result += it->first;
		++it;
	}
	return result / (double)maxSize;
}
// 返回子种群中50%最佳个体的平均多样性值
double Population::getDiversity(const SubPopulation & pop)
{
	double average = 0.;
	int size = std::min<int>(params.ap.mu, pop.size()); // Only monitoring the "mu" better solutions to avoid too much noise in the measurements
	for (int i = 0; i < size; i++) average += averageBrokenPairsDistanceClosest(*pop[i],size);
	if (size > 0) return average / (double)size;
	else return -1.0;
}
// 返回子种群中50%最佳个体的平均解决方案值
double Population::getAverageCost(const SubPopulation & pop)
{
	double average = 0.;
	int size = std::min<int>(params.ap.mu, pop.size()); // Only monitoring the "mu" better solutions to avoid too much noise in the measurements
	for (int i = 0; i < size; i++) average += pop[i]->eval.penalizedCost;
	if (size > 0) return average / (double)size;
	else return -1.0;
}
// 将解决方案改进的历史导出到文件
void Population::exportSearchProgress(std::string fileName, std::string instanceName)
{
	std::ofstream myfile(fileName);
	for (std::pair<clock_t, double> state : searchProgress)
		//myfile << instanceName << ";" << params.ap.seed << ";" << state.second << ";" << (double)state.first / (double)CLOCKS_PER_SEC << std::endl;
		myfile << instanceName << ";" << params.ap.seed << ";" << state.first << ";" << state.second << std::endl;
}


// 将个体导出为CVRPLib格式
void Population::exportCVRPLibFormat(const Individual & indiv, std::string fileName)
{
	std::ofstream myfile(fileName);
	if (myfile.is_open())
	{
		for (int k = 0; k < (int)indiv.chromR.size(); k++)
		{
			if (!indiv.chromR[k].empty())
			{
				myfile << "Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
				for (int i : indiv.chromR[k]) myfile << " " << i;
				myfile << std::endl;
			}
		}
		myfile << "Cost " << indiv.eval.penalizedCost << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
}

Population::Population(Params & params, Split & split, LocalSearch & localSearch) : params(params), split(split), localSearch(localSearch), bestSolutionRestart(params), bestSolutionOverall(params)
{
	listFeasibilityLoad = std::list<bool>(params.ap.nbIterPenaltyManagement, true);
	listFeasibilityDuration = std::list<bool>(params.ap.nbIterPenaltyManagement, true);
}

Population::~Population()
{
	for (int i = 0; i < (int)feasibleSubpop.size(); i++) delete feasibleSubpop[i];
	for (int i = 0; i < (int)infeasibleSubpop.size(); i++) delete infeasibleSubpop[i];
}