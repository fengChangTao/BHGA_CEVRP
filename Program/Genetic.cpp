#include "Genetic.h"

void Genetic::run()
{	
	/* 初始化种群 */
	population.generatePopulation();

	int nbIter;				// 总迭代次数
	int nbIterNonProd = 1;	// 未改进迭代次数
	if (params.verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	for (nbIter = 0,params.dai=0 ; nbIterNonProd <= params.ap.nbIter && (params.ap.timeLimit == 0 || (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC < params.ap.timeLimit) ; nbIter++,params.dai++)
	{	
		/* 选择并交叉 */
		crossoverOX(offspring, population.getBinaryTournament(),population.getBinaryTournament());

		/* 局部搜索 */
		localSearch.run(offspring, params.penaltyCapacity, params.penaltyDuration);
		bool isNewBest = population.addIndividual(offspring,true);
		if (!offspring.eval.isFeasible && params.ran()%2 == 0) // 在不可行的情况下修复一半的解决方案
		{
			localSearch.run(offspring, params.penaltyCapacity*10., params.penaltyDuration*10.);
			if (offspring.eval.isFeasible) isNewBest = (population.addIndividual(offspring,false) || isNewBest);
		}

		/* 跟踪自上次解决方案改进以来的迭代次数 */
		if (isNewBest) nbIterNonProd = 1;
		else nbIterNonProd ++ ;

		/* DIVERSIFICATION, 惩罚管理 AND 追踪 */
		if (nbIter % params.ap.nbIterPenaltyManagement == 0) population.managePenalties();
		if (nbIter % params.ap.nbIterTraces == 0) population.printState(nbIter, nbIterNonProd);

		/* 对于涉及连续运行直到时间限制的测试:每次maxIterNonProd达到时，我们重置算法/人口*/
		if (params.ap.timeLimit != 0 && nbIterNonProd == params.ap.nbIter)
		{
			population.restart();
			nbIterNonProd = 1;
		}
		
		std::ofstream fengfile(params.s1 + ".dai.txt",std::ios::app);
		fengfile<<nbIter<<" "<<population.getBestFeasible()->eval.distance<<std::endl;
		
	}
	if (params.verbose) std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC << std::endl;
}

void Genetic::crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2)
{
	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool> (params.nbClients + 1, false);

	// Picking the beginning and end of the crossover zone
	std::uniform_int_distribution<> distr(0, params.nbClients-1);
	int start = distr(params.ran);
	int end = distr(params.ran);

	// Avoid that start and end coincide by accident
	while (end == start) end = distr(params.ran);

	// Copy from start to end
	int j = start;
	while (j % params.nbClients != (end + 1) % params.nbClients)
	{
		result.chromT[j % params.nbClients] = parent1.chromT[j % params.nbClients];
		freqClient[result.chromT[j % params.nbClients]] = true;
		j++;
	}

	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <= params.nbClients; i++)
	{
		int temp = parent2.chromT[(end + i) % params.nbClients];
		if (freqClient[temp] == false)
		{
			result.chromT[j % params.nbClients] = temp;
			j++;
		}
	}

	// Complete the individual with the Split algorithm
	split.generalSplit(result, parent1.eval.nbRoutes);
}

Genetic::Genetic(Params & params) : 
	params(params), 
	split(params),
	localSearch(params),
	population(params,this->split,this->localSearch),
	offspring(params){}

