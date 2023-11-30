#ifndef GENETIC_H
#define GENETIC_H

#include "Population.h"
#include "Individual.h"

class Genetic
{
public:

	Params & params;				// 问题参数
	Split split;					// Split分割算法
	LocalSearch localSearch;		// 局部搜索结构
	Population population;			// 种群 (public for now to give access to the solutions, but should be be improved later on)
	Individual offspring;			// 第一个被用作交叉输入的个体

	// OX交叉
	void crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2);

    // 运行遗传算法，直到maxIterNonProd连续迭代或达到时间限制
    void run() ;

	// 构造函数
	Genetic(Params & params);
};

#endif
