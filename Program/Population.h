

#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "LocalSearch.h"
#include "Split.h"

typedef std::vector <Individual*> SubPopulation ;

class Population
{
   private:

   Params & params ;							// 问题参数
   Split & split;								// Split分割算法
   LocalSearch & localSearch;					// 局部搜索结构
   SubPopulation feasibleSubpop;			    // 可行子种群，按递增的惩罚成本排序
   SubPopulation infeasibleSubpop;		        // 不可行子种群，按递增的惩罚成本排序
   std::list <bool> listFeasibilityLoad ;		// 由本地搜索生成的最近个体的负载可行性
   std::list <bool> listFeasibilityDuration ;	// 由本地搜索生成的最近个体的持续时间可行性
   // std::vector<std::pair<clock_t, double>> searchProgress; // 记录连续最佳解决方案的时间戳
   std::vector<std::pair<int, double>> searchProgress;
   Individual bestSolutionRestart;              // 当前算法重启时找到的最佳解决方案
   Individual bestSolutionOverall;              // 算法执行过程中找到的最佳解决方案

   // 评估种群中所有个体的偏置适应度
   void updateBiasedFitnesses(SubPopulation & pop);

   // 移除偏置适应度最差的个体
   void removeWorstBiasedFitness(SubPopulation & subpop);

   public:

   // 创建初始个体种群
   void generatePopulation();

   // 在种群中添加个体（当种群达到最大大小时，自动触发生存者选择）
   // 如果找到了新的最佳解决方案，则返回TRUE
   bool addIndividual (const Individual & indiv, bool updateFeasible);

    // 清除所有解决方案并生成新的初始种群（仅在运行HGS直到达到时间限制时使用，此时算法会一直重启直到达到时间限制）
   void restart();

   // 惩罚参数的调整
   void managePenalties();

   // 通过二元锦标赛在可行和不可行子种群的联合中选择个体
   const Individual & getBinaryTournament();

   // 访问最佳可行个体
   const Individual * getBestFeasible();

   // 访问最佳不可行个体
   const Individual * getBestInfeasible();

   // 访问所有时间内找到的最佳解决方案
   const Individual * getBestFound();

   // 打印种群状态
   void printState(int nbIter, int nbIterNoImprovement);

   // 计算两个个体之间的距离，用于多样性计算
   double brokenPairsDistance(const Individual & indiv1, const Individual & indiv2);

   // 返回此个体与种群中最接近的nbClosest个体的平均破碎对距离
   double averageBrokenPairsDistanceClosest(const Individual & indiv, int nbClosest);

   // 返回子种群中50%最佳个体的平均多样性值
   double getDiversity(const SubPopulation & pop);

   // 返回子种群中50%最佳个体的平均解决方案值
   double getAverageCost(const SubPopulation & pop);

   // 将解决方案改进的历史导出到文件
   void exportSearchProgress(std::string fileName, std::string instanceName);


   
   // 将个体导出为CVRPLib格式
   void exportCVRPLibFormat(const Individual & indiv, std::string fileName);

   // 构造函数
   Population(Params & params, Split & split, LocalSearch & localSearch);

   // 析构函数
   ~Population();
};

#endif
