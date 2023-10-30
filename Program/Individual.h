

#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H
#pragma once
#include "Params.h"
#include "case.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <set>
#include <unordered_set>
#include <list>
#include <sstream>
using namespace std;

// 我引入的两个函数
pair<vector<int>, double> insertStationByRemove(vector<int> route, Case& instance);
pair<double, vector<vector<int>>> chroSplit_new(vector<int> x, Case instance);
//pair<vector<int>, double> insertStationByFeng(vector<int> route, Case& instance);

//  类：评估个体
struct EvalIndiv
{
	double penalizedCost = 0.;		// 解决方案的惩罚成本
	int nbRoutes = 0;				// 路线数量
	double distance = 0.;			// 总距离
	double capacityExcess = 0.;		// 所有路线中超出的负载总和
	double durationExcess = 0.;		// 所有路线中超出的持续时间总和
	bool isFeasible = false;		// 个体的可行性状态
};
// 类：个体
class Individual
{
public:

  EvalIndiv eval;															// 解决方案的成本参数
  std::vector < int > chromT ;								// 表示个体的巨型旅行
  std::vector < std::vector <int> > chromR ;	// 每个车辆的配送序列（完整解决方案）
  std::vector < int > successors ;						// 解决方案中每个节点的后继（可以是仓库0）
  std::vector < int > predecessors ;					// 解决方案中每个节点的前驱（可以是仓库0）
  std::multiset < std::pair < double, Individual* > > indivsPerProximity ;	// 种群中的其他个体, 根据 proximity递增排序 (集合容器根据对的第一个值进行自然排序)
  double biasedFitness;												// 解决方案的偏置适应度
  
  // 从chromR的信息中测量个体的成本和可行性（需要填充chromR并访问Params）
  void evaluateCompleteCost(const Params & params);

  // 构造一个只包含有随机访问顺序的巨型旅行的随机个体
  Individual(Params & params);

  // 从CVRPLib解决方案格式的文件中构造个体，该格式由算法生成（如果用户希望输入初始解决方案，这将很有用）
  Individual(Params & params, std::string fileName);
};
#endif
