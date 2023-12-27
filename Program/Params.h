#ifndef PARAMS_H
#define PARAMS_H

#include "CircleSector.h"
#include "AlgorithmParameters.h"
#include "case.h"
#include <string>
#include <vector>
#include <list>
#include <set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <time.h>
#include <climits>
#include <algorithm>
#include <unordered_set>
#include <random>
#define MY_EPSILON 0.00001 // 精度参数，用于避免数值不稳定
#define PI 3.14159265359

// 客户
struct Client
{
	double coordX;			// X坐标
	double coordY;			// Y坐标
	double serviceDuration; // 服务持续时间
	double demand;			// 客户的需求量
	int polarAngle;			// 客户围绕仓库的极角，以度为单位，并为了方便而截断
};

class Params
{
public:

    bool isChu=true;
	/* 遗传算法参数 */
	bool verbose;                       // 控制日志的详细级别，在迭代过程中
	AlgorithmParameters ap;	            // HGS算法的主要参数

	/* 自适应惩罚系数 */
	double penaltyCapacity;				// 单位容量超额的惩罚（在搜索过程中进行调整）
	double penaltyDuration;				// 单位持续时间超额的惩罚（在搜索过程中进行调整）

	/* 算法开始时间 */
	clock_t startTime;                  // 优化的开始时间（在Params构造时设置）

	/* 随机数生成器 */       
	std::minstd_rand ran;               // 随机数生成器。使用最快和最简单的LCG。随机数的质量对LS不关键，但速度关键。

	/* 问题实例数据 */
	bool isDurationConstraint ;								// 表示问题是否包含持续时间约束
	int nbClients ;											// 客户数量（不包括仓库）
	int nbVehicles ;										// 车辆数量
	double durationLimit;									// 路线持续时间限制
	double vehicleCapacity;									// 容量限制
	double totalDemand ;									// 客户们要求的总需求
	double maxDemand;										// 客户的最大需求
	double maxDist;											// 两个客户之间的最大距离
	std::vector< Client > cli ;								// 包含每个客户信息的vector
	const std::vector< std::vector< double > >& timeCost;	// 距离矩阵
	std::vector< std::vector< int > > correlatedVertices;	// 邻域限制：对于每个客户，列出附近的客户列表
	bool areCoordinatesProvided;                            // 检查是否提供了有效的坐标

    Case c_evrp=Case("../Instances/CVRP/X-n143-k7.evrp",1);	// 引入evrp实例
	string s1="";
    int mode=0;
	int dai=0;
    int preCharge;  // 是否使用预充电策略，默认为--否
    int hou;        // 是否后检查，局部搜索后确认evrp值变优了才录用，默认为--否
    int numMoves;   // 预充电具体步数到第几个move，默认为--0
    std::string selected;
	// 从给定数据集开始初始化
    Params(
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
		const AlgorithmParameters& ap);
};
#endif

