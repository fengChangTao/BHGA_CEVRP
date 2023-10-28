#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <float.h>
#include <string>
#include <stdio.h>

using namespace std;

class Case {
public:
	Case(string, int);
	~Case();
	double getDistance(int, int);
	// 获取两点间能量需求
	double getEnergyDemand(int, int);
	// 计算一条路径的距离
	double calculateRouteDistance(vector<int>);
	double calculateRouteDistance(int*, int);
	double calculateRouteDistance(int*);
	// 寻找最近的充电站
	int findNearestStation(int);
	int findNearestStation(int, int);
	// 寻找最近的可行充电站
	int findNearestStationFeasible(int, int, double);
	int findNearestStationFeasible2(int, int, double);
	// 输出所有位置
	void writeAllPositions();
	void writeAllPositions(vector<int>);
	// 画出一条路径
	// void drawARoute(vector<int>, string);
	// 测试充电站的可达性
	void testTheStationReach();
	// 检查一个解决方案
	void checkASoluton(string);
	// 找到非支配充电站
	vector<int> findTheNonDominatedStations(int, int);
	// 获取候选列表
	vector<set<int>> getCandiList(int candino);
	vector<vector<int>> getCandiList2(int candino);

	int depotNumber;// 仓库数量
	int customerNumber;// 客户数量
	int stationNumber;// 充电站数量
	int vehicleNumber;// 车辆数量
	int depot;// 仓库编号
	vector<pair<double, double>> positions;// 所有点的位置
	vector<vector<double>> distances;
	
	vector<vector<vector<int>>> bestStations;
	// 每个客户点最佳充电站
	vector<vector<int>> bestStation;
	vector<double> demand;// 每个客户的需求
	double maxC;// 车辆容量
	double maxQ;// 能量容量
	double conR;// 能耗系数
	double maxDis;// 行驶距离限制
	double totalDem;// 所有客户需求的总和
	string filename; //文件名
	bool posflag;// 表示位置是否可用的标志。
	int ID;//案例的唯一标识符。
	vector<vector<int>> candidatelist;//一个二维向量，表示候选列表。

	vector<vector<int>> correlatedVertices;//存储最近的20个节点
};
