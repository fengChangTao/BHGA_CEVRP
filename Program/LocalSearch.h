

#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "Individual.h"

struct Node ;

// 包含一条路径的结构
struct Route
{
	int cour;							// 路线索引
	int nbCustomers;					// 路线中访问的客户数量
	int whenLastModified;				// 此路线最后一次被修改的时间
	int whenLastTestedSWAPStar;			// 此路线上的SWAP*移动最后一次被测试的时间
	Node * depot;						// 指向相关联的仓库的指针
	double duration;					// 路线上的总时间
	double load;						// 路线上的总负载
	double reversalDistance;			// 如果路线被反转，成本的差异
	double penalty;						// 当前的负载和持续时间惩罚的总和
	double polarAngleBarycenter;		// 路线的质心的极角
	CircleSector sector;				// 与客户集相关联的圆形区域
};

struct Node
{
	bool isDepot;						// 此节点是否代表一个仓库
	int cour;							// 节点索引
	int position;						// 在路线中的位置
	int whenLastTestedRI;				// 此节点的RI移动最后一次被测试的时间
	Node * next;						// 在路线顺序中的下一个节点
	Node * prev;						// 在路线顺序中的前一个节点
	Route * route;						// 指向相关联的路线的指针
	double cumulatedLoad;				// 在此路线上，直到客户（包括自身）的累积负载
	double cumulatedTime;				// 在此路线上，直到客户（包括自身）的累积时间
	double cumulatedReversalDistance;	// 如果路线的段（0...cour）被反转，成本的差异（对于不对称问题的2-opt移动很有用）
	double deltaRemoval;				// 如果节点从当前路线中移除，成本的差异（在SWAP*中使用）
};

// 在SWAP*中使用的结构，用于记住在给定路线中客户的三个最佳插入位置
struct ThreeBestInsert
{
	int whenLastCalculated;		// 最后一次计算的时间
	double bestCost[3];			// 三个最佳插入位置的成本
	Node * bestLocation[3];		// 三个最佳插入位置的节点

	void compareAndAdd(double costInsert, Node * placeInsert)
	{
		if (costInsert >= bestCost[2]) return;
		else if (costInsert >= bestCost[1])
		{
			bestCost[2] = costInsert; bestLocation[2] = placeInsert;
		}
		else if (costInsert >= bestCost[0])
		{
			bestCost[2] = bestCost[1]; bestLocation[2] = bestLocation[1];
			bestCost[1] = costInsert; bestLocation[1] = placeInsert;
		}
		else
		{
			bestCost[2] = bestCost[1]; bestLocation[2] = bestLocation[1];
			bestCost[1] = bestCost[0]; bestLocation[1] = bestLocation[0];
			bestCost[0] = costInsert; bestLocation[0] = placeInsert;
		}
	}

	// 重置结构（没有计算插入）
	void reset()
	{
		bestCost[0] = 1.e30; bestLocation[0] = NULL;
		bestCost[1] = 1.e30; bestLocation[1] = NULL;
		bestCost[2] = 1.e30; bestLocation[2] = NULL;
	}

	ThreeBestInsert() { reset(); };
};

// 用于追踪最佳SWAP*移动的结构
struct SwapStarElement
{
	double moveCost = 1.e30 ;
	Node * U = NULL ;
	Node * bestPositionU = NULL;
	Node * V = NULL;
	Node * bestPositionV = NULL;
};

// 主要局部搜索结构
class LocalSearch
{

private:
	
	Params & params ;							// 问题参数
	bool searchCompleted;						// 表示所有移动是否已经评估过且没有成功
	int nbMoves;								// 在本地搜索中应用的总移动次数（RI和SWAP*）。注意：这不仅仅是一个简单的计数器，它也用于避免重复移动评估
	std::vector < int > orderNodes;				// 在RI本地搜索中检查节点的随机顺序
	std::vector < int > orderRoutes;			// 在SWAP*本地搜索中检查路线的随机顺序
	std::set < int > emptyRoutes;				// 所有空路线的索引
	int loopID;									// 当前循环索引

	/* 解决方案表示为元素的链表 */
	std::vector < Node > clients;				// 表示客户的元素（clients[0]是一个哨兵，不应访问）
	std::vector < Node > depots;				// 表示仓库的元素
	std::vector < Node > depotsEnd;				// Duplicate of the depots to mark the end of the routes
	std::vector < Route > routes;				// 表示路线的元素
	std::vector < std::vector < ThreeBestInsert > > bestInsertClient;   // (SWAP*) 对于每个路线和节点，存储最便宜的插入成本 

	/* 本地搜索循环中使用的临时变量 */
	// nodeUPrev -> nodeU -> nodeX -> nodeXNext
	// nodeVPrev -> nodeV -> nodeY -> nodeYNext
	Node * nodeU ;
	Node * nodeX ;
    Node * nodeV ;
	Node * nodeY ;
	Route * routeU ;
	Route * routeV ;
	int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex ;	
	int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex ;	
	double loadU, loadX, loadV, loadY;
	double serviceU, serviceX, serviceV, serviceY;
	double penaltyCapacityLS, penaltyDurationLS ;
	bool intraRouteMove ; // 是否路径内移动

	void setLocalVariablesRouteU(); // 初始化一些与routeU相关的本地变量和距离，以避免在距离矩阵中总是查询相同的值
	void setLocalVariablesRouteV(); // 初始化一些与routeV相关的本地变量和距离，以避免在距离矩阵中总是查询相同的值

	inline double penaltyExcessDuration(double myDuration) {return std::max<double>(0., myDuration - params.durationLimit)*penaltyDurationLS;}
	inline double penaltyExcessLoad(double myLoad) {return std::max<double>(0., myLoad - params.vehicleCapacity)*penaltyCapacityLS;}

	/* RELOCATE MOVES */
	// (遗留标记：Prins 2004的move1...move9)
	bool move1(); // If U is a client node, remove U and insert it after V
	bool move2(); // If U and X are client nodes, remove them and insert (U,X) after V
	bool move3(); // If U and X are client nodes, remove them and insert (X,U) after V

	/* SWAP MOVES */
	bool move4(); // If U and V are client nodes, swap U and V
	bool move5(); // If U, X and V are client nodes, swap (U,X) and V
	bool move6(); // If (U,X) and (V,Y) are client nodes, swap (U,X) and (V,Y) 
	 
	/* 2-OPT and 2-OPT* MOVES */
	bool move7(); // If route(U) == route(V), replace (U,X) and (V,Y) by (U,V) and (X,Y)
	bool move8(); // If route(U) != route(V), replace (U,X) and (V,Y) by (U,V) and (X,Y)
	bool move9(); // If route(U) != route(V), replace (U,X) and (V,Y) by (U,Y) and (V,X)

	/* 用于有效交换评估的子程序 */
	bool swapStar(); // 计算路线U和路线V之间的所有SWAP*，并应用最佳改进移动
	double getCheapestInsertSimultRemoval(Node * U, Node * V, Node *& bestPosition); // 计算V在路线中的插入成本和位置，其中V被省略
	void preprocessInsertions(Route * R1, Route * R2); // 预处理路线R1中所有节点在路线R2中的插入成本

	/* ROUTINES TO UPDATE THE SOLUTIONS */
	static void insertNode(Node * U, Node * V);		// 解决方案更新：在V之后插入U
	static void swapNode(Node * U, Node * V) ;		// 解决方案更新：交换U和V							   
	void updateRouteData(Route * myRoute);			// 更新路线的预处理数据

	public:

	// 使用指定的惩罚值运行局部搜索
	void run(Individual & indiv, double penaltyCapacityLS, double penaltyDurationLS);

	// 将初始解决方案加载到局部搜索中
	void loadIndividual(const Individual & indiv);

	// 将LS解决方案导出到个体，并根据Params中的原始惩罚权重计算惩罚成本
	void exportIndividual(Individual & indiv);

	// 构造函数
	LocalSearch(Params & params);
};

#endif
