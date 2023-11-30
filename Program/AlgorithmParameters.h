#ifndef ALGORITHMPARAMETERS_H
#define ALGORITHMPARAMETERS_H

struct AlgorithmParameters {
	int nbGranular;			// 粒度搜索参数，限制RI局部搜索中的移动次数，默认值为20
	int mu;					// 最小种群大小，默认值为25
	int lambda;				// 在达到最大种群规模之前创建的解决方案数量（即，一代的大小），默认值为40。
	int nbElite;			// 精英个体的数量，默认值为5
	int nbClose;			// 在计算多样性贡献时考虑的最近解决方案/个体的数量，默认值为4还是5（存疑）
	int nbIterPenaltyManagement;  // 每隔多少次迭代更新一次惩罚，默认值为100
	double targetFeasible;	      // 可行个体数量的参考比例，用于适应惩罚参数，默认值为0.2
	double penaltyDecrease;	      // 如果有足够的可行个体，用于减少惩罚参数的乘数，默认值为0.85
	double penaltyIncrease;	      // 如果可行个体不足，用于增加惩罚参数的乘数，默认值为1.2

	int seed;				// 随机数种子，默认值: 0
	int nbIter;				// 没有改进时的最大迭代次数，在终止（或如果指定了时间限制，则重新开始）之前，默认值为20,000
	int nbIterTraces;       // 在HGS执行期间，每多少次迭代显示一次追踪，默认值为500。
	double timeLimit;		// CPU时间限制，默认值: 0
	int useSwapStar;		// 是否使用SWAP*局部搜索，默认值为1。只有在提供坐标时才可用。
    
    int preCharge=0;        // 是否使用预充电策略，默认为--否
    int hou=0;              // 是否局部搜索后确认evrp值变优了才录用，默认为--否
    int numMoves=88;         // 预充电具体步数到第几个move，默认为88，即根据前选项。
    int mode=0;
};

#ifdef __cplusplus
extern "C"
#endif
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
void print_algorithm_parameters(const AlgorithmParameters & ap);
#endif


#endif //ALGORITHMPARAMETERS_H
