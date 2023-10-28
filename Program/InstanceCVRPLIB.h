//
// 根据路径读取实例文件
//

#ifndef INSTANCECVRPLIB_H
#define INSTANCECVRPLIB_H
#include<string>
#include<vector>

class InstanceCVRPLIB
{
public:
	std::vector<double> x_coords;					// 所有客户的x坐标
	std::vector<double> y_coords;					// 所有客户的y坐标
	std::vector< std::vector<double> > dist_mtx;	// 距离矩阵
	std::vector<double> service_time;				// 服务时间限制
	std::vector<double> demands;					// 需求
	double durationLimit = 1.e30;							// 路线的持续时间限制
	double vehicleCapacity = 1.e30;							// 容量限制
	bool isDurationConstraint = false;						// 问题是否包含持续时间约束，默认值为false
	int nbClients ;											// 客户的数目 (excluding the depot)

	InstanceCVRPLIB(std::string pathToInstance, bool isRoundingInteger);
};


#endif //INSTANCECVRPLIB_H
