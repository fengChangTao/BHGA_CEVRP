#include "case.h"
Case::Case()
{

}
Case::Case(string filename, int ID) {
	this->ID = ID;
	this->filename = filename;
	stringstream ss;
	this->depotNumber = 1;
	this->depot = 0;
	ifstream infile(filename.c_str());
	char line[250];
    if(!infile.is_open())
        throw std::string("Impossible to open instance file: " + filename);
	while (infile.getline(line, 249))
	{
		string templine(line);
		if (templine.find("DIMENSION:") != string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->customerNumber;
			ss.clear();
			this->customerNumber--;
		}
		else if (templine.find("STATIONS:") != string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->stationNumber;
			ss.clear();
		}
		else if (templine.find("VEHICLES:") != string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->vehicleNumber;
			ss.clear();
		}
		else if (templine.find("CAPACITY:") != string::npos && templine.find("ENERGY") == string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->maxC;
			ss.clear();
		}
		else if (templine.find("ENERGY_CAPACITY:") != string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->maxQ;
			ss.clear();
		}
		else if (templine.find("ENERGY_CONSUMPTION:") != string::npos) {
			string substr = templine.substr(templine.find(":") + 1);
			ss << substr;
			ss >> this->conR;
			ss.clear();
		}
		else if (templine.find("NODE_COORD_SECTION") != string::npos) {
			int totalNumber = depotNumber + customerNumber + stationNumber;
			for (int i = 0; i < totalNumber; i++) {
				positions.push_back(make_pair(0, 0));
			}
			for (int i = 0; i < totalNumber; i++) {
				infile.getline(line, 249);
				templine = line;
				ss << templine;
				int ind;
				double x, y;
				ss >> ind >> x >> y;
				ss.clear();
				positions[ind - 1].first = x;
				positions[ind - 1].second = y;
			}
		}
		else if (templine.find("DEMAND_SECTION") != string::npos) {
			int totalNumber = depotNumber + customerNumber;
			for (int i = 0; i < totalNumber; i++) {
				demand.push_back(0);
			}
			for (int i = 0; i < totalNumber; i++) {
				infile.getline(line, 249);
				templine = line;
				ss << templine;
				int ind;
				double c;
				ss >> ind >> c;
				ss.clear();
				demand[ind - 1] = c;
				if (c == 0) {
					depot = ind - 1;
				}
			}
		}
	}
	totalNumber = depotNumber + customerNumber + stationNumber;

	this->distances.resize(totalNumber, vector<double>(totalNumber, 0));

	for (int i = 0; i < totalNumber; i++) {
		for (int j = i; j < totalNumber; j++) {
			if (i == j) {
				this->distances[i][j] = 0;
			}
			else {
				double xx = this->positions[i].first - this->positions[j].first;
				double yy = this->positions[i].second - this->positions[j].second;
				this->distances[i][j] = this->distances[j][i] = sqrt(xx * xx + yy * yy);
			}
		}
	}
	this->maxDis = maxQ / conR;
	this->posflag = false;

	// �� bestStations ����Ϊ�����С
	bestStations.resize(depotNumber + customerNumber, vector<vector<int>>(depotNumber + customerNumber));

	// ɾ��ԭ�е� for ѭ��
	for (int i = 0; i < depotNumber + customerNumber - 1; i++) {
		for (int j = i + 1; j < depotNumber + customerNumber; j++) {
			this->bestStations[i][j] = this->bestStations[j][i] = findTheNonDominatedStations(i, j);
		}
	}

	this->bestStation.resize(depotNumber + customerNumber, vector<int>(depotNumber + customerNumber, 0));

	for (int i = 0; i < depotNumber + customerNumber - 1; i++) {
		for (int j = i + 1; j < depotNumber + customerNumber; j++) {
			this->bestStation[i][j] = this->bestStation[j][i] = findNearestStation(i, j);
		}
	}

	this->totalDem = 0;
	for (auto& e : demand) {
		this->totalDem += e;
	}

	this->candidatelist = getCandiList2(20);
    // new features
    bs2.resize(depotNumber + customerNumber, vector<pair<int,double>>(depotNumber + customerNumber,pair<int,double>(-1,0)));
    for (int i = 0; i < depotNumber + customerNumber - 1; i++)
    {
        for (int j = i + 1; j < depotNumber + customerNumber - 1; j++)
        {
            int cs = 0;
            double a, b, c;
            double duo = 0;
            if (this->bestStation[i][j]!=0)
            {
                cs = this->bestStation[i][j];
                a = this->distances[i][cs];
                b = this->distances[cs][j];
                c = this->distances[i][j];
                duo = a + b - c;
                this->bs2[i][j].second = this->bs2[j][i].second = duo;
                if ((pow(a, 2) + pow(b, 2) > pow(c, 2))&&i!=0)
                {
                    this->bs2[i][j].first = this->bs2[j][i].first = 2;
                }
            }
            
        }
    }


    
    feng6_bestStations.resize(depotNumber + customerNumber, vector<vector<int>>(depotNumber + customerNumber));
    // 对0-other的候选cs进行过滤
    int su2=0,su3=0;
    // 对0-other部分，用自己规则,筛选候选cs-多个
    for(int j=1;j<depotNumber + customerNumber;j++)
    {
        
        auto kkk=bestStations[0][j];
        vector<tuple<int,double,double>> biao;
        for(auto h:kkk)
        {
            int csk=h;
            double akkb=distances[0][csk]+distances[csk][j];
            double kb=distances[csk][j];
            tuple<int,double,double> temp(csk,akkb,kb);
            biao.push_back(temp);
            
        }
        std::sort(biao.begin(), biao.end(), [](const tuple<int, double, double>& a, const tuple<int, double, double>& b) {
            if (get<1>(a) != get<1>(b)) return get<1>(a) < get<1>(b); // Compare second elements
            if (get<2>(a) != get<2>(b)) return get<2>(a) < get<2>(b); // Compare third elements
            return get<0>(a) < get<0>(b); // Finally compare first elements
        });
        
        vector<tuple<int,double,double>> temp_biao;
        //遍历biao中的元素，合适的加入feng6_bestStations[0][j]
        double lastKb = INT_MAX; // 用于存储前一个元素的第三个字段
        for (const auto& item : biao) {
            double currentKb = get<2>(item); // 当前元素的第三个字段
            if (lastKb-currentKb>0.00001 ) {
                feng6_bestStations[0][j].push_back(get<0>(item)); // 将当前元素添加到feng6_bestStations[0][j]
                feng6_bestStations[j][0].push_back(get<0>(item));
                lastKb = currentKb; // 更新lastKb
            }
            else
            {
                continue;
            }

        }
        su2+=kkk.size();
        su3+=feng6_bestStations[0][j].size();
    }
    // 其他部分，照搬候选cs-单个
    for(int i=1;i<depotNumber + customerNumber;i++)
    {
        for(int j=i+1;j<depotNumber + customerNumber;j++)
        {
            feng6_bestStations[i][j].push_back(bestStation[i][j]);
            feng6_bestStations[j][i].push_back(bestStation[j][i]);
        }
    }
    
}

vector<vector<int>> Case::getCandiList2(int candino) {
	vector<set<int>> candilist = getCandiList(candino);
	vector<vector<int>> candilist2;
	for (auto a : candilist) {
		vector<int> temp;
		for (auto h : a) {
			temp.push_back(h);
		}
		candilist2.push_back(temp);
	}
	return candilist2;
}

vector<set<int>> Case::getCandiList(int candino) {
	vector<set<int>> candilist;
	for (int i = 0; i < this->depotNumber + this->customerNumber; i++) {
		set<int> onelist;
		int largeone = -1;
		double largedis = 0;
		for (int j = 0; j < this->depotNumber + this->customerNumber; j++) {
			if (i == j) continue;
			if ((int)onelist.size() < candino) {
				onelist.insert(j);
				if (largedis < this->distances[i][j]) {
					largedis = this->distances[i][j];
					largeone = j;
				}
			}
			else {
				if (largedis > this->distances[i][j]) {
					onelist.erase(largeone);
					onelist.insert(j);
					largedis = 0;
					for (auto e : onelist) {
						if (this->distances[i][e] > largedis) {
							largedis = this->distances[i][e];
							largeone = e;
						}
					}
				}
			}
		}
		candilist.push_back(onelist);
	}
	return candilist;
}

vector<int> Case::findTheNonDominatedStations(int x, int y) {
	vector<int> temp;
	for (int i = this->customerNumber + this->depotNumber; i < this->customerNumber + this->depotNumber + this->stationNumber; i++) {
		bool bedominated = false;
		for (auto e : temp) {
			if (this->distances[x][e] <= this->distances[x][i] &&
				this->distances[e][y] <= this->distances[i][y]) {
				bedominated = true;
				break;
			}
		}
		if (bedominated == false) {
			for (int j = 0; j < (int)temp.size(); j++) {
				if (this->distances[x][i] <= this->distances[x][temp[j]] &&
					this->distances[i][y] <= this->distances[temp[j]][y]) {
					temp.erase(temp.begin() + j);
					j--;
				}
			}
			temp.push_back(i);
		}
	}
	return temp;
}

Case::~Case() {
	int totalNumber = depotNumber + customerNumber + stationNumber;



}

double Case::getDistance(int i, int j) {
	return this->distances[i][j];
}

double Case::getEnergyDemand(int i, int j) {
	return this->distances[i][j] * this->conR;
}

double Case::calculateRouteDistance(vector<int> x) {
	double sum = 0;
	double piecesum = 0;
	double piececap = 0;
	for (int i = 0; i < (int)x.size() - 1; i++) {
		piecesum += distances[x[i]][x[i + 1]];
		if (x[i] == 0) piececap = 0;
		else if (x[i] > 0 && x[i] < depotNumber + customerNumber) piececap += demand[x[i]];
		sum += distances[x[i]][x[i + 1]];
		if (piecesum > maxDis || piececap > maxC) return -1 * piecesum;
		if (x[i + 1] < depotNumber || x[i + 1] >= depotNumber + customerNumber)
			piecesum = 0;
	}
	return sum;
}

double Case::calculateRouteDistance(int* x, int length) {
	double sum = 0;
	double piecesum = 0;
	for (int i = 0; i < length - 1; i++) {
		piecesum += distances[x[i]][x[i + 1]];
		sum += distances[x[i]][x[i + 1]];
		if (piecesum > maxDis) return -1 * piecesum;
		if (x[i + 1] < depotNumber || x[i + 1] >= depotNumber + customerNumber)
			piecesum = 0;
	}
	return sum;
}

double Case::calculateRouteDistance(int* x) {
	double sum = 0;
	int totalNumber = depotNumber + customerNumber + stationNumber;
	int counter = 0;
	while (!(x[counter + 1] >= totalNumber || x[counter + 1] < 0))
	{
		sum += distances[x[counter]][x[counter + 1]];
	}
	return sum;
}

int Case::findNearestStation(int x) {
	int thestation = -1;
	double bigdis = DBL_MAX;
	for (int i = depotNumber + customerNumber; i < depotNumber + customerNumber + stationNumber; i++) {
		if (bigdis > distances[x][i] && x != i) {
			thestation = i;
			bigdis = distances[x][i];
		}
	}
	return thestation;
}

int Case::findNearestStation(int x, int y) {
	int thestation = -1;
	double bigdis = DBL_MAX;
	for (int i = depotNumber + customerNumber; i < depotNumber + customerNumber + stationNumber; i++) {
		if (bigdis > distances[x][i] + distances[y][i] && x != i && y != i) {
			thestation = i;
			bigdis = distances[x][i] + distances[y][i];
		}
	}
	return thestation;
}


//reture the station within maxdis from x, and min dis[x][s]+dis[y][s]
int Case::findNearestStationFeasible(int x, int y, double maxdis){
	int thestation = -1;
	double bigdis = DBL_MAX;
	for (int i = depotNumber + customerNumber; i < depotNumber + customerNumber + stationNumber; i++) {
		if (distances[x][i] < maxdis && bigdis > distances[x][i] + distances[y][i] && i != x && y != i && distances[i][y] < maxDis) {
			thestation = i;
			bigdis = distances[x][i] + distances[y][i];
		}
	}
	return thestation;
}
// 考虑depot节点也可以充电的情况
int Case::findNearestStationFeasible2(int x, int y, double maxdis) {
	int thestation = -1;
	double bigdis = DBL_MAX;
	for (int i = depotNumber + customerNumber; i < depotNumber + customerNumber + stationNumber; i++) {
		if (distances[x][i] < maxdis && bigdis > distances[x][i] + distances[y][i] && i != x && y != i && distances[i][y] < maxDis) {
			thestation = i;
			bigdis = distances[x][i] + distances[y][i];
		}
	}
	if (distances[x][0] < maxdis && bigdis > distances[x][0] + distances[y][0] && 0 != x && y != 0 && distances[0][y] < maxDis) {
		thestation = 0;
		bigdis = distances[x][0] + distances[y][0];
	}
	return thestation;
}

void Case::writeAllPositions() {
	string outfilename = filename.substr(0, filename.find(".evrp") + 1) + "pos";
	ofstream oufile(outfilename.c_str());
	oufile << depotNumber + customerNumber << endl;
	for (auto a : positions) {
		oufile << a.first << ',' << a.second << endl;
	}
	oufile.close();
	posflag = true;
}

void Case::writeAllPositions(vector<int> kkk) {
	string outfilename = filename.substr(0, filename.find(".evrp") + 1) + "pos";
	ofstream oufile(outfilename.c_str());
	oufile << depotNumber + customerNumber << endl;
	//for (;;)
	//{

	//}
	for (auto a : positions) {
		oufile << a.first << ',' << a.second << endl;
	}
	oufile.close();
	posflag = true;
}

// void Case::drawARoute(vector<int> route, string picname) {
// 	if (posflag == false) {
// 		writeAllPositions();
// 	}
// 	string solufilename = "tempsolution.txt";
// 	string posfilename = filename.substr(0, filename.find(".evrp") + 1) + "pos";
// 	ofstream solufile(solufilename.c_str());
// 	for (int i = 0; i < (int)route.size(); i++) {
// 		solufile << route[i];
// 		if (i != (int)route.size() - 1) {
// 			solufile << ',';
// 		}
// 	}
// 	solufile << endl;
// 	string command = "python ./draw.py " + posfilename + " " + picname;
// 	system(command.c_str());
// 	remove(solufilename.c_str());
// }

void Case::testTheStationReach() {
	bool flag = false;
	for (int i = 0; i < stationNumber; i++) {
		if (maxDis < distances[0][1 + customerNumber + i]) {
			cout << 1 + customerNumber + i << endl;
			flag = true;
		}
	}
	if (flag) {
		cout << "There is station that cannot be directly reached " << ID << endl;
	}
	else {
		cout << "all good" << endl;
	}
}

void Case::checkASoluton(string filename) {
	ifstream ifile(filename.c_str());
	vector<int> arr;
	string solustr;
	getline(ifile, solustr);
	getline(ifile, solustr);
	istringstream is(solustr);
	int node;
	char dot;
	vector<int> arr1;
	while (is >> node)
	{
		arr1.push_back(node);
		is >> dot;
	}
	vector<vector<int>> solution;
	for (int i = 0; i < (int)arr1.size(); i++) {
		if (arr1[i] == 0) {
			vector<int> temp;
			solution.push_back(temp);
		}
		solution[solution.size() - 1].push_back(arr1[i]);
	}
	for (int i = 0; i < (int)solution.size(); i++) {
		solution[i].push_back(0);
	}
	//bool capcheck = true;
	//bool elecheck = true;
	for (int i = 0; i < (int)solution.size(); i++) {
		double totc = 0;
		double totd = 0;
		for (int j = 1; j < (int)solution[i].size(); j++) {
			if (solution[i][j - 1] == 0 || solution[i][j - 1] >= 1 + customerNumber) {
				totd = distances[solution[i][j - 1]][solution[i][j]];
			}
			else {
				totd += distances[solution[i][j - 1]][solution[i][j]];
			}
			if (solution[i][j] > 0 && solution[i][j] < 1 + customerNumber) {
				totc += demand[solution[i][j]];
			}
			if (totc > maxC || totd > maxDis) {
				cout << filename << endl;
				return;
			}
		}
	}
}

vector<int> Case::findTheNonDominatedStations_feng(int x, int y) {
    vector<int> temp;
    for (int i = this->customerNumber + this->depotNumber; i < this->customerNumber + this->depotNumber + this->stationNumber; i++) {
        bool bedominated = false;
        for (auto e : temp) {
            if (this->distances[x][e] <= this->distances[x][i] &&
                this->distances[e][y] <= this->distances[i][y]) {
                bedominated = true;
                break;
            }
        }
        if (bedominated == false) {
            for (int j = 0; j < (int)temp.size(); j++) {
                if (this->distances[x][i] <= this->distances[x][temp[j]] &&
                    this->distances[i][y] <= this->distances[temp[j]][y]) {
                    temp.erase(temp.begin() + j);
                    j--;
                }
            }
            temp.push_back(i);
        }
    }
    return temp;
}





// 传统的Split分割算法，很少用了，可能是原来Case复制这里太费时
pair<double, vector<vector<int>>> chroSplit_new5(vector<int> x, Case& instance) {
    x.insert(x.begin(), 0);
    int lens = instance.depotNumber + instance.customerNumber;
    vector<int> pp(lens, 0);
    vector<double> vv(lens, DBL_MAX);
    
    vv[0] = 0;
    for (int i = 1; i < (int)x.size(); i++) {
        double load = 0;
        double cost = 0;
        int j = i;
        for (;;)
        {
            load += instance.demand[x[j]];
            if (i == j) {
                cost = instance.distances[0][x[j]] * 2;
            }
            else {
                cost = cost - instance.distances[x[j - 1]][0];
                cost = cost + instance.distances[x[j - 1]][x[j]];
                cost = cost + instance.distances[0][x[j]];
            }
            if (load <= (double)instance.maxC) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
            if (j > instance.customerNumber || load > instance.maxC)
                break;
        }
    }
    
    vector<vector<int>> allroutes;
    int j = x.size() - 1;
    while (true) {
        int i = pp[j];
        vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
        allroutes.push_back(temp);
        j = i;
        if (i == 0) {
            break;
        }
    }
    
    double totalDistance = 0.0;
    for (auto route : allroutes)
    {
        // 在route的开头插入一个0
        route.insert(route.begin(), 0);
        // 在route的末尾添加一个0
        route.push_back(0);
        
        totalDistance +=focusEnumeration(route, instance).second;
    }
    
    return pair<double, vector<vector<int>>>(totalDistance, allroutes);
}

// 用于个体中计算cost
double calCost(vector<vector<int>>& allRoutes,Case& instance)
{
    double cost=0;
    for(auto route:allRoutes)
    {
        if(route.size()==0) continue;
        route.insert(route.begin(), 0);
        route.push_back(0);
        cost+=focusEnumeration(route,instance).second;
    }
    return cost;
}

bool Case::checkAevRoute(vector<int>& route)
{
    double ssum = 0;
    for (int i = 0; i < route.size() - 1; i++)
    {
        ssum += this->distances[route[i]][route[i + 1]];
        if (ssum - this->maxDis > 0.00001)
            return false;
        if (route[i + 1] >= depotNumber + customerNumber)
            ssum = 0;
    }
    return true;
}
double Case::caluAvroute(vector<int>& route)
{
    double ssum = 0;
    for (int i = 0; i < route.size() - 1; i++)
        ssum += this->distances[route[i]][route[i + 1]];
    return ssum;
}
vector<int> blendRoute(vector<int> route, vector<vector<int>>& ins)
{
    for (int i = ins.size() - 1; i >= 0; i--)
    {
        if (ins[i].size() != 0)
        {
            if (i <= route.size()) {
                // 在route[i]后插入ins[i]的所有元素
                route.insert(route.begin() + i + 1, ins[i].begin(), ins[i].end());
            }
            
        }
    }
    return route;
}

pair<vector<int>, double> clusterEnumeration(vector<int> route, Case& instance)
{
    double yuan = instance.caluAvroute(route);
    int lb = floor( yuan/ instance.maxDis);
    int ub = lb + 1;
    if (lb <= 0)
        return pair<vector<int>, double>(route,yuan);
    // 间隙，连续的2为一个聚类
    vector<pair<int,double>> jian(route.size() - 1, pair<int,double>(-1,-1));
    for (int i = 0; i < route.size() - 1; i++)
    {
        jian[i].first = instance.bs2[route[i]][route[i + 1]].first;
        jian[i].second = instance.bs2[route[i]][route[i + 1]].second;
    }
    // 聚类中，也寻找一个间隙，使其变为-1
    double now=INT_MAX;
    int select=-1;
    unordered_set<int> need;
    
    for(int i=0;i<jian.size();i++)
    {
        if(jian[i].first==-1)
        {
            if(select!=-1)
            {
                need.insert(select);
            }
            now=INT_MAX;
            select=-1;
            continue;
        }
        if(jian[i].second<now)
        {
            now=jian[i].second;
            select=i;
        }
        
    }
    for(auto& h:need)
        jian[h].first=-1;
    
    
    
    
    return pair<vector<int>, double>(route,-1);
}

// 焦点枚举
void focus_tryACertainN(int mlen, int nlen, int* chosenSta, int* chosenPos, vector<int>& finalRoute, double& finalfit, int curub, vector<int>& route, vector<double>& accumulateDis, Case& instance)
{
    for (int i = mlen; i <= (int)route.size() - 1 - nlen; i++) {
        // 瞻前
        if (curub == nlen) {
            if (accumulateDis[i] - instance.maxDis > 0.00001) {
                break;
            }
        }
        else {
            int lastpos = chosenPos[curub - nlen - 1];
            auto lastcs = chosenSta[curub - nlen - 1];
            double onedis = instance.distances[route[lastpos + 1]][lastcs];
            if (onedis + accumulateDis[i] - accumulateDis[lastpos + 1] - instance.maxDis > 0.00001)
                break;
            
        }
        // 顾后
        if (nlen >= 1) {
            if (accumulateDis.back() - accumulateDis[i + 1] - nlen * (instance.maxDis) > 0.00001) {
                continue;
            }
        }
        
        //if (nlen == 1) {
        //	if (accumulateDis.back() - accumulateDis[i + 1] >= instance->maxDis) {
        //		continue;
        //	}
        //}
        
        
        
        
        for (int j = 0; j < (int)instance.feng6_bestStations[route[i]][route[i + 1]].size(); j++) {
            // 再次瞻前
            auto select_cs = instance.feng6_bestStations[route[i]][route[i + 1]][j];
            double qian = instance.distances[route[i]][select_cs];
            
            if (curub == nlen) {
                if (qian + accumulateDis[i] - instance.maxDis > 0.00001) {
                    continue;
                }
            }
            else {
                int lastpos = chosenPos[curub - nlen - 1];
                auto lastcs = chosenSta[curub - nlen - 1];
                double onedis = instance.distances[route[lastpos + 1]][lastcs];
                if (qian + onedis + accumulateDis[i] - accumulateDis[lastpos + 1] - instance.maxDis > 0.00001)
                    continue;
            }
            
            // 再次顾后
            double hou = instance.distances[select_cs][route[i + 1]];
            if (nlen >= 1) {
                if (hou + accumulateDis.back() - accumulateDis[i + 1] - nlen * (instance.maxDis) > 0.00001) {
                    continue;
                }
            }
            chosenSta[curub - nlen] = instance.feng6_bestStations[route[i]][route[i + 1]][j];
            chosenPos[curub - nlen] = i;
            if (nlen > 1) {
                focus_tryACertainN(i + 1, nlen - 1, chosenSta, chosenPos, finalRoute, finalfit, curub, route, accumulateDis, instance);
            }
            else {
                bool feasible = true;
                double piecedis = accumulateDis[chosenPos[0]] + instance.distances[route[chosenPos[0]]][chosenSta[0]];
                if (piecedis > instance.maxDis)
                    feasible = false;
                for (int k = 1; feasible && k < curub; k++) {
                    piecedis = accumulateDis[chosenPos[k]] - accumulateDis[chosenPos[k - 1] + 1];
                    piecedis += instance.distances[chosenSta[k - 1]][route[chosenPos[k - 1] + 1]];
                    piecedis += instance.distances[chosenSta[k]][route[chosenPos[k]]];
                    if (piecedis > instance.maxDis) feasible = false;
                }
                piecedis = accumulateDis.back() - accumulateDis[chosenPos[curub - 1] + 1];
                piecedis += instance.distances[route[chosenPos[curub - 1] + 1]][chosenSta[curub - 1]];
                if (piecedis > instance.maxDis)
                {
                    feasible = false;
                }
                
                if (feasible) {
                    double totaldis = accumulateDis.back();
                    for (int k = 0; k < curub; k++) {
                        int firstnode = route[chosenPos[k]];
                        int secondnode = route[chosenPos[k] + 1];
                        totaldis -= instance.distances[firstnode][secondnode];
                        totaldis += instance.distances[firstnode][chosenSta[k]];
                        totaldis += instance.distances[chosenSta[k]][secondnode];
                    }
                    if (totaldis < finalfit) {
                        finalfit = totaldis;
                        finalRoute = route;
                        for (int k = curub - 1; k >= 0; k--) {
                            finalRoute.insert(finalRoute.begin() + chosenPos[k] + 1, chosenSta[k]);
                        }
                    }
                }
            }
        }
    }
}
pair<vector<int>, double> focusEnumeration(vector<int> route, Case& instance)
{
    //计算累计距离
    vector<double> accumulateDistance(route.size(), 0);
    for (int i = 1; i < (int)route.size(); i++) {
        accumulateDistance[i] = accumulateDistance[i - 1] + instance.distances[route[i]][route[i - 1]];
    }
    //说明不需要充电
    if (accumulateDistance.back() <= instance.maxDis) {
        return make_pair(route, accumulateDistance.back());
    }
    
    int ub = (int)(accumulateDistance.back() / instance.maxDis + 1);
    int lb = (int)(accumulateDistance.back() / instance.maxDis);
    int* chosenPos = new int[route.size()];
    int* chosenSta = new int[route.size()];
    vector<int> finalRoute;		//最终路线
    double finalfit = DBL_MAX;	//最终适应度
    for (int i = lb; i <= ub; i++) {
        focus_tryACertainN(0, i, chosenSta, chosenPos, finalRoute, finalfit, i, route, accumulateDistance, instance);
    }
    delete[] chosenPos;
    delete[] chosenSta;
    if (finalfit != DBL_MAX) {
        return make_pair(finalRoute, finalfit);
    }
    else {
        return make_pair(route, INT_MAX);
    }
}

//// 限制枚举
//void tryACertainN(int mlen, int nlen, int* chosenSta, int* chosenPos, vector<int>& finalRoute, double& finalfit, int curub, vector<int>& route, vector<double>& accumulateDis, Case* instance)
//{
//    // 遍历路径中的每个位置，以确定充电站的插入位置
//    for (int i = mlen; i <= (int)route.size() - 1 - nlen; i++) {
//        // 瞻前，看看前半段是否满足充电约束
//        if (curub == nlen) {
//            // 如果还没插入过充电站
//            if (accumulateDis[i] >= instance->maxDis) {
//                break;
//            }
//        }
//        else {
//            // 如果前面有充电站了，确保从上一个充电站到当前位置的距离不超过最大距离
//            if (accumulateDis[i] - accumulateDis[chosenPos[curub - nlen - 1] + 1] >= instance->maxDis) {
//                break;
//            }
//        }
//        // 顾后，看看后半段是否还有希望
//        if (nlen == 1) {
//            // 如果只剩一个充电站未确定，确保从当前位置到终点的距离不超过最大距离
//            if (accumulateDis.back() - accumulateDis[i + 1] >= instance->maxDis) {
//                continue;
//            }
//        }
//        // 遍历当前间隔所有可选择的cs
//        for (int j = 0; j < (int)instance->bestStations[route[i]][route[i + 1]].size(); j++) {
//            // 选定一个充电站和对应的位置
//            chosenSta[curub - nlen] = instance->bestStations[route[i]][route[i + 1]][j];
//            chosenPos[curub - nlen] = i;
//            // 如果还需要确定更多充电站的位置，递归调用此函数
//            if (nlen > 1) {
//                tryACertainN(i + 1, nlen - 1, chosenSta, chosenPos, finalRoute, finalfit, curub, route, accumulateDis, instance);
//            }
//            else {
//                // 如果已确定所有充电站位置，检查整条路线是否满足距离约束
//                bool feasible = true;
//                // 计算从起点到第一个充电站的距离
//                double piecedis = accumulateDis[chosenPos[0]] + instance->distances[route[chosenPos[0]]][chosenSta[0]];
//                if (piecedis > instance->maxDis) feasible = false;
//                // 计算每两个充电站之间的距离
//                for (int k = 1; feasible && k < curub; k++) {
//                    piecedis = accumulateDis[chosenPos[k]] - accumulateDis[chosenPos[k - 1] + 1];
//                    piecedis += instance->distances[chosenSta[k - 1]][route[chosenPos[k - 1] + 1]];
//                    piecedis += instance->distances[chosenSta[k]][route[chosenPos[k]]];
//                    if (piecedis > instance->maxDis) feasible = false;
//                }
//                // 计算从最后一个充电站到终点的距离
//                piecedis = accumulateDis.back() - accumulateDis[chosenPos[curub - 1] + 1];
//                piecedis += instance->distances[route[chosenPos[curub - 1] + 1]][chosenSta[curub - 1]];
//                if (piecedis > instance->maxDis) feasible = false;
//                // 如果当前配置可行，计算整条路线的总距离，并更新最优解
//                if (feasible) {
//                    double totaldis = accumulateDis.back();
//                    for (int k = 0; k < curub; k++) {
//                        int firstnode = route[chosenPos[k]];
//                        int secondnode = route[chosenPos[k] + 1];
//                        totaldis -= instance->distances[firstnode][secondnode];
//                        totaldis += instance->distances[firstnode][chosenSta[k]];
//                        totaldis += instance->distances[chosenSta[k]][secondnode];
//                    }
//                    if (totaldis < finalfit) {
//                        finalfit = totaldis;
//                        finalRoute = route;
//                        for (int k = curub - 1; k >= 0; k--) {
//                            finalRoute.insert(finalRoute.begin() + chosenPos[k] + 1, chosenSta[k]);
//                        }
//                    }
//                }
//            }
//        }
//    }
//}
//pair<vector<int>, double> insertStationByEnumeration(vector<int> route, Case* instance)
//{
//    //计算累计距离
//    vector<double> accumulateDistance(route.size(), 0);
//    for (int i = 1; i < (int)route.size(); i++) {
//        accumulateDistance[i] = accumulateDistance[i - 1] + instance->distances[route[i]][route[i - 1]];
//    }
//    //说明不需要充电
//    if (accumulateDistance.back() <= instance->maxDis) {
//        return make_pair(route, accumulateDistance.back());
//    }
//
//    int ub = (int)(accumulateDistance.back() / instance->maxDis + 3);
//    int lb = (int)(accumulateDistance.back() / instance->maxDis);
//    int* chosenPos = new int[route.size()];
//    int* chosenSta = new int[route.size()];
//    vector<int> finalRoute;		//最终路线
//    double finalfit = DBL_MAX;	//最终适应度
//    for (int i = lb; i <= ub; i++) {
//        tryACertainN(0, i, chosenSta, chosenPos, finalRoute, finalfit, i, route, accumulateDistance, instance);
//    }
//    delete[] chosenPos;
//    delete[] chosenSta;
//    if (finalfit != DBL_MAX) {
//        return make_pair(finalRoute, finalfit);
//    }
//    else {
//        return make_pair(route, -1);
//    }
//}


