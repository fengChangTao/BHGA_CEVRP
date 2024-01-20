#include "LocalSearch.h" 

void LocalSearch::run(Individual & indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
	this->penaltyCapacityLS = penaltyCapacityLS;
	this->penaltyDurationLS = penaltyDurationLS;
	loadIndividual(indiv);

	// 打乱LS探索的节点的顺序，以便在搜索中有更多的多样性
	std::shuffle(orderNodes.begin(), orderNodes.end(), params.ran);
	std::shuffle(orderRoutes.begin(), orderRoutes.end(), params.ran);
	for (int i = 1; i <= params.nbClients; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // 平均0 (n/nbGranular)次调用内部函数，以实现整体的线性时间复杂度
			std::shuffle(params.correlatedVertices[i].begin(), params.correlatedVertices[i].end(), params.ran);

	searchCompleted = false;
	for (loopID = 0; !searchCompleted; loopID++)
	{
		if (loopID > 1) // 允许至少两个循环，因为一些涉及空路线的移动在第一个循环时没有被检查
			searchCompleted = true;

		/* 经典路线改进(RI)的移动受到邻近限制 */
		for (int posU = 0; posU < params.nbClients; posU++)
		{
   
            
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVertices[nodeU->cour].size(); posV++)
			{
    
				nodeV = &clients[params.correlatedVertices[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // 仅评估自上次移动评估以来已修改的路线中涉及的移动，用于节点nodeU
				{
					// 在这个循环中随机化邻域的顺序并不十分重要，因为我们已经随机化了节点对的顺序（对于同一节点对，找到不同类型的改进移动并不常见）。
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
                    // 后验证1-前
                    std::set < int > q_emptyRoutes;
                    std::vector < Node > q_clients;
                    std::vector < Node > q_depots;
                    std::vector < Node > q_depotsEnd;
                    std::vector < Route > q_routes;
                    std::vector < std::vector < ThreeBestInsert > > q_bestInsertClient;
                    double cost_u= 0;
                    double cost_v= 0;
                    if(hou==1)
                    {
                        q_emptyRoutes=emptyRoutes;
                        q_clients=clients;
                        q_depots=depots;
                        q_depotsEnd=depotsEnd;
                        q_routes=routes;
                        q_bestInsertClient=bestInsertClient;
                        cost_u= calRouteCharge(routeU);
                        cost_v= calRouteCharge(routeV);
                    }
                    
     
					if (move1()) continue; // RELOCATE
					if (move2()) continue; // RELOCATE
					if (move3()) continue; // RELOCATE
					if (nodeUIndex <= nodeVIndex && move4()) continue; // SWAP
					if (move5()) continue; // SWAP
					if (nodeUIndex <= nodeVIndex && move6()) continue; // SWAP
					if (intraRouteMove && move7()) continue; // 2-OPT
					if (!intraRouteMove && move8()) continue; // 2-OPT*
					if (!intraRouteMove && move9()) continue; // 2-OPT*

					// 尝试将节点nodeU直接插入到仓库之后的移动
					if (nodeV->prev->isDepot)
					{
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						if (move1()) continue; // RELOCATE
						if (move2()) continue; // RELOCATE
						if (move3()) continue; // RELOCATE
						if (!intraRouteMove && move8()) continue; // 2-OPT*
						if (!intraRouteMove && move9()) continue; // 2-OPT*
					}
                    // 后验证1-后
                    if(hou==1)
                    {
                        double cost_u2=calRouteCharge(routeU);
                        double cost_v2=calRouteCharge(routeV);
                        if(cost_u2+cost_v2-cost_u-cost_v>-MY_EPSILON)
                        {
                            this->emptyRoutes=q_emptyRoutes;
                            this->clients=q_clients;
                            this->depots=q_depots;
                            this->depotsEnd=q_depotsEnd;
                            this->routes=q_routes;
                            this->bestInsertClient=q_bestInsertClient;
                        }
                    }
				}
    
			}

			/* 涉及空路径的移动，在第一个循环中未经测试，以避免过度增加车队规模。 */
			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
    
				if (move1()) continue; // RELOCATE
				if (move2()) continue; // RELOCATE
				if (move3()) continue; // RELOCATE
				if (move9()) continue; // 2-OPT*
    
			}
   
		}

		if (params.ap.useSwapStar == 1 && params.areCoordinatesProvided)
		{
			/* （SWAP*）移动限制在圆形扇区重叠的路径对之间 */
			for (int rU = 0; rU < params.nbVehicles; rU++)
			{
				routeU = &routes[orderRoutes[rU]];
				int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
				routeU->whenLastTestedSWAPStar = nbMoves;
				for (int rV = 0; rV < params.nbVehicles; rV++)
				{
					routeV = &routes[orderRoutes[rV]];
					if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour
						&& (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified)
							> lastTestSWAPStarRouteU))
						if (CircleSector::overlap(routeU->sector, routeV->sector))
							swapStar();
				}
			}
		}
        // 后验证2
	}

	// 将局部搜索产生的解决方案注册到个体中。
	exportIndividual(indiv);
}

void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;
    nodeUp = nodeU->prev;
	nodeX = nodeU->next;
    nodeXn = nodeX->next;
	nodeXNextIndex = nodeX->next->cour;
	nodeUIndex = nodeU->cour;
	nodeUPrevIndex = nodeU->prev->cour;
	nodeXIndex = nodeX->cour;
	loadU    = params.cli[nodeUIndex].demand;
	serviceU = params.cli[nodeUIndex].serviceDuration;
	loadX	 = params.cli[nodeXIndex].demand;
	serviceX = params.cli[nodeXIndex].serviceDuration;
}

void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
    nodeVp = nodeV->prev;
	nodeY = nodeV->next;
    nodeYn = nodeY->next;
	nodeYNextIndex = nodeY->next->cour;
	nodeVIndex = nodeV->cour;
	nodeVPrevIndex = nodeV->prev->cour;
	nodeYIndex = nodeY->cour;
	loadV    = params.cli[nodeVIndex].demand;
	serviceV = params.cli[nodeVIndex].serviceDuration;
	loadY	 = params.cli[nodeYIndex].demand;
	serviceY = params.cli[nodeYIndex].serviceDuration;
	intraRouteMove = (routeU == routeV);
}

bool LocalSearch::move1()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];
    // 路径剪枝
	if (!intraRouteMove) // 如果不在同一路径
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU)
			+ penaltyExcessLoad(routeU->load - loadU)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU)
			- routeV->penalty;
	}
 
	if (costSuppU + costSuppV > -MY_EPSILON) return false;  // 如果VRP路径变长了（惩罚的角度）
 
	if (nodeUIndex == nodeYIndex) return false; // 如果u本来就在v后面了
    
    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            remove_f3(R_u,{nodeUIndex});
            add_f3(R_v,{nodeUIndex},nodeVIndex);
            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;

            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            remove_f3(R_u,{nodeUIndex});
            add_f3(R_u,{nodeUIndex},nodeVIndex);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    // 这里是已经决定更改了
	insertNode(nodeU, nodeV);   // 更改"受影响节点"的前后指针
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=1 && checkSelected(1))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    
    }
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move2()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;

    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            remove_f3(R_u,{nodeUIndex,nodeXIndex});
            add_f3(R_v,{nodeUIndex,nodeXIndex},nodeVIndex);
            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;

            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            remove_f3(R_u,{nodeUIndex,nodeXIndex});
            add_f3(R_u,{nodeUIndex,nodeXIndex},nodeVIndex);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=2 && checkSelected(2))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                insertNode(nodeX,nodeU);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                insertNode(nodeX,nodeU);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    }
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move3()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeXIndex] + params.timeCost[nodeXIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            remove_f3(R_u,{nodeUIndex,nodeXIndex});
            add_f3(R_v,{nodeXIndex,nodeUIndex},nodeVIndex);
            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;

            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            remove_f3(R_u,{nodeUIndex,nodeXIndex});
            add_f3(R_u,{nodeXIndex,nodeUIndex},nodeVIndex);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=3 && checkSelected(3))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                insertNode(nodeX,nodeU);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                insertNode(nodeU,nodeUp);
                insertNode(nodeX,nodeU);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    }
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move4()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU + serviceV - serviceU)
			+ penaltyExcessLoad(routeU->load + loadV - loadU)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV - serviceV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_v.begin(), R_v.end(), nodeVIndex);
            iter_swap(it_u, it_v);
            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_u.begin(), R_u.end(), nodeVIndex);
            iter_swap(it_u, it_v);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
	swapNode(nodeU, nodeV);
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=4 && checkSelected(4))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    }
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move5()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] + serviceV - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] - serviceV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;

    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_v.begin(), R_v.end(), nodeVIndex);
            iter_swap(it_u, it_v);
            remove_f3(R_u, { nodeXIndex });
            add_f3(R_v,{ nodeXIndex },nodeUIndex);

            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_u.begin(), R_u.end(), nodeVIndex);
            iter_swap(it_u, it_v);
            remove_f3(R_u, { nodeXIndex });
            add_f3(R_u,{ nodeXIndex },nodeUIndex);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
    
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=5 && checkSelected(5))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                insertNode(nodeX, nodeU);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                insertNode(nodeX, nodeU);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    }
    
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move6()
{
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeYIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYNextIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeYIndex][nodeYNextIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] + params.timeCost[nodeVIndex][nodeYIndex] + serviceV + serviceY - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV + loadY - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex] - serviceV - serviceY + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV - loadY)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

    if(yuC)
    {
        vector<int> R_u=routeU->rrr;
        vector<int> R_v=routeV->rrr;
        if(!intraRouteMove)
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_v.begin(), R_v.end(), nodeVIndex);
            auto it_x=find(R_u.begin(), R_u.end(), nodeXIndex);
            auto it_y = find(R_v.begin(), R_v.end(), nodeYIndex);
            iter_swap(it_u, it_v);
            iter_swap(it_x,it_y);
            double dian2= focusEnumeration(R_u, params.c_evrp).second;
            double dian3= focusEnumeration(R_v, params.c_evrp).second;
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
                return false;
        }
        else
        {
            auto it_u = find(R_u.begin(), R_u.end(), nodeUIndex);
            auto it_v = find(R_u.begin(), R_u.end(), nodeVIndex);
            auto it_x=find(R_u.begin(), R_u.end(), nodeXIndex);
            auto it_y = find(R_u.begin(), R_u.end(), nodeYIndex);
            iter_swap(it_u, it_v);
            iter_swap(it_x,it_y);
            double dian4= focusEnumeration(R_u, params.c_evrp).second;
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
                return false;
        }
    }
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(yu2 && numMoves>=6 && checkSelected(6))
    {
        if(!intraRouteMove)
        {
            double dian2= calRouteCharge(routeU);
            double dian3= calRouteCharge(routeV);
            if((dian2+dian3-routeU->fit_charge-routeV->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                swapNode(nodeX, nodeY);
                auto r9=seeRoute(routeU);
                auto r10=seeRoute(routeV);
                if(isEqual(r5,r9)==false||isEqual(r6,r10)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
        else
        {
            double dian4=calRouteCharge(routeU);
            if((dian4-routeU->fit_charge)>-MY_EPSILON)
            {
                swapNode(nodeU, nodeV);
                swapNode(nodeX, nodeY);
                auto r11=seeRoute(routeU);
                if(isEqual(r5,r11)==false)
                {
                    cout<<endl<<"错错错，是我的错"<<endl;
                }
                return false;
            }
        }
    }
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move7()
{
	if (nodeU->position > nodeV->position) return false;

	double cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex] + nodeV->cumulatedReversalDistance - nodeX->cumulatedReversalDistance;

	if (cost > -MY_EPSILON) return false;

	if (nodeU->next == nodeV) return false;
    
    auto R_u=routeU->rrr;
    if(yu2 && numMoves>=7 && checkSelected(7))
    {
        
        int lhand=nodeXIndex,rhand=nodeVIndex;

        auto it_l = find(R_u.begin(), R_u.end(), lhand);
        auto it_r = find(R_u.begin(), R_u.end(), rhand);
        if (it_l < it_r)
            std::reverse(it_l, it_r + 1);
        else
            std::reverse(it_r, it_l + 1);
        double dian2= focusEnumeration(R_u, params.c_evrp).second;
        if(dian2-(routeU->fit_charge)>-MY_EPSILON)
            return false;
    }
    
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    // 反转线路
	Node * nodeNum = nodeX->next;
	nodeX->prev = nodeNum;
	nodeX->next = nodeY;
	while (nodeNum != nodeV)
	{
		Node * temp = nodeNum->next;
		nodeNum->next = nodeNum->prev;
		nodeNum->prev = temp;
		nodeNum = temp;
	}
 

	nodeV->next = nodeV->prev;
	nodeV->prev = nodeU;
	nodeU->next = nodeV;
	nodeY->prev = nodeX;
    
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(isWarn==true && isEqual(R_u,r7)==false)
        cout<<"\n\n"<<"错错错"<<"\n\n";
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	return true;
}

bool LocalSearch::move8()
{
	double cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex]
		+ nodeV->cumulatedReversalDistance + routeU->reversalDistance - nodeX->cumulatedReversalDistance
		- routeU->penalty - routeV->penalty;

	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= 0) return false;
		
	cost += penaltyExcessDuration(nodeU->cumulatedTime + nodeV->cumulatedTime + nodeV->cumulatedReversalDistance + params.timeCost[nodeUIndex][nodeVIndex])
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params.timeCost[nodeUIndex][nodeXIndex] + routeU->reversalDistance - nodeX->cumulatedReversalDistance + routeV->duration - nodeV->cumulatedTime - params.timeCost[nodeVIndex][nodeYIndex] + params.timeCost[nodeXIndex][nodeYIndex])
		+ penaltyExcessLoad(nodeU->cumulatedLoad + nodeV->cumulatedLoad)
		+ penaltyExcessLoad(routeU->load + routeV->load - nodeU->cumulatedLoad - nodeV->cumulatedLoad);
		
	if (cost > -MY_EPSILON) return false;
    
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    
    auto it_u=find(r5.begin(), r5.end(),nodeUIndex);
    auto it_x=find(r5.begin()+1, r5.end(),nodeXIndex);
    auto it_v=find(r6.begin(), r6.end(),nodeVIndex);
    auto it_y=find(r6.begin()+1, r6.end(),nodeYIndex);
    vector<int> new_r5(r5.begin(),it_x);
    new_r5.insert(new_r5.end(),reverse_iterator<decltype(it_u)>(it_v+1),
                  reverse_iterator<decltype(it_u)>(r6.begin()));
    vector<int> new_r6(reverse_iterator<decltype(it_u)>(r5.end()),
                       reverse_iterator<decltype(it_u)>(it_x));
    new_r6.insert(new_r6.end(),it_y,r6.end());
    if(yu2 && numMoves>=8 && checkSelected(8))
    {
        double dian3= focusEnumeration(new_r5, params.c_evrp).second;
        double dian4= focusEnumeration(new_r6, params.c_evrp).second;
        if(dian3+dian4-routeU->fit_charge-routeV->fit_charge>-MY_EPSILON)
            return false;
        
    }
    
	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = routeU->depot->prev;
	Node * depotVFin = routeV->depot->prev;
	Node * depotVSuiv = depotV->next;

	Node * temp;
	Node * xx = nodeX;
	Node * vv = nodeV;

	while (!xx->isDepot)
	{
		temp = xx->next;
		xx->next = xx->prev;
		xx->prev = temp;
		xx->route = routeV;
		xx = temp;
	}

	while (!vv->isDepot)
	{
		temp = vv->prev;
		vv->prev = vv->next;
		vv->next = temp;
		vv->route = routeU;
		vv = temp;
	}

	nodeU->next = nodeV;
	nodeV->prev = nodeU;
	nodeX->next = nodeY;
	nodeY->prev = nodeX;

	if (nodeX->isDepot)
	{
		depotUFin->next = depotU;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
		depotV->next = nodeY;
		nodeY->prev = depotV;
	}
	else if (nodeV->isDepot)
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotV->prev = depotVFin;
		depotUFin->prev = nodeU;
		nodeU->next = depotUFin;
	}
	else
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
	}
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(isWarn==true && (isEqual(new_r5,r7)==false|| isEqual(new_r6,r8)==false))
        cout<<endl<<"错错错"<<endl;
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

bool LocalSearch::move9()
{
	double cost = params.timeCost[nodeUIndex][nodeYIndex] + params.timeCost[nodeVIndex][nodeXIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex]
		        - routeU->penalty - routeV->penalty;

	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= 0) return false;
		
	cost += penaltyExcessDuration(nodeU->cumulatedTime + routeV->duration - nodeV->cumulatedTime - params.timeCost[nodeVIndex][nodeYIndex] + params.timeCost[nodeUIndex][nodeYIndex])
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params.timeCost[nodeUIndex][nodeXIndex] + nodeV->cumulatedTime + params.timeCost[nodeVIndex][nodeXIndex])
		+ penaltyExcessLoad(nodeU->cumulatedLoad + routeV->load - nodeV->cumulatedLoad)
		+ penaltyExcessLoad(nodeV->cumulatedLoad + routeU->load - nodeU->cumulatedLoad);

	if (cost > -MY_EPSILON) return false;
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    auto it_u=find(r5.begin(), r5.end(),nodeUIndex);
    auto it_x=find(r5.begin()+1, r5.end(),nodeXIndex);
    auto it_v=find(r6.begin(), r6.end(),nodeVIndex);
    auto it_y=find(r6.begin()+1, r6.end(),nodeYIndex);
    vector<int> new_r5(r5.begin(),it_x);
    new_r5.insert(new_r5.end(),it_y,r6.end());
    vector<int> new_r6(r6.begin(),it_y);
    new_r6.insert(new_r6.end(),it_x,r5.end());
    if(yu2 && numMoves>=9 && checkSelected(9))
    {
        double dian3= focusEnumeration(new_r5, params.c_evrp).second;
        double dian4= focusEnumeration(new_r6, params.c_evrp).second;
        if(dian3+dian4-routeU->fit_charge-routeV->fit_charge>-MY_EPSILON)
            return false;
        
    }
    
	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = depotU->prev;
	Node * depotVFin = depotV->prev;
	Node * depotUpred = depotUFin->prev;
	Node * count = nodeY;
	while (!count->isDepot)
	{
		count->route = routeU;
		count = count->next;
	}
	count = nodeX;
	while (!count->isDepot)
	{
		count->route = routeV;
		count = count->next;
	}
	nodeU->next = nodeY;
	nodeY->prev = nodeU;
	nodeV->next = nodeX;
	nodeX->prev = nodeV;
	if (nodeX->isDepot)
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		nodeV->next = depotVFin;
		depotVFin->prev = nodeV;
	}
	else
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		depotVFin->prev = depotUpred;
		depotVFin->prev->next = depotVFin;
	}
    
    
    auto r7=seeRoute(routeU);
    auto r8=seeRoute(routeV);
    if(isWarn==true && (isEqual(new_r5,r7)==false|| isEqual(new_r6,r8)==false))
        cout<<endl<<"错错错"<<endl;
    
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}
// (SWAP*) 计算路线U和路线V之间的所有SWAP*，并应用最佳改进移动
bool LocalSearch::swapStar()
{
	SwapStarElement myBestSwapStar; // 用于记录当前最佳 SWAP* 移动的对象。

	// 对两条路线进行插入成本的预处理，以便更快地评估潜在的SWAP*移动。
	preprocessInsertions(routeU, routeV);
	preprocessInsertions(routeV, routeU);

	// 遍历路线U和V上的每个节点，评估所有可能的SWAP*移动。
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
		{
            // 计算交换操作对路线U和V负载惩罚的影响。
			double deltaPenRouteU = penaltyExcessLoad(routeU->load + params.cli[nodeV->cour].demand - params.cli[nodeU->cour].demand) - routeU->penalty;
			double deltaPenRouteV = penaltyExcessLoad(routeV->load + params.cli[nodeU->cour].demand - params.cli[nodeV->cour].demand) - routeV->penalty;

			// 快速筛选：基于容量约束和插入成本上限，可能提前排除许多SWAP*移动。
			if (deltaPenRouteU + nodeU->deltaRemoval + deltaPenRouteV + nodeV->deltaRemoval <= 0)
			{
				SwapStarElement mySwapStar;
				mySwapStar.U = nodeU;
				mySwapStar.V = nodeV;

				// 评估将U插入到V所在路线（在移除V后）的最佳重插入成本。
				double extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);

				// 评估将V插入到U所在路线（在移除U后）的最佳重插入成本。
				double extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);

				// 计算总的移动成本。
				mySwapStar.moveCost = deltaPenRouteU + nodeU->deltaRemoval + extraU + deltaPenRouteV + nodeV->deltaRemoval + extraV
					+ penaltyExcessDuration(routeU->duration + nodeU->deltaRemoval + extraU + params.cli[nodeV->cour].serviceDuration - params.cli[nodeU->cour].serviceDuration)
					+ penaltyExcessDuration(routeV->duration + nodeV->deltaRemoval + extraV - params.cli[nodeV->cour].serviceDuration + params.cli[nodeU->cour].serviceDuration);
                // 如果找到了更优的移动方案，更新最佳移动。
				if (mySwapStar.moveCost < myBestSwapStar.moveCost)
					myBestSwapStar = mySwapStar;
			}
		}
	}
    
    // 包括从nodeU到routeV的RELOCATE（在这一步包含评估成本不高，因为我们已经有了最佳插入位置）
    // 此外，由于粒度标准不同，这可能导致不同的改进移动
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.U = nodeU;
		mySwapStar.bestPositionU = bestInsertClient[routeV->cour][nodeU->cour].bestLocation[0];
		double deltaDistRouteU = params.timeCost[nodeU->prev->cour][nodeU->next->cour] - params.timeCost[nodeU->prev->cour][nodeU->cour] - params.timeCost[nodeU->cour][nodeU->next->cour];
		double deltaDistRouteV = bestInsertClient[routeV->cour][nodeU->cour].bestCost[0];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load - params.cli[nodeU->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load + params.cli[nodeU->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU - params.cli[nodeU->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV + params.cli[nodeU->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	// 包括从nodeV到routeU的RELOCATE
	for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.V = nodeV;
		mySwapStar.bestPositionV = bestInsertClient[routeU->cour][nodeV->cour].bestLocation[0];
		double deltaDistRouteU = bestInsertClient[routeU->cour][nodeV->cour].bestCost[0];
		double deltaDistRouteV = params.timeCost[nodeV->prev->cour][nodeV->next->cour] - params.timeCost[nodeV->prev->cour][nodeV->cour] - params.timeCost[nodeV->cour][nodeV->next->cour];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load + params.cli[nodeV->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load - params.cli[nodeV->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU + params.cli[nodeV->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV - params.cli[nodeV->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

    // 没有改进，返回false
    if (myBestSwapStar.moveCost > -MY_EPSILON) return false;
    
    auto r5=seeRoute(routeU);
    auto r6=seeRoute(routeV);
    auto new_r5=r5;
    auto new_r6=r6;
    if(yu2 && numMoves>=10 && checkSelected(10))
    {
        if (myBestSwapStar.bestPositionU != NULL)
        {
            remove_f3(new_r5,{myBestSwapStar.U->cour});
            add_f3(new_r6,{myBestSwapStar.U->cour},myBestSwapStar.bestPositionU->cour);
            
        }
        if (myBestSwapStar.bestPositionV != NULL)
        {
            remove_f3(new_r6,{myBestSwapStar.V->cour});
            add_f3(new_r5,{myBestSwapStar.V->cour},myBestSwapStar.bestPositionV->cour);
        }
        double dian3= focusEnumeration(new_r5, params.c_evrp).second;
        double dian4= focusEnumeration(new_r6, params.c_evrp).second;
        if(dian3+dian4-routeU->fit_charge-routeV->fit_charge>-MY_EPSILON)
            return false;
    }
    
    
    // 如果有改进，则应用最佳移动
    if (myBestSwapStar.bestPositionU != NULL) insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
    if (myBestSwapStar.bestPositionV != NULL) insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
    
    auto r9=seeRoute(routeU);
    auto r10=seeRoute(routeV);
    if(isWarn==true && (isEqual(new_r5,r9)==false||isEqual(new_r6,r10)==false))
    {
        cout<<endl<<"错错错，是我的错"<<endl;
    }
    
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    updateRouteData(routeV);
    return true;
}
// (SWAP*) 计算V所在路线中的插入成本和位置，其中V被省略
double LocalSearch::getCheapestInsertSimultRemoval(Node * U, Node * V, Node *& bestPosition)
{
    // 获取关于节点V路线中节点U的三个最佳插入位置的预处理信息
	ThreeBestInsert * myBestInsert = &bestInsertClient[V->route->cour][U->cour];
	bool found = false;
    
    // 检查这三个最佳位置，并找到一个既不是V也不紧邻V的位置
    // 循环遍历这三个位置，选择成本最低且符合条件的位置作为最佳插入位置
	bestPosition = myBestInsert->bestLocation[0];
	double bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
    // 如果第一个位置不符合条件，检查第二个和第三个位置
    // ...
	if (!found && myBestInsert->bestLocation[1] != NULL)
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != NULL)
		{
			bestPosition = myBestInsert->bestLocation[2];
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}
    
    // 计算将U直接插入到V的位置的成本，如果这个成本更低，则选择这个位置作为最佳位置
	double deltaCost = params.timeCost[V->prev->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->prev->cour][V->next->cour];
	if (!found || deltaCost < bestCost)
	{
		bestPosition = V->prev;
		bestCost = deltaCost;
	}

	return bestCost;
}
// (SWAP*) 函数用于预处理路线R1中所有节点在路线R2中的插入成本
void LocalSearch::preprocessInsertions(Route * R1, Route * R2)
{
	for (Node * U = R1->depot->next; !U->isDepot; U = U->next)
	{
        // 计算并存储从其当前位置移除节点U的成本。
		U->deltaRemoval = params.timeCost[U->prev->cour][U->next->cour] - params.timeCost[U->prev->cour][U->cour] - params.timeCost[U->cour][U->next->cour];
        // 如果路线R2自上次计算以来已被修改，则重新计算U在R2中的插入成本。
        if (R2->whenLastModified > bestInsertClient[R2->cour][U->cour].whenLastCalculated)
		{
            // 重置相关数据结构，为新的计算做准备。
			bestInsertClient[R2->cour][U->cour].reset();
			bestInsertClient[R2->cour][U->cour].whenLastCalculated = nbMoves;
            // 计算U插入到R2开头的成本，并更新最佳插入位置。
			bestInsertClient[R2->cour][U->cour].bestCost[0] = params.timeCost[0][U->cour] + params.timeCost[U->cour][R2->depot->next->cour] - params.timeCost[0][R2->depot->next->cour];
			bestInsertClient[R2->cour][U->cour].bestLocation[0] = R2->depot;
            // 遍历R2的每个节点V，评估将U插入到V之后的成本，并更新最佳插入位置。
			for (Node * V = R2->depot->next; !V->isDepot; V = V->next)
			{
				double deltaCost = params.timeCost[V->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->cour][V->next->cour];
				bestInsertClient[R2->cour][U->cour].compareAndAdd(deltaCost, V);
                //找到3最优点
			}
		}
	}
}
// 解决方案更新：在V之后插入U
void LocalSearch::insertNode(Node * U, Node * V)
{
	U->prev->next = U->next;
	U->next->prev = U->prev;
	V->next->prev = U;
	U->prev = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}
// 解决方案更新：交换U和V
void LocalSearch::swapNode(Node * U, Node * V)
{
	Node * myVPred = V->prev;
	Node * myVSuiv = V->next;
	Node * myUPred = U->prev;
	Node * myUSuiv = U->next;
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}


//在vector中移除一些元素
void LocalSearch::remove_f3(vector<int>& g, vector<int> shanChu)
{
    for (auto h : shanChu)
    {
        auto it2 = find(g.begin(), g.end(), h);
        if (it2 != g.end())
        {
            g.erase(it2);
        }
    }
}
//在vector中的某位置pos后加入一些元素
void LocalSearch::add_f3(vector<int>& g, vector<int> add, int after)
{
    auto it2 = find(g.begin(), g.end(), after);
    if (it2 != g.end())
    {
        g.insert(it2 + 1, add.begin(), add.end());

    }


}




// 更新路线的预处理数据
void LocalSearch::updateRouteData(Route * myRoute)
{
	int myplace = 0;
	double myload = 0.;
	double mytime = 0.;
	double myReversalDistance = 0.;
	double cumulatedX = 0.;
	double cumulatedY = 0.;
//修改的地方
    vector<int> temp_route({0});
	Node * mynode = myRoute->depot;
	mynode->position = 0;
	mynode->cumulatedLoad = 0.;
	mynode->cumulatedTime = 0.;
	mynode->cumulatedReversalDistance = 0.;

	bool firstIt = true;
	while (!mynode->isDepot || firstIt)
	{
		mynode = mynode->next;
		myplace++;
		mynode->position = myplace;
		myload += params.cli[mynode->cour].demand;
		mytime += params.timeCost[mynode->prev->cour][mynode->cour] + params.cli[mynode->cour].serviceDuration;
        temp_route.push_back(mynode->cour);//修改的地方
		myReversalDistance += params.timeCost[mynode->cour][mynode->prev->cour] - params.timeCost[mynode->prev->cour][mynode->cour] ;
		mynode->cumulatedLoad = myload;
		mynode->cumulatedTime = mytime;
		mynode->cumulatedReversalDistance = myReversalDistance;
		if (!mynode->isDepot)
		{
			cumulatedX += params.cli[mynode->cour].coordX;
			cumulatedY += params.cli[mynode->cour].coordY;
			if (firstIt) myRoute->sector.initialize(params.cli[mynode->cour].polarAngle);
			else myRoute->sector.extend(params.cli[mynode->cour].polarAngle);
		}
		firstIt = false;
	}
    // 更新路线持续时间、负载、罚分、顾客数量、反转后距离
	myRoute->duration = mytime;
	myRoute->load = myload;
	myRoute->penalty = penaltyExcessDuration(mytime) + penaltyExcessLoad(myload);
	myRoute->nbCustomers = myplace-1;
	myRoute->reversalDistance = myReversalDistance;
	// 更新这条路线最后一次修改的时间 (将会用在过滤不重要移动的评估)
	myRoute->whenLastModified = nbMoves ;
    
    // 如果路线没有顾客，则更新该路线在空路线集合中的状态
	if (myRoute->nbCustomers == 0)
	{
		myRoute->polarAngleBarycenter = 1.e30; // 设置一个非常大的角度值，代表没有重心角
		emptyRoutes.insert(myRoute->cour); // 将路线加入空路线集合
	}
	else
	{
		myRoute->polarAngleBarycenter = atan2(cumulatedY/(double)myRoute->nbCustomers - params.cli[0].coordY, cumulatedX/(double)myRoute->nbCustomers - params.cli[0].coordX);
		emptyRoutes.erase(myRoute->cour);
	}
    //修改的地方
    myRoute->rrr=temp_route;
    if(temp_route.size() <= 2)
        myRoute->fit_charge=INT_MAX;
    else
        myRoute->fit_charge= focusEnumeration(temp_route, params.c_evrp).second;
}

double LocalSearch::calRouteCharge(Route * myRoute)
{
    vector<int> r2({0});
    Node * mynode = myRoute->depot;
    bool firstIt = true;
    while (!mynode->isDepot || firstIt)
    {
        mynode = mynode->next;
        r2.push_back(mynode->cour);

        firstIt = false;
    }
    auto fitness_evrp= focusEnumeration(r2, params.c_evrp).second;
    return fitness_evrp;

}
vector<int> LocalSearch::seeRoute(Route * myRoute)
{
    vector<int> r2({0});
    Node * mynode = myRoute->depot;
    bool firstIt = true;
    while (!mynode->isDepot || firstIt)
    {
        mynode = mynode->next;
        r2.push_back(mynode->cour);

        firstIt = false;
    }
    return r2;
}
bool LocalSearch::isEqual(const std::vector<int>& v1, const std::vector<int>& v2) {
    if (v1.size() != v2.size()) {
        return false; // 大小不同，直接返回false
    }
    return std::equal(v1.begin(), v1.end(), v2.begin());
}
// 将解决方案加载到LS中
void LocalSearch::loadIndividual(const Individual & indiv)
{
	emptyRoutes.clear();
	nbMoves = 0; 
	for (int r = 0; r < params.nbVehicles; r++) //nbVeh是由系统决定一个足够大的值
	{
		Node * myDepot = &depots[r];
		Node * myDepotFin = &depotsEnd[r];
		Route * myRoute = &routes[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		if (!indiv.chromR[r].empty())
		{
			Node * myClient = &clients[indiv.chromR[r][0]];
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;
			for (int i = 1; i < (int)indiv.chromR[r].size(); i++)
			{
				Node * myClientPred = myClient;
				myClient = &clients[indiv.chromR[r][i]]; 
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}
		updateRouteData(&routes[r]);
		routes[r].whenLastTestedSWAPStar = -1;
		for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
			bestInsertClient[r][i].whenLastCalculated = -1;
	}

	for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
		clients[i].whenLastTestedRI = -1;
}
// 将LS解决方案导出到个体，并根据Params中的原始惩罚权重计算惩罚成本
void LocalSearch::exportIndividual(Individual & indiv)
{
	std::vector < std::pair <double, int> > routePolarAngles ;
	for (int r = 0; r < params.nbVehicles; r++)
		routePolarAngles.push_back(std::pair <double, int>(routes[r].polarAngleBarycenter, r));
	std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

	int pos = 0;
	for (int r = 0; r < params.nbVehicles; r++)
	{
		indiv.chromR[r].clear();
		Node * node = depots[routePolarAngles[r].second].next;
		while (!node->isDepot)
		{
			indiv.chromT[pos] = node->cour;
			indiv.chromR[r].push_back(node->cour);
			node = node->next;
			pos++;
		}
	}

	indiv.evaluateCompleteCost(params);
}
bool LocalSearch::checkSelected(int m)
{
    if(set2.empty())
        return true;
    else
    {
        return (bool)set2.count(m);
    }
}
LocalSearch::LocalSearch(Params & params) : params (params),dis3(0,1)
{
	clients = std::vector < Node >(params.nbClients + 1);   // 客户+哨兵
	routes = std::vector < Route >(params.nbVehicles);  // 表示路线的元素
	depots = std::vector < Node >(params.nbVehicles);   // 表示仓库的元素
	depotsEnd = std::vector < Node >(params.nbVehicles);// 复制仓库以标记路线的终点
	bestInsertClient = std::vector < std::vector <ThreeBestInsert> >(params.nbVehicles, std::vector <ThreeBestInsert>(params.nbClients + 1));// (SWAP*) 对于每个路线和节点，存储最便宜的插入成本
    // 修改的部分
    yu2=params.preCharge;   // 是否启用预充电策略
    isWarn=0;
    hou=params.hou;         // 后验证
    if(params.numMoves<=0)
        numMoves=88;
    else
        numMoves=params.numMoves;
    for (char ch : params.selected)
    {
        set2.insert(ch - '0');
    }
	for (int i = 0; i <= params.nbClients; i++) 
	{
        // 0是哨兵
		clients[i].cour = i; 
		clients[i].isDepot = false; 
	}
	for (int i = 0; i < params.nbVehicles; i++)
	{
        // 路径初始化
		routes[i].cour = i;
		routes[i].depot = &depots[i];
        // 仓库节点初始化
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];

		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
    // 加入客户节点
	for (int i = 1 ; i <= params.nbClients ; i++) orderNodes.push_back(i);
    // 加入路线
	for (int r = 0 ; r < params.nbVehicles ; r++) orderRoutes.push_back(r);
}

