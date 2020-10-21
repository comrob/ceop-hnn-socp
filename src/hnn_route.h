/*
 * File name: hnn_route.h
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <iostream>
#include <vector>

#include "target.h"
#include "hnn_graph.h"
#include "hnn_nn.h"

typedef std::vector<int> IntVector;
typedef std::vector<bool> BoolVector;

namespace hnn
{

	class CRoute
	{
	public:
		CRoute(int size, const CGraph *graph);
		~CRoute();
		void constructRoute(const Double2Vector nodes, const int size);
		void improveRoute();
		void examineRoute();

		STarget *getRouteLocation(int i) const { return routeLocations[i]; }
		void setRouteLocation(int index, int value);
		void setRoute(const TargetPtrVector &newRoute);
		void setSize(int size);
		int getSize() const { return routeSize; }
		TargetPtrVector getRoute() const { return routeLocations; }
		double getDistance() const;
		double getReward() const;
		double getReward(const TargetPtrVector &targets) const;
		void twoOpt(TargetPtrVector &path);

	private:
		void deleteDuplicates();
		double getDistanceOnRoute(TargetPtrVector route);
		void twoOpt(TargetPtrVector &route, int i, int j);
		int getIdxToRemoveByScoreRatio(TargetPtrVector route);
		void getCheapestDistanceIndex(TargetPtrVector route, int &fromIdx, int &toLoc);
		void deleteLocation(TargetPtrVector &route, int remIdx);
		void insertLocation(TargetPtrVector &route, int fromIdx, int toLoc);
		double optimizeWaypoints(TargetPtrVector &route);
		double distPointToSegment(const Coords &center, const Coords &a, const Coords &b, Coords &p, char &s) const;

	private:
		double getDistance(const Coords &a, const Coords &b) const { return sqrt(a.squared_distance(b)); }

	private:
		int routeSize;									
		TargetPtrVector routeLocations; 
		const CGraph *graph;						
	};

} // namespace hnn

#endif // __ROUTE_H__
