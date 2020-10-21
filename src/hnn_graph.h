/*
 * File name: hnn_graph.h
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#ifndef __HNN_GRAPH_H__
#define __HNN_GRAPH_H__

#include <vector>
#include <cmath>

#include "target.h"

#define SOCP 0
#define GEOM 1

typedef std::vector<int> IntVector;
typedef std::vector<std::vector<std::vector<double>>> Double3Vector;
typedef std::vector<std::vector<std::vector<Coords>>> Coords3Vector;

namespace hnn
{
    class CGraph
    {
    public:
        CGraph(const int sDepot, const int eDepot, const int budget, TargetPtrVector &targets, const std::string heuristics);
        ~CGraph();
        void init(const std::string &problem, bool compute);
        double getDistance(int from, int middle, int to);
        double getDistance(int from, int to) const;
        double getCenterDistance(int from, int to) const;
        double getOptimalDist(int from, int middle, int to, Coords &waypoint);
        double getReward(int label) const { return targets[label]->reward; }
        STarget *getLocation(int index) const { return targets[index]; }
        TargetPtrVector getTargets() const { return targets; }

    private:
        double distPointToSegment(const Coords &c, const Coords &a, const Coords &b, Coords &p);

    public:
        const int SDEPOT;
        const int EDEPOT;
        const int budget;
        const int size;

    private:
        TargetPtrVector targets;
        Double3Vector distMat;
        Coords3Vector waypoints;
        int heuristic;
    };

} // end namespace hnn
#endif //__HNN_GRAPH_H__