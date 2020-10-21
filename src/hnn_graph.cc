/*
 * File name: hnn_graph.cc
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#include "hnn_graph.h"

#include <cmath>
#include <sstream>

#include <crl/logging.h>
#include <ilcplex/ilocplex.h>

#define CONVTOL 1e-12

ILOSTLBEGIN

using crl::logger;

using namespace std;
using namespace hnn;

/// - constructor ---------------------------------------------------------------
CGraph::CGraph(const int sDepot, const int eDepot, const int budget, TargetPtrVector &targets, const std::string heuristics) : SDEPOT(sDepot), EDEPOT(eDepot), budget(budget), targets(targets),
                                                                                                                               size(targets.size())
{
    if (heuristics == "socp")
    {
        heuristic = SOCP;
    }
    else
    {
        heuristic = GEOM;
    }

    distMat.resize(size);
    waypoints.resize(size);
    for (int i = 0; i < size; i++)
    {
        distMat[i].resize(size);
        waypoints[i].resize(size);
        for (int j = 0; j < size; j++)
        {
            distMat[i][j].assign(size, 0.0);
            waypoints[i][j].resize(size);
        }
    }
}

/// - destructor ---------------------------------------------------------------
CGraph::~CGraph()
{
    targets.clear();
}

/// - public method ------------------------------------------------------------
void CGraph::init(const std::string &matrix_file, bool compute)
{
    if (heuristic == SOCP)
    {
        if (compute)
        {
            int cnt = 0, cntAll = 0;
            Coords w;
            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    for (int k = i; k < size; k++)
                    {
                        cntAll++;
                        if (distMat[i][j][k] > 0.0)
                            continue;
                        if (i == j && j == k)
                        {
                            distMat[i][j][i] = 0.0;
                            waypoints[i][j][i] = targets[j]->coords;
                        }
                        else
                        {
                            distMat[i][j][k] = getOptimalDist(i, j, k, w);
                            waypoints[i][j][k] = w;
                            distMat[k][j][i] = distMat[i][j][k];
                            waypoints[k][j][i] = w;
                            cnt++;
                        }
                    }
                }
            }

            DEBUG("Distance matrix computed.");
            ofstream outfile(matrix_file, ios::out);

            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    for (int k = 0; k < size; k++)
                    {
                        outfile << distMat[i][j][k] << " " << waypoints[i][j][k].x << " " << waypoints[i][j][k].y << std::endl;
                    }
                }
            }
            outfile.close();
        }
        else
        {
            ifstream infile(matrix_file, ios::in);
            if (infile.is_open())
            {
                for (int i = 0; i < size; i++)
                {
                    for (int j = 0; j < size; j++)
                    {
                        for (int k = 0; k < size; k++)
                        {
                            infile >> distMat[i][j][k] >> waypoints[i][j][k].x >> waypoints[i][j][k].y;
                        }
                    }
                }
                infile.close();
                DEBUG("Distance matrix loaded from file " << matrix_file);
            }
            else
            {
                WARN("Distance matrix could not be loaded from file.");
                for (int i = 0; i < size; i++)
                {
                    for (int j = 0; j < size; j++)
                    {
                        for (int k = 0; k < size; k++)
                        {
                            if (i == j && j == k)
                            {
                                distMat[i][j][i] = 0.0;
                                waypoints[i][j][i] = targets[j]->coords;
                            }
                            else
                            {
                                distMat[i][j][k] = distPointToSegment(targets[j]->coords, targets[i]->coords, targets[k]->coords, waypoints[i][j][k]);
                            }
                        }
                    }
                }
                DEBUG("Distance matrix calculated.");
            }
        }
    }
    else
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int k = 0; k < size; k++)
                {
                    if (i == j && j == k)
                    {
                        distMat[i][j][i] = 0.0;
                        waypoints[i][j][i] = targets[j]->coords;
                    }
                    else
                    {
                        distMat[i][j][k] = distPointToSegment(targets[j]->coords, targets[i]->coords, targets[k]->coords, waypoints[i][j][k]);
                    }
                }
            }
        }
    }
}

/// - public method ------------------------------------------------------------
double CGraph::getDistance(int from, int middle, int to)
{
    double dist = 0.0;
    if (heuristic == SOCP)
    {
        dist = distMat[from][middle][to];
        targets[middle]->waypoint = waypoints[from][middle][to];
    }
    else if (heuristic == GEOM)
    {
        Coords wtmp;
        dist = distPointToSegment(targets[middle]->coords, targets[from]->waypoint, targets[to]->waypoint, wtmp);
        targets[middle]->waypoint = wtmp;
    }
    if (middle == SDEPOT || middle == EDEPOT)
    {
        targets[middle]->waypoint = targets[middle]->coords;
    }

    return dist;
}

/// - public method ------------------------------------------------------------
double CGraph::getCenterDistance(int from, int to) const
{
    return sqrt(targets[from]->coords.squared_distance(targets[to]->coords));
}

/// - public method ------------------------------------------------------------
double CGraph::getDistance(int from, int to) const
{
    return sqrt(targets[from]->waypoint.squared_distance(targets[to]->waypoint));
}

/// - private method ------------------------------------------------------------
double CGraph::distPointToSegment(const Coords &c, const Coords &a, const Coords &b, Coords &p)
{
    const double r_num = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
    const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    const double r = r_num / r_den;

    if (r >= 0 && r <= 1)
    {
        p.x = a.x + r * (b.x - a.x);
        p.y = a.y + r * (b.y - a.y);

        const double s = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y);
        return sqrt(s * s / r_den);
    }
    const double dist1 = c.squared_distance(a);
    const double dist2 = c.squared_distance(b);
    if (dist1 < dist2)
    {
        p = a;
        return sqrt(dist1);
    }
    p = b;
    return sqrt(dist2);
}

/// - private method ------------------------------------------------------------
double CGraph::getOptimalDist(int from, int middle, int to, Coords &waypoint)
{
    IntVector ids;
    ids.push_back(from);
    ids.push_back(middle);
    ids.push_back(to);
    TargetPtrVector curTargets;
    for (auto id : ids)
    {
        curTargets.push_back(targets[id]);
    }

    int N = (int)curTargets.size();

    double orig_len = 0.0;
    for (int i = 0; i < N - 1; i++)
    {
        orig_len += sqrt(curTargets[i]->waypoint.squared_distance(curTargets[i + 1]->waypoint));
    }

    IloEnv env;
    try
    {
        IloModel model(env);

        IloNumVarArray f(env, N - 1, 0, IloInfinity);
        char var[100];
        for (int i = 0; i < N - 1; i++)
        {
            sprintf(var, "f(%d)", i);
            f[i].setName(var);
            model.add(f[i]);
        }

        IloNumVarArray x(env, N, -IloInfinity, IloInfinity);
        IloNumVarArray y(env, N, -IloInfinity, IloInfinity);

        //variable x
        for (int i = 0; i < N; i++)
        {
            sprintf(var, "x(%d)", i);
            x[i].setName(var);
            model.add(x[i]);
        }

        //variable y
        for (int i = 0; i < N; i++)
        {
            sprintf(var, "y(%d)", i);
            y[i].setName(var);
            model.add(y[i]);
        }

        // set first and last target locations
        IloRange cx0 = x[0] == curTargets[0]->coords.x;
        model.add(cx0);
        IloRange cy0 = y[0] == curTargets[0]->coords.y;
        model.add(cy0);
        IloRange cxn = x[N - 1] == curTargets[N - 1]->coords.x;
        model.add(cxn);
        IloRange cyn = y[N - 1] == curTargets[N - 1]->coords.y;
        model.add(cyn);

        IloNumVarArray w(env, N - 1, -IloInfinity, IloInfinity);
        for (int i = 0; i < N - 1; i++)
        {
            sprintf(var, "w(%d)", i);
            w[i].setName(var);
            model.add(w[i]);
        }

        IloNumVarArray u(env, N - 1, -IloInfinity, IloInfinity);
        for (int i = 0; i < N - 1; i++)
        {
            sprintf(var, "u(%d)", i);
            u[i].setName(var);
            model.add(u[i]);
        }

        IloExpr obj(env);
        for (int i = 0; i < N - 1; i++)
        {
            obj += f[i];
        }

        model.add(IloMinimize(env, obj));

        for (int j = 0; j < N - 1; j++)
        {
            IloRange r = (-f[j] * f[j] + w[j] * w[j] + u[j] * u[j] <= 0);
            char c[100];
            sprintf(c, "cone%d", j);
            r.setName(c);
            model.add(r);
        }

        for (int i = 0; i < N; i++)
        {
            IloRange r = (x[i] - curTargets[i]->coords.x) * (x[i] - curTargets[i]->coords.x) + (y[i] - curTargets[i]->coords.y) * (y[i] - curTargets[i]->coords.y) <= curTargets[i]->radius * curTargets[i]->radius;
            char c[100];
            sprintf(c, "q%d", i);
            r.setName(c);
            model.add(r);
        }

        for (int j = 1; j < N - 1; j++)
        {
            IloRange r = (w[j] - x[j - 1] + x[j] == 0);
            char c[100];
            sprintf(c, "c%d", j);
            r.setName(c);
            model.add(r);
        }

        for (int j = 1; j < N - 1; j++)
        {
            IloRange r = (u[j] - y[j - 1] + y[j] == 0);
            char c[100];
            sprintf(c, "c%d", j + N);
            r.setName(c);
            model.add(r);
        }

        IloCplex cplex(env);

        cplex.extract(model);

        cplex.setParam(IloCplex::Param::Barrier::QCPConvergeTol, CONVTOL);
        cplex.setParam(IloCplex::Threads, 1);
        cplex.setParam(IloCplex::ParallelMode, 1);

        cplex.setOut(env.getNullStream());

        bool solve = cplex.solve();
        if (solve)
        {
            for (int i = 1; i < N - 1; i++)
            {
                waypoint.x = cplex.getValue(x[i]);
                waypoint.y = cplex.getValue(y[i]);
            }

            double length = cplex.getObjValue();
            return length;
        }
    }
    catch (const IloException &e)
    {
        std::cerr << "CPLEX Error: " << e << std::endl;
    }

    env.end();
    return 0.0;
}