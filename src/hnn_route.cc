/*
 * File name: hnn_route.cc
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */
#include <cstdlib>
#include <limits>

#include <crl/logging.h>
#include <crl/stringconversions.h>

#include <ilcplex/ilocplex.h>

#define CONVTOL 1e-12
#define dist(i, j) sqrt(path[i]->coords.squared_distance(path[j]->coords))

ILOSTLBEGIN

#include "hnn_route.h"

using namespace hnn;
using namespace crl;

/// ----------------------------------------------------------------------------
double get_length(const Coords &a, const Coords &b)
{
	return sqrt(a.squared_distance(b));
}

/// ----------------------------------------------------------------------------
double get_path_length_centers(const TargetPtrVector &path)
{
	double len = 0.0;
	for (int i = 1; i < (int)path.size(); i++)
		len += sqrt(path[i - 1]->coords.squared_distance(path[i]->coords));
	return len;
}

/// - constructor --------------------------------------------------------------
CRoute::CRoute(int size, const CGraph *graph) : routeSize(size), graph(graph)
{
	routeLocations.resize(size);
}

/// - destructor ---------------------------------------------------------------
CRoute::~CRoute()
{
	routeLocations.clear();
}

/// - public method ------------------------------------------------------------
void CRoute::setRoute(const TargetPtrVector &newRoute)
{
	routeLocations.clear();
	for (int i = 0; i < newRoute.size(); i++)
	{
		routeLocations.push_back(newRoute[i]);
	}
	routeSize = routeLocations.size();
}

/// - public method ------------------------------------------------------------
void CRoute::setSize(int size)
{
	routeLocations.resize(size);
	routeSize = size;
}

/// - public method ------------------------------------------------------------
void CRoute::constructRoute(const Double2Vector nodes, const int size)
{
	int route_size = 0;

	setSize(size);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			if (nodes[i][j] == 1)
			{
				setRouteLocation(j, i);
				route_size++;
			}
		}
	}
	routeSize = route_size;

	deleteDuplicates();
}

/// - public method ------------------------------------------------------------
void CRoute::improveRoute()
{
	double dist = getDistanceOnRoute(routeLocations);
	twoOpt(routeLocations);

	double best_dist = getDistanceOnRoute(routeLocations);

	TargetPtrVector route;
	for (auto pt : routeLocations)
		route.push_back(pt);
	double new_len = optimizeWaypoints(route);
	if (new_len < best_dist)
	{
		setRoute(route);
	}
}

/// - public method ------------------------------------------------------------
void CRoute::twoOpt(TargetPtrVector &path)
{
	const int N = path.size();
	int counter = 0;
	double mchange;
	do
	{
		mchange = 0.0;
		int mi = -1;
		int mj = -1;
		for (int i = 1; i < N - 1; i++)
		{
			for (int j = i + 1; j < N - 1; j++)
			{
				double change = dist(i - 1, j) + dist(i, j + 1) - dist(i - 1, i) - dist(j, j + 1);
				if (mchange > change)
				{
					mchange = change;
					mi = i;
					mj = j;
				}
				counter += 1;
			}
		}
		if (mi > 0 and mj > 0)
		{
			TargetPtrVector newPath;
			for (int i = 0; i < mi; i++)
			{
				newPath.push_back(path[i]);
			}
			for (int i = mj; i >= mi; i--)
			{
				newPath.push_back(path[i]);
			}
			for (int i = mj + 1; i < N; i++)
			{
				newPath.push_back(path[i]);
			}
			path = newPath;
		}
	} while (mchange < -1e-5);
}

/// - public method ------------------------------------------------------------
void CRoute::examineRoute()
{

	double dist = getDistance();
	const double limit = graph->budget;
	int attempts = 0, max_attempts = 100, last_inserted_loc = 0;
	bool is_after_add = false;

	int fromIdx = 0, toLoc = 0;

	dist = optimizeWaypoints(routeLocations);

	int remove_cnt = 0, add_cnt = 0;

	for (attempts = 0; attempts < max_attempts; attempts++)
	{
		dist = getDistance();
		if (dist > limit)
		{
			int remove_to = -1;
			if (last_inserted_loc != 0)
			{
				remove_to = last_inserted_loc;
			}
			else
			{
				remove_to = getIdxToRemoveByScoreRatio(routeLocations);
			}
			if (remove_to > 0)
			{
				deleteLocation(routeLocations, remove_to);
				setSize(routeLocations.size());
				if (is_after_add)
				{
					break;
				}
			}
		}
		else
		{
			getCheapestDistanceIndex(routeLocations, fromIdx, toLoc);
			if (fromIdx > 0 && toLoc > 0)
			{
				last_inserted_loc = fromIdx + 1;
				is_after_add = true;
				insertLocation(routeLocations, fromIdx, toLoc);
				setSize(routeLocations.size());
				improveRoute();
			}
		}
	}

	deleteDuplicates();
	INFO("Route examined: " << attempts << " " << remove_cnt << " removed, " << add_cnt << " added.");
}

/// - public method ------------------------------------------------------------
void CRoute::setRouteLocation(int index, int value)
{
	routeLocations[index] = graph->getLocation(value);
}

/// - public method ------------------------------------------------------------
double CRoute::getDistance() const
{
	double dist = 0.0;
	for (int i = 0; i < routeLocations.size() - 1; i++)
	{
		dist += get_length(routeLocations[i]->waypoint, routeLocations[i + 1]->waypoint);
	}
	return dist;
}

/// - public method ------------------------------------------------------------
double CRoute::getReward() const
{

	int score = 0;
	for (int i = 0; i < routeSize; ++i)
	{
		score += routeLocations[i]->reward;
	}
	return score;
}

/// - public method ------------------------------------------------------------
double CRoute::getReward(const TargetPtrVector &targets) const
{
	int score = 0;
	int cnt = 0;
	for (const auto target : targets)
	{
		for (int i = 0; i < (int)routeLocations.size() - 1; i++)
		{
			Coords a = routeLocations[i]->waypoint;
			Coords b = routeLocations[i + 1]->waypoint;
			Coords p;
			char s;

			double dist2 = distPointToSegment(target->coords, a, b, p, s);
			if (sqrt(dist2) <= target->radius + 1e-5)
			{
				score += target->reward;
				cnt++;
				break;
			}
		}
	}
	// std::cout << "Reward " << score << " no. covered: " << cnt << " " << targets.size() << std::endl;
	return score;
}

/// - private method ------------------------------------------------------------
void CRoute::deleteDuplicates()
{
	int route_size = routeSize;
	TargetPtrVector new_route;
	new_route.resize(route_size);
	int size = 0;
	STarget *point = NULL;
	bool in_path = false;
	BoolVector points(graph->size, false);
	point = getRouteLocation(0);
	new_route[size++] = point;
	points[point->label] = true;
	for (int i = 1; i < route_size; i++)
	{
		point = getRouteLocation(i);
		if (point->label == graph->SDEPOT || point->label == graph->EDEPOT)
		{
			continue;
		}
		if (points[point->label])
		{
			continue;
		}
		points[point->label] = true;
		new_route[size++] = point;
	}
	new_route[size++] = graph->getLocation(graph->EDEPOT);
	if (size < route_size)
	{
		new_route.resize(size);
	}
	setRoute(new_route);
	new_route.clear();
}

/// - private method ------------------------------------------------------------
double CRoute::getDistanceOnRoute(TargetPtrVector routeTargets)
{
	double dist = 0.0;
	for (int i = 0; i < routeTargets.size() - 1; i++)
	{
		dist += get_length(routeTargets[i]->waypoint, routeTargets[i + 1]->waypoint);
	}
	return dist;
}

/// - private method ------------------------------------------------------------
void CRoute::twoOpt(TargetPtrVector &route, int i, int j)
{
	int route_size = route.size(), index = j;
	TargetPtrVector new_route;
	new_route.resize(route_size);
	std::copy(route.begin(), route.end(), new_route.begin());

	new_route[0] = route[0];
	for (int h = 1; h < i; h++)
	{
		new_route[h] = route[h];
	}
	for (int h = i; h < j + 1; h++)
	{
		new_route[h] = route[index];
		index--;
	}
	for (int h = j + 1; h < route_size; h++)
	{
		new_route[h] = route[h];
	}
	route.clear();
	route.resize(new_route.size());
	std::copy(new_route.begin(), new_route.end(), route.begin());
	new_route.clear();
}

/// - private method ------------------------------------------------------------
int CRoute::getIdxToRemoveByScoreRatio(TargetPtrVector route)
{
	double max_ratio = 0.0, ratio = 0.0;
	int to = -1;
	for (int i = 1; i < route.size() - 2; i++)
	{
		STarget *la = route[i];
		STarget *lb = route[i + 1];
		if (lb->reward == 0)
			continue;

		ratio = get_length(la->waypoint, lb->waypoint) / lb->reward;
		if (ratio > max_ratio)
		{
			if (lb->label != graph->SDEPOT && lb->label != graph->EDEPOT)
			{
				max_ratio = ratio;
				to = i + 1;
			}
		}
	}
	return to;
}

/// - private method ------------------------------------------------------------
void CRoute::deleteLocation(TargetPtrVector &route, int remIdx)
{
	route.erase(route.begin() + remIdx);
}

/// - private method ------------------------------------------------------------
void CRoute::insertLocation(TargetPtrVector &route, int fromIdx, int toLoc)
{
	route.insert(route.begin() + fromIdx + 1, graph->getLocation(toLoc));
}

/// - private method ------------------------------------------------------------
void CRoute::getCheapestDistanceIndex(TargetPtrVector route, int &fromIdx, int &toLoc)
{
	fromIdx = 0, toLoc = 0;
	bool is_break = false;
	double max_ratio = 10000000000.0, dist_aj, dist_jb, dist_ratio, ratio;
	for (int i = 0; i < route.size() - 1; i++)
	{
		int a = route[i]->label;
		int b = route[i + 1]->label;
		for (int j = 1; j < graph->size - 1; j++)
		{
			if (a == j || b == j)
			{
				continue;
			}
			for (int k = 0; k < route.size(); k++)
			{ // checks if is in route already
				if (j == route[k]->label)
				{
					is_break = true;
					break;
				}
			}
			if (is_break)
			{
				is_break = false;
				continue;
			}

			dist_aj = get_length(route[i]->waypoint, graph->getLocation(j)->waypoint);
			dist_jb = get_length(route[i + 1]->waypoint, graph->getLocation(j)->waypoint);

			dist_ratio = dist_aj + dist_jb - get_length(route[i]->waypoint, route[i + 1]->waypoint);
			ratio = dist_ratio / graph->getLocation(j)->reward;
			if (ratio < max_ratio)
			{
				max_ratio = ratio;
				fromIdx = i;
				toLoc = j;
			}
		}
	}
}

/// - private method ------------------------------------------------------------
double CRoute::optimizeWaypoints(TargetPtrVector &route)
{
	TargetPtrVector new_route;
	int N = (int)route.size();
	TargetPtrVector curTargets;
	for (auto pt : route)
	{
		curTargets.push_back(pt);
	}

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
		IloRange cx0 = x[0] == curTargets[0]->waypoint.x;
		model.add(cx0);
		IloRange cy0 = y[0] == curTargets[0]->waypoint.y;
		model.add(cy0);
		IloRange cxn = x[N - 1] == curTargets[N - 1]->waypoint.x;
		model.add(cxn);
		IloRange cyn = y[N - 1] == curTargets[N - 1]->waypoint.y;
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
			for (int i = 0; i < N; i++)
			{

				route[i]->waypoint.x = cplex.getValue(x[i]);
				route[i]->waypoint.y = cplex.getValue(y[i]);
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

/// - private method ------------------------------------------------------------
double CRoute::distPointToSegment(const Coords &center, const Coords &a, const Coords &b, Coords &p, char &s) const
{
	const double r_num = (center.x - a.x) * (b.x - a.x) + (center.y - a.y) * (b.y - a.y);
	const double r_den = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
	const double r = r_num / r_den;
	double ret;

	if (r >= 0 && r <= 1)
	{
		const double r_s = (a.y - center.y) * (b.x - a.x) - (a.x - center.x) * (b.y - a.y);
		p = Coords(a.x + r * (b.x - a.x), a.y + r * (b.y - a.y));
		s = 's';
		return r_s * r_s / r_den;
	}
	const double dist1 = center.squared_distance(a);
	const double dist2 = center.squared_distance(b);
	p = dist1 < dist2 ? a : b;
	s = dist1 < dist2 ? 'a' : 'b';
	return dist1 < dist2 ? dist1 : dist2;
}
