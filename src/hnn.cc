/*
 * File name: hnn.cc
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#include <cmath>
#include <limits>
#include <iomanip>

#include <crl/random.h>
#include <crl/timerN.h>
#include <crl/perf_timer.h>
#include <crl/logging.h>
#include <crl/stringconversions.h>
#include <crl/alg/text_result_log.h>
#include <crl/file_utils.h>
#include <crl/assert.h>

#include <crl/gui/shapes.h>
#include <crl/gui/colormap.h>

#include <utility>

#include "canvasview_coords.h"
#include "hnn.h"
#include "hnn_route.h"

#define MAXACT 0.9999

using namespace hnn;
using namespace crl;
using namespace crl::gui;

typedef std::vector<int> IntVector;

/// ----------------------------------------------------------------------------
std::ostream &operator<<(std::ostream &os, const crl::gui::SColor &color)
{
    os << color.red << "," << color.green << "," << color.blue;
    return os;
}

/// ----------------------------------------------------------------------------
void createPermutation(int number, IntVector &permutation)
{
    permutation.clear();
    for (int i = 0; i < number; i++)
    {
        permutation.push_back(i);
    }
}

/// ----------------------------------------------------------------------------
void permute(IntVector &permutation)
{
    int k, tmp;
    crl::CRandom::randomize();
    for (int i = permutation.size(); i > 0; --i)
    {
        k = crl::CRandom::random() % i;
        tmp = permutation[i - 1];
        permutation[i - 1] = permutation[k];
        permutation[k] = tmp;
    }
}

/// ----------------------------------------------------------------------------
std::vector<std::string> split(std::string str, std::string delimiter = " ")
{
    std::vector<std::string> str_split;

    size_t pos = 0;
    std::string token;
    while ((pos = str.find(delimiter)) != std::string::npos)
    {
        token = str.substr(0, pos);
        if (token.length() > 0)
        {
            str_split.push_back(token);
        }

        str.erase(0, pos + delimiter.length());
    }
    str_split.push_back(str);
    return str_split;
}

/// ----------------------------------------------------------------------------
void getProblemName(std::string &problem, std::string &name, const double radius)
{
    auto x = split(problem, "/");
    auto tmp = x[x.size() - 1];
    auto x1 = split(tmp, "_");
    std::stringstream ss1;
    ss1 << x1[0] << "_" << x1[1] << x1[2];
    name = ss1.str(); 
    std::stringstream ss;
    ss << name << "_radius_" << radius << ".txt";
    problem = ss.str();
}

/// ----------------------------------------------------------------------------
/// Class CHNN
/// ----------------------------------------------------------------------------

/// - static method ------------------------------------------------------------
crl::CConfig &CHNN::getConfig(crl::CConfig &config)
{
    // basic config
    config.add<std::string>("output", "output directory to store particular results and outputs", "./");
    config.add<std::string>("results", "result log file, it will be placed in output directory", "/results.log");
    config.add<std::string>("info", "information file, it will be placed in particular experiment directory",
                            "info.txt");
    config.add<std::string>("settings", "store configurations in boost::program_options config file format ",
                            "settings.txt");
    config.add<std::string>("result-path", "file name for the final found path (ring) as sequence of locations",
                            "path.txt");
    config.add<std::string>("result-canvas-output",
                            "file name for final canvas output (eps,png,pdf,svg) are supported");
    config.add<std::string>("result-canvas-suffixes",
                            "comman separated list of exptensis e.g. png,pdf.  if specified  several result images are created, with particular suffix to the resultCanvasOutput");
    config.add<std::string>("name",
                            "name used in result log as user identification if not set a default values (cities) is used");
    config.add<int>("iteration", "set particular interation, otherwise all interations (batch) are performed", -1);
    config.add<int>("batch", "number of iterations from 0 to batch (not included) ", -1);
    config.add<bool>("continue",
                     "in batch mode, partial results are loaded and checked, only missing iterations are computed ",
                     false);
    config.add<bool>("save-results", "disable/enable save results,configs and so on", true);
    config.add<bool>("save-info", "disable/enable save info", true);
    config.add<bool>("save-settings", "disable/enable save settings", true);
    config.add<bool>("save-visual", "disable/enable save visualization results, canvas be", true);
    config.add<bool>("verbose-result-log", "disable/enable printing results log into logger", false);
    // endLoc basic config

    config.add<std::string>("pic-dir", "relative directory in result directory to store pictures from each iteration", "./");
    config.add<std::string>("pic-ext",
                            "extension of pic, eps, png, pdf, svg (supported by particular gui renderer rendered",
                            "png");
    config.add<bool>("save-pic", "enable/disable saving pictures (after each refine)", true);

    // Gui properties
    config.add<bool>("draw-targets", "Enable/Disable drawing targets", true);
    config.add<double>("canvas-border", "Free space around the canvas", 10);
    config.add<std::string>("draw-shape-targets", "Shape of the target", Shape::CITY());
    config.add<std::string>("draw-shape-path", "Shape of the path", Shape::RED_LINE());
    config.add<std::string>("draw-shape-depot", "Shape of the depot node", Shape::DEPOT());

    config.add<bool>("draw-targets-reward", "enable/disable drawing targets in different color using penalty", true);
    config.add<std::string>("draw-targets-reward-palette", "File name with colors for the reward palette", "");

    config.add<bool>("draw-path-iter", "enable/disable drawing path at each iteration", true);

    config.add<std::string>("draw-shape-communication-radius", "Shape of the communication radius for highlight coverage", Shape::POLYGON_FILL());

    config.add<bool>("draw-communication-radius", "enable/disable drawing radius of the selected targets", false);
    config.add<bool>("draw-depot", "enable/disable drawing depot", false);

    // Algorithm parameters
    config.add<int>("iterations", "No. of repreated NN learning from which the best solution is determined", 10);
    config.add<int>("repetitions", "No. of repetations within single NN learning", 2);
    config.add<double>("communication-radius", "Communication radius", 0.0);
    config.add<bool>("compute-matrix", "Compute distance matrix, if false matrix is loaded from file.", true);
    config.add<std::string>("matrix-dir", "Directory with precomputed distance matrices.", "matrices");
    config.add<std::string>("heuristic", "Used heuristic to estimate D function", "socp");

    // Parameters
    config.add<int>("param-a", "Param a", 1);
    config.add<int>("param-b", "Param b", 1);
    config.add<int>("param-c", "Param c", 20);
    config.add<int>("param-d", "Param d", 10);
    config.add<int>("param-e", "Param e", 20);
    config.add<int>("param-f", "Param f", 50);
    config.add<double>("param-t", "Param delta t", 0.0001);
    config.add<double>("param-theta", "Param vartheta", 2);
    return config;
}

/// - static method ------------------------------------------------------------
TargetPtrVector &CHNN::loadOP(const std::string &filename, TargetPtrVector &targets, int &budget, int &numPaths, double radius)
{
    Coords pt;
    int reward;
    std::ifstream in(filename.c_str());
    in >> budget >> numPaths;
    while (in >> pt.x >> pt.y >> reward)
    {
        if (reward == 0)
        {
            // starting and ending location
            targets.push_back(new STarget(targets.size(), pt, reward, 0.0));
        }
        else
        {
            targets.push_back(new STarget(targets.size(), pt, reward, radius));
        }
    }
    in.close();
    return targets;
}

/// - static method ------------------------------------------------------------
void CHNN::releaseTargets(TargetPtrVector &targets)
{
    for (STarget *st : targets)
    {
        delete st;
    }
    targets.clear();
}

/// - constructor --------------------------------------------------------------
CHNN::CHNN(crl::CConfig &config, const std::string &problemFile) : Base(config, "TRIAL"),
                                                                   SAVE_RESULTS(config.get<bool>("save-results")),
                                                                   SAVE_SETTINGS(config.get<bool>("save-settings")),
                                                                   SAVE_INFO(config.get<bool>("save-info")),
                                                                   BORDER(config.get<double>("canvas-border")),
                                                                   ITERATIONS(config.get<int>("iterations")),
                                                                   REPETITIONS(config.get<int>("repetitions")),
                                                                   RADIUS(config.get<double>("communication-radius"))
{
    savePicEnabled = config.get<bool>("save-pic");
    drawPathIter = config.get<bool>("draw-path-iter");
    shapeTargets.setShape(config.get<std::string>("draw-shape-targets"));
    shapePath.setShape(config.get<std::string>("draw-shape-path"));
    shapeCommRadius.setShape(config.get<std::string>("draw-shape-communication-radius"));
    shapeDepot.setShape(config.get<std::string>("draw-shape-depot"));

    int numPaths;
    loadOP(problemFile, targets, budget, numPaths, RADIUS);
    ASSERT_ARGUMENT(targets.size() > 1, "At least 2 targets must be loaded for open routeLocations");
    startTarget = targets[0];
    endTarget = targets[1];
    ASSERT_ARGUMENT(!targets.empty(), "Empty targets loaded");

    problem = problemFile;
    getProblemName(problem, name, RADIUS);
    const std::string matrix_file = config.get<std::string>("matrix-dir") + "/" + problem;

    graph = new CGraph(0, 1, budget, targets, config.get<std::string>("heuristic"));
    graph->init(matrix_file, config.get<bool>("compute-matrix"));
    network = new CNN(config, graph, (int)targets.size());
}

/// - destructor ---------------------------------------------------------------
CHNN::~CHNN()
{
    releaseTargets(targets);
}

/// ----------------------------------------------------------------------------
std::string CHNN::getRevision(void)
{
    return "$Id: hnn.cc 59 2017-12-14 13:08:58Z deckejin $";
}

/// ----------------------------------------------------------------------------
void CHNN::setTimers(const crl::CTimerN &load, const crl::CTimerN &init)
{
    loadTimer = load;
    initTimer = init;
}

/// ----------------------------------------------------------------------------
std::string CHNN::getVersion(void)
{
    return "CHNN 0.9";
}

/// ----------------------------------------------------------------------------
void CHNN::solve(void)
{
    crl::CRandom::randomize();
    Base::solve();
}

/// - protected method ---------------------------------------------------------
void CHNN::load(void)
{
    // nothing to load, structures are loaded in the constructor

    if (canvas)
    {
        CoordsVector points;

        for (STarget *target : targets)
        {
            points.push_back(Coords(target->coords.x + target->radius, target->coords.y + target->radius));
            points.push_back(Coords(target->coords.x - target->radius, target->coords.y - target->radius));
        }
        *canvas << canvas::AREA << points << canvas::END;

        *canvas << "targets" << CShape(config.get<std::string>("draw-shape-targets")) << canvas::POINT;

        for (const STarget *target : targets)
        {
            if (target->radius > 0.0)
            {
                *canvas << target->coords;
            }
        }
        *canvas << shapeDepot << targets[0]->coords;
        *canvas << shapeDepot << targets[1]->coords;

        if (config.get<bool>("draw-targets"))
        {
            std::string pallete = config.get<std::string>("draw-targets-reward-palette");
            if (config.get<bool>("draw-targets-reward") and crl::isFile(pallete))
            {
                crl::gui::CColorMap map;
                map.load(pallete);
                double maxReward = 0;
                double minReward = std::numeric_limits<double>::max();
                for (const STarget *target : targets)
                {
                    if (maxReward < target->reward)
                    {
                        maxReward = target->reward;
                    }
                    if (minReward > target->reward)
                    {
                        minReward = target->reward;
                    }
                }
                if (minReward == maxReward)
                {
                    minReward = 0.99 * maxReward;
                }
                map.setRange(minReward, maxReward);

                *canvas << shapeCommRadius << canvas::ARC;

                for (const STarget *target : targets)
                {
                    if (target->radius > 0.0)
                    {
                        SColor color = map.getColor((double)target->reward);
                        color.alpha = 0.3;
                        *canvas << crl::gui::canvas::FILL_COLOR << color
                                << target->coords.x << target->coords.y << target->radius << 0.0 << 2 * M_PI;
                    }
                }
            }
            else
            {
                if (config.get<bool>("draw-communication-radius"))
                {
                    *canvas << shapeCommRadius << canvas::ARC;
                    for (const STarget *target : targets)
                    {
                        if (target->radius > 0.0)
                        {
                            *canvas << target->coords.x << target->coords.y << target->radius << 0.0 << 2 * M_PI;
                        }
                    }
                } //end draw-communication-radius
            }
        }
        canvas->redraw();
    } //end canvas
}

/// - protected method ---------------------------------------------------------
void CHNN::initialize(void)
{
    createPermutation(targets.size(), permutation); // create permutation
}

/// - protected method ---------------------------------------------------------
void CHNN::iterate(int trial)
{

    double best_reward = 0, reward = 0;
    double best_dist = 0.0, dist = 0.0;
    int best_iter = -1;
    int avg_epoch = 0;

    bool is_feasible = false;
    CRoute *route = new CRoute(0, graph);
    CRoute *best_route = new CRoute(0, graph);

    for (int i = 0; i < ITERATIONS; i++)
    {
        network->init();
        int epoch = 0;
        for (int j = 0; j < REPETITIONS; j++)
        {
            is_feasible = false;
            network->resetLocalMinima();
            while (true)
            {

                network->updateRandomRow();
                epoch++;
                if (network->checkLocalMinima())
                { // a local minimum is reached
                    break;
                }
            }
            network->applyFilter();
            route->constructRoute(network->getStates(), (int)targets.size());
            route->improveRoute();

            double impr_dist = route->getDistance();
            if (impr_dist <= graph->budget)
            {
                is_feasible = true;
            }

            route->examineRoute();
            reward = route->getReward(targets);
            dist = route->getDistance();
            if (dist <= graph->budget && reward > best_reward)
            {
                best_reward = reward;
                best_dist = dist;
                best_iter = i;
                best_route->setSize(route->getSize());
                finalPath.clear();
                for (int k = 0; k < route->getSize(); ++k)
                {
                    finalPath.push_back(route->getRouteLocation(k)->waypoint);
                    best_route->setRouteLocation(k, route->getRouteLocation(k)->label);
                }
                if (canvas && drawPathIter)
                {
                    drawPath(finalPath);
                    if (savePicEnabled)
                    {
                        savePic(i + j, true);
                    }
                }
            }
            avg_epoch += epoch;
            network->adjustNetwork(route->getRoute(), is_feasible);

            INFO("Iterations " << i
                               << " repetetion " << j
                               << " reward: " << reward
                               << " dist " << dist
                               << " isFeasible: " << is_feasible
                               << " best reward: " << best_reward
                               << " best dist: " << best_dist
                               << " budget: " << budget);
        }
    }

    INFO("Best iter: " << best_iter << " best reward: " << best_reward
                       << " best dist: " << best_dist
                       << " budget: " << budget
                       << " epochs: " << (int)(avg_epoch / ITERATIONS));
    fillResultRecord(trial, best_dist, best_reward, ITERATIONS);
    resultLog
        << RADIUS
        << best_iter
        << budget
        << (best_iter != -1)
        << (int)(avg_epoch / ITERATIONS)
        << crl::result::endrec;
}

/// - protected method ---------------------------------------------------------
void CHNN::save(void)
{
    std::string dir;
    updateResultRecordTimes(); //update timers as load and initilization is outside class
    DEBUG("LOAD_TIME_CPU: " << tLoad.cpuTime());
    DEBUG("INIT_TIME_CPU: " << tInit.cpuTime());
    DEBUG("SAVE_TIME_CPU: " << tSave.cpuTime());
    DEBUG("SOLVE_TIME_CPU: " << tSolve.cpuTime());
    if (SAVE_SETTINGS)
    {
        saveSettings(getOutputIterPath(config.get<std::string>("settings"), dir));
    }
    if (SAVE_INFO)
    {
        saveInfo(getOutputIterPath(config.get<std::string>("info"), dir));
    }
    if (SAVE_RESULTS)
    {
        std::string file = getOutputIterPath(config.get<std::string>("result-path"), dir);
        assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
        std::ofstream ofs(file.c_str());
        assert_io(ofs.good(), "Cannot create path '" + file + "'");
        ofs << std::setprecision(14);
        for (const Coords &pt : finalPath)
        {
            ofs << pt.x << " " << pt.y << std::endl;
        }
        assert_io(ofs.good(), "Error occur during path saving");
        ofs.close();
    }
    if (canvas)
    { // map must be set
        drawPath(finalPath);
        saveCanvas();
    }
}

/// - protected method ---------------------------------------------------------
void CHNN::release(void)
{
}

/// - protected method ---------------------------------------------------------
void CHNN::visualize(void)
{
}

/// - protected method ---------------------------------------------------------
void CHNN::defineResultLog(void)
{
    static bool resultLogInitialized = false;
    if (!resultLogInitialized)
    {
        resultLog << result::newcol << "NAME";
        resultLog << result::newcol << "METHOD";
        resultLog << result::newcol << "TRIAL";
        resultLog << result::newcol << "RTIME";
        resultLog << result::newcol << "CTIME";
        resultLog << result::newcol << "UTIME";
        resultLog << result::newcol << "REWARDS";
        resultLog << result::newcol << "LENGTH";
        resultLog << result::newcol << "ITERS";
        resultLog << result::newcol << "RADIUS";
        resultLog << result::newcol << "SOLUTION_ITER";
        resultLog << CResultLog::AddColumn("BUDGET", "x");
        resultLog << CResultLog::AddColumn("ADMISSIBLE", "x");
        resultLog << result::newcol << "EPOCHS";

        resultLogInitialized = true;
    }
}

/// - protected method ---------------------------------------------------------
void CHNN::fillResultRecord(int trial, double length, double rewards, int steps)
{
    resultLog << result::newrec << name << getMethod() << trial;
    long t[3] = {0, 0, 0};
    tLoad.addTime(t);
    tInit.addTime(t);
    tSolve.addTime(t);
    tSave.addTime(t);
    resultLog << t[0] << t[1] << t[2] << rewards << length << steps;
}

/// - protected method ---------------------------------------------------------
void CHNN::after_init(void)
{
    tLoad.append(loadTimer);
    tInit.append(initTimer);
}

/// - protected method ---------------------------------------------------------
double CHNN::refine(int step, double errorMax)
{
    double errorToGoal = errorMax;
    double error = 0.0;
    permute(permutation);
    return error; // return largest error to city
}

/// - private method -----------------------------------------------------------
void CHNN::drawPath(const CoordsVector &pts)
{
    if (canvas)
    {
        *canvas
            << canvas::CLEAR << "path"
            << "path"
            << CShape(config.get<std::string>("draw-shape-path"))
            << canvas::LINESTRING << finalPath << canvas::END
            << canvas::REDRAW;
    } //endLoc if canvas
}

/// - private method -----------------------------------------------------------
void CHNN::savePic(int step, bool detail, const std::string &dir_suffix)
{
    static int lastStep = step;
    static int i = 0;
    if (lastStep != step)
    {
        i = 0;
    }
    if (canvas)
    {
        canvas->redraw();
        std::string dir;
        std::string file = getOutputIterPath(config.get<std::string>("pic-dir") + dir_suffix, dir);
        std::stringstream s1;
        s1 << problem << "_" << budget;
        file = file + "_" + s1.str();
        assert_io(createDirectory(file), "Can not create file in path '" + file + "'");
        std::stringstream ss;
        ss << file << "/"
           << "iter-" << std::setw(3) << std::setfill('0') << step;
        ss << "-" << std::setw(4) << std::setfill('0') << i;

        std::string suffixes(config.get<std::string>("pic-ext"));
        if (!suffixes.empty())
        {
            std::string::size_type cur = 0;
            std::string::size_type next;
            do
            {
                next = suffixes.find(',', cur);
                const std::string &ext = suffixes.substr(cur, next - cur);
                if (!ext.empty())
                {
                    assert_io(canvas->save(ss.str() + "." + ext), "Can create output canvas file '" + file + "'");
                }
                cur = next + 1;
            } while (next != std::string::npos);
        }
        else
        {
            ss << "." << config.get<std::string>("pic-ext");
            assert_io(canvas->save(ss.str()), "Can create output canvas file '" + ss.str() + "'");
        }
    }
    lastStep = step;
    i++;
}

/// - private method -----------------------------------------------------------
double CHNN::get_path_length(const CoordsVector &path, bool closed) const
{
    double len = 0.0;
    for (int i = 1; i < path.size(); ++i)
    {
        len += sqrt(path[i - 1].squared_distance(path[i]));
    }
    if (closed and path.size() > 1)
    {
        len += sqrt(path.back().squared_distance(path.front()));
    }
    return len;
}
