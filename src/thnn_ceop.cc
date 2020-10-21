/*
 * File name: thnn_ceop.cc
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <crl/config.h>
#include <crl/logging.h>
#include <crl/perf_timer.h>
#include <crl/boost_args_config.h>

#include <crl/gui/guifactory.h>
#include <crl/gui/win_adjust_size.h>

#include "hnn.h"

using crl::logger;
using namespace hnn;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

const std::string HNN_VERSION = "2.0";

typedef crl::gui::CCanvasBase Canvas;

/// ----------------------------------------------------------------------------
/// Program options variables
/// ----------------------------------------------------------------------------
std::string guiType = "none";
std::string problemFile = "";

crl::CConfig guiConfig;
crl::CConfig hnnConfig;
std::string canvasOutput = "";
std::string solutionFile = "";

/// ----------------------------------------------------------------------------
/// Global variable
/// ----------------------------------------------------------------------------
crl::gui::CGui *g = 0;
#define GUI(x) \
    if (gui)   \
    {          \
        x;     \
    }

/// ----------------------------------------------------------------------------
bool parseArgs(int argc, char *argv[])
{
    bool ret = true;
    std::string configFile;
    std::string guiConfigFile;
    std::string loggerCfg = "";

    po::options_description desc("General options");
    desc.add_options()("help,h", "produce help message")
        ("config,c", po::value<std::string>(&configFile)->default_value(std::string(argv[0]) + ".cfg"), "configuration file")
          ("logger-config,l", po::value<std::string>(&loggerCfg)->default_value(loggerCfg), "logger configuration file")
          ("config-gui", po::value<std::string>(&guiConfigFile)->default_value(std::string(argv[0]) + "-gui.cfg"), "dedicated gui configuration file")
          ("problem", po::value<std::string>(&problemFile), "problem file");
    try
    {
        po::options_description guiOptions("Gui options");
        crl::gui::CGuiFactory::getConfig(guiConfig);
        crl::gui::CWinAdjustSize::getConfig(guiConfig);
        guiConfig.add<double>("gui-add-x",
                              "add the given value to the loaded goals x coord to determine the canvas netSize and transformation",
                              0);
        guiConfig.add<double>("gui-add-y",
                              "add the given value to the loaded goals y coord to determine the canvas netSize and transformation",
                              0);
        boost_args_add_options(guiConfig, "", guiOptions);
        guiOptions.add_options()("canvas-output", po::value<std::string>(&canvasOutput), "result canvas outputfile");

        po::options_description tspOptions("PC-TSP solver options");
        boost_args_add_options(CHNN::getConfig(hnnConfig), "", tspOptions);

        po::options_description cmdline_options;
        cmdline_options.add(desc).add(guiOptions).add(tspOptions);

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
        po::notify(vm);

        std::ifstream ifs(configFile.c_str());
        store(parse_config_file(ifs, cmdline_options), vm);
        po::notify(vm);
        ifs.close();
        ifs.open(guiConfigFile.c_str());
        store(parse_config_file(ifs, cmdline_options), vm);
        po::notify(vm);
        ifs.close();

        if (vm.count("help"))
        {
            std::cerr << std::endl;
            std::cerr << "HNN-based Solver for the CEOP ver. " << HNN_VERSION << std::endl;
            std::cerr << cmdline_options << std::endl;
            ret = false;
        }
        if (
            ret &&
            loggerCfg != "" &&
            fs::exists(fs::path(loggerCfg)))
        {
            crl::initLogger("hnn", loggerCfg.c_str());
        }
        else
        {
            crl::initLogger("hnn");
        }
        if (!fs::exists(fs::path(problemFile)))
        {
            ERROR("Problem file '" + problemFile + "' does not exists");
            ret = false;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << std::endl;
        std::cerr << "Error in parsing arguments: " << e.what() << std::endl;
        ret = false;
    }
    return ret;
}

/// ---------------------------------------------------------------------------
CoordsVector &load_goals_coords(const std::string &filename, CoordsVector &pts)
{
    double x, y, r, re;
    int budget, t;
    r = hnnConfig.get<double>("communication-radius");
    pts.clear();
    std::ifstream in(filename);
    in >> budget >> t;
    while (in >> x >> y >> re)
    {
        pts.push_back(Coords(x - r, y - r));
        pts.push_back(Coords(x + r, y + r));
    }
    return pts;
}

/// - main program -------------------------------------------------------------
int main(int argc, char *argv[])
{
    Canvas *canvas = 0;
    int ret = -1;
    if (parseArgs(argc, argv))
    {
        INFO("Start Logging");
        try
        {
            CoordsVector pts;
            {
                crl::CPerfTimer t("Load problem real time:");
                pts = load_goals_coords(problemFile, pts);
            }
            crl::gui::CWinAdjustSize::adjust(pts, guiConfig);
            if ((g = crl::gui::CGuiFactory::createGui(guiConfig)) != 0)
            {
                INFO("Start gui " + guiConfig.get<std::string>("gui"));
                canvas = new Canvas(*g);
            }
            CHNN hnn(hnnConfig, problemFile);
            hnn.setCanvas(canvas);
            {
                crl::CPerfTimer t("Total solve time: ");
                hnn.solve();
            }
            INFO("End logging.");
            if (canvas)
            {
                if (canvasOutput.size())
                {
                    canvas->save(canvasOutput);
                }
                if (!guiConfig.get<bool>("nowait"))
                {
                    INFO("CLICK TO EXIT");
                    canvas->click();
                }
                delete canvas;
                delete g;
            }
        }
        catch (crl::exception &e)
        {
            ERROR("Exception " << e.what() << "!");
        }
        catch (std::exception &e)
        {
            ERROR("Runtime exception " << e.what() << "!");
        }
        ret = EXIT_SUCCESS;
    }
    crl::shutdownLogger();
    return ret;
}