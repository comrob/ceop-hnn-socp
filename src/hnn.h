/*
 * File name: hnn.h
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#ifndef __HNN_OP_H__
#define __HNN_OP_H__

#include <vector>

#include <crl/config.h>
#include <crl/alg/algorithm.h>
#include <crl/gui/shape.h>

#include "coords.h"
#include "target.h"
#include "hnn_graph.h"
#include "hnn_nn.h"

namespace hnn
{

   class CHNN : public crl::CAlgorithm
   {
      typedef crl::CAlgorithm Base;
      typedef std::vector<int> IntVector;

   public:
      static crl::CConfig &getConfig(crl::CConfig &config);
      static std::string getName(void) { return "hnn"; }
      static TargetPtrVector &loadOP(const std::string &filename, TargetPtrVector &targets, int &budget, int &numPaths, double radius);
      static void releaseTargets(TargetPtrVector &targets);

      CHNN(crl::CConfig &config, const std::string &problemFile);
      ~CHNN();

      std::string getRevision(void);
      std::string getMethod(void) { return getName(); }
      void setTimers(const crl::CTimerN &load, const crl::CTimerN &init);
      std::string getVersion(void);

      void solve(void);

   protected:
      // methods from CAlgorithm
      void load(void);
      void initialize(void);
      void after_init(void);
      void iterate(int trial);
      void save(void);
      void release(void);
      void visualize(void);
      void defineResultLog(void);
      void fillResultRecord(int iter, double length, double rewards, int steps);

      double refine(int step, double errorMax);

   private:
      void drawPath(const CoordsVector &pts);
      void savePic(int step, bool detail = false, const std::string &dir_suffix = "");
      void releaseTargets(void);

      double get_path_length(const CoordsVector &path, bool closed = true) const;

      /// ----------------------------------------------------------------------------
      /// Variables
      /// ----------------------------------------------------------------------------
   protected:
      IntVector permutation;

      crl::CTimerN loadTimer;
      crl::CTimerN initTimer;
      bool savePicEnabled;

      const bool SAVE_RESULTS;
      const bool SAVE_INFO;
      const bool SAVE_SETTINGS;

      const double BORDER;

      const int ITERATIONS;
      const int REPETITIONS;
      const double RADIUS;

      TargetPtrVector targets;
      int budget;
      std::string problem;
      STarget *startTarget;
      STarget *endTarget;
      CGraph *graph;
      CNN *network;

      bool drawPathIter;
      crl::gui::CShape shapeTargets;
      crl::gui::CShape shapePath;
      crl::gui::CShape shapeCommRadius;
      crl::gui::CShape shapePathNodes;
      crl::gui::CShape shapeDepot;

      CoordsVector finalPath;
   };

} // namespace hnn

#endif

/* end of hnn.h */