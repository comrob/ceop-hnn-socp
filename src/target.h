/*
 * File name: target.h
 * Date:      2013/10/13 20:08
 * Author:    Jan Faigl
 */

#ifndef __TARGET_H__
#define __TARGET_H__

#include <vector>

#include "coords.h"

namespace hnn {

   struct STarget {
      const int label;
      const Coords coords;
      const int reward;
      const double radius;
      Coords waypoint;

      STarget(const int label, const Coords& pt, const int reward = 0, const double radius = 0.0) : label(label), coords(pt), reward(reward), radius(radius) {
         waypoint = Coords(pt);
      }
      ~STarget() {}

      inline double squared_distance(const Coords & c) const { return waypoint.squared_distance(c); }
   };

   typedef std::vector<STarget*> TargetPtrVector;

} //endLoc namespace nn

#endif

/* end of target.h */