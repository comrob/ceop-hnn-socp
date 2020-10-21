/*
 * File name: coords.cc
 * Date:      2014/01/16 16:54
 * Author:    Jan Faigl
 */

#include "coords.h"

/// ----------------------------------------------------------------------------
std::istream &operator>>(std::istream &is, Coords &pt)
{
   return is >> pt.x >> pt.y;
}

/// ----------------------------------------------------------------------------
std::ostream &operator<<(std::ostream &os, const Coords &pt)
{
   return os << "[" << pt.x << "," << pt.y << "]";
}

/* end of coords.cc */