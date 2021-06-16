#ifndef MIF27_BOX_HPP
#define MIF27_BOX_HPP

#include "vec.h"
#include "mat.h"

class Box {
  public :
    Box() ;
    Box(const Point& pmin, const Point& pmax) ;

    bool collides(const Box& rhs) ;

    Point pmin, pmax ;
    Transform T ;
} ;

#endif
