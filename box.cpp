#include "box.hpp"

Box::Box() {
  pmin = Origin() ;
  pmax = Origin() ;
  T = Identity() ;
}

Box::Box(const Point& i_pmin, const Point& i_pmax) {
  pmin = i_pmin ;
  pmax = i_pmax ;
  T = Identity() ;
}

static bool collides_boxes(const Box& b1, const Box& b2, int i, int direction){
  Point r;
  Vector v;

  v(i) = direction;

  // Transformer la direction pour l'amener dans le repère de b2
  Vector v_b2 = b2.T.inverse()(b1.T(v));

  // Déterminer le point à tester
  for(int j=0; j<3; j++){
    if(v_b2(j) > 0) r(j) = b2.pmin(j);
      else r(j) = b2.pmax(j);
  }

  // Ramener ce point dans le repère de b1
  Point p_b1 = b1.T.inverse()(b2.T(r));

  // Tester ce point avec pmin/pmax
  if(direction < 0){
    if((p_b1(i) < b1.pmin(i))) {
       return false;
     }
  } else {
    if((p_b1(i) > b1.pmax(i))) {
       return false;
     }
  }
  return true;
}


bool Box::collides(const Box& rhs) {
  // partie 1

  // Point pmin1 = T(pmin)
  // Point pmax1 = T(pmax)
  // Point pmin2 = rhs.T(rhs.pmin)
  // Point pmax2 = rhs.T(rhs.pmax)
  //
  // if((pmax1.x >= pmin2.x) && (pmin1.x <= pmax2.x)){
  //   if ((pmax1.y >= pmin2.y) && (pmin1.y <= pmax2.y)){
  //     if ((pmax1.z >= pmin2.z) && (pmin1.z <= pmax2.z)){
  //       return true;
  //     }
  //   }
  // }
  //
  // return false ;

  // partie 2

  const Box & b1= *(this);


  for(int i = 0; i<3; i++){

    if(!collides_boxes(b1, rhs, i, 1) || !collides_boxes(b1, rhs, i, -1)
        || !collides_boxes(rhs, b1, i, 1) || !collides_boxes(rhs, b1, i, -1)){
      return false;
    }
  }

  return true;
}
