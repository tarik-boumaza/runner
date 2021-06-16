#include "utils.cpp"

class Tube {

  public :
    void init() {
      points.push_back(Point(0*deplace_p, 0*deplace_p, 1*deplace_p));
      points.push_back(Point(0*deplace_p, 0*deplace_p, 2*deplace_p));
      points.push_back(Point(0*deplace_p, 0*deplace_p, 3*deplace_p));
      points.push_back(Point(0*deplace_p, 1*deplace_p, 3*deplace_p));
      points.push_back(Point(0*deplace_p, 2*deplace_p, 3*deplace_p));
      points.push_back(Point(0*deplace_p, 3*deplace_p, 3*deplace_p));
      points.push_back(Point(1*deplace_p, 3*deplace_p, 3*deplace_p));
      points.push_back(Point(1*deplace_p, 4*deplace_p, 3*deplace_p));
      points.push_back(Point(2*deplace_p, 4*deplace_p, 3*deplace_p));
      points.push_back(Point(3*deplace_p, 4*deplace_p, 3*deplace_p));
      points.push_back(Point(4*deplace_p, 4*deplace_p, 3*deplace_p));
      points.push_back(Point(4*deplace_p, 4*deplace_p, 4*deplace_p));
      points.push_back(Point(4*deplace_p, 3*deplace_p, 4*deplace_p));
      points.push_back(Point(4*deplace_p, 2*deplace_p, 4*deplace_p));
      points.push_back(Point(4*deplace_p, 1*deplace_p, 4*deplace_p));
      points.push_back(Point(4*deplace_p, 0*deplace_p, 4*deplace_p));
      points.push_back(Point(4*deplace_p, 0*deplace_p, 3*deplace_p));
      points.push_back(Point(3*deplace_p, 0*deplace_p, 3*deplace_p));
      points.push_back(Point(2*deplace_p, 0*deplace_p, 3*deplace_p));
      points.push_back(Point(2*deplace_p, 0*deplace_p, 2*deplace_p));
      points.push_back(Point(2*deplace_p, 0*deplace_p, 1*deplace_p));
      points.push_back(Point(2*deplace_p, 0*deplace_p, 0*deplace_p));
      points.push_back(Point(1*deplace_p, 0*deplace_p, 0*deplace_p));
      points.push_back(Point(0*deplace_p, 0*deplace_p, 0*deplace_p));
      points.push_back(Point(0*deplace_p, 0*deplace_p, 1*deplace_p));
      points.push_back(Point(0*deplace_p, 0*deplace_p, 2*deplace_p));


      for(unsigned int i=0; i < 8; i++){
        chaikin(points);
      }

      std::vector<std::vector<Point>> cercles;
      std::vector<std::vector<Vector>> norm;

      //je construis les vecteurs orthogonaux à la courbe
      vecteur_orthogonal(points, orthogonaux);
      ///calcul rayon
      r = getNorme(orthogonaux[0]);  //rayon tube
      //génération des cercles
      generation_cercles(points, orthogonaux, cercles, norm);

      m_tube= Mesh(GL_TRIANGLES);
      //génération et dessin des trianges
      float longueur = longueur_tube(points);
      dessine_triangles(m_tube, cercles, norm, longueur);
    }

    Point getPoint(const unsigned int i) {
      if (i >= points.size()) {
        std::cerr << "Indice inconnu!" << std::endl;
        exit(EXIT_FAILURE);
      }
      return points[i];
    }

    Vector getOrthogonal(const unsigned int i) {
      if (i >= orthogonaux.size()) {
        std::cerr << "Indice inconnu!" << std::endl;
        exit(EXIT_FAILURE);
      }
      return orthogonaux[i];
    }

    unsigned int getNbPoints() {
      return points.size();
    }

    double getR() {
      return r;
    }

    Mesh getMesh() {
      return m_tube;
    }

    int nbOrtho() {
      return orthogonaux.size();
    }

  protected :
    Mesh m_tube;
    std::vector<Point> points;
    std::vector<Vector> orthogonaux;
    double r;

  private :
    const float deplace_p = 10;

    double getNorme(const Vector & p1) {
        return sqrt( (p1.x) * (p1.x) + (p1.y) * (p1.y) + (p1.z) * (p1.z) );
      }
};
