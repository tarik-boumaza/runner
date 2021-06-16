
//! \file tuto9.cpp utilisation d'un shader 'utilisateur' pour afficher un tube Mesh

#include "mat.h"
#include "mesh.h"
#include "wavefront.h"

#include "orbiter.h"
#include "program.h"
#include "uniforms.h"
#include "draw.h"
#include "text.h"
#include "box.hpp"
#include "app_camera.h"      // classe Application a deriver

#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>      // std::stringstream
#include <cmath>        // std::abs


static const float deplace_p = 10;

void  chaikin(std::vector<Point> & points){
  std::vector<Point> res;
  res.push_back(points[0]);
  Point m1, m2;
  for(unsigned int i=0; i < points.size() - 1; i++){
    m1 = points[i]+ 0.25*Vector(points[i],points[i+1]);
    m2 = points[i]+ 0.75*Vector(points[i],points[i+1]);
    res.push_back(m1);
    res.push_back(m2);
  }
  points.clear();
  for(unsigned int i=0; i < res.size(); i++){
    points.push_back(res[i]);
  }
}

Vector rotation (Vector & v1, Vector & v2, Vector & d){
  Transform t = Rotation(v1,v2);
  return t(d);
}

void vecteur_orthogonal (std::vector<Point> & points, std::vector<Vector> & orthogonaux){
  Vector a0(points[0],points[1]), a1;
  Vector v(1,0,0);
  Vector d = normalize(cross(a0,v));
  if (abs(d.x - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1) {
    v = Vector(0,1,0);
    d = normalize(cross(a0,v));
    if (abs(d.x - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1) {
      v = Vector(0,0,1);
      d = normalize(cross(a0,v));
    }
  }
  orthogonaux.push_back(d);
  for(unsigned int i = 0; i < points.size() - 2; i++){
    a0 = Vector (points[i],points[i+1]);
    a1 = Vector (points[i+1], points[i+2]);
    d = rotation(a0, a1,d);
    orthogonaux.push_back(d);
  }
}


void points_cercle(Point & p1, Point & p2, Vector & v, std::vector<Point> & pc, std::vector<Vector> & nm){
  Vector axe(p1,p2);
  float angle = 30.0;
  Transform tr = Rotation(axe, angle);
  Transform tt;
  Point p, pt;
  for(int i = 0; i < 12; i++){
    v = tr(v);
    tt = Translation(v);
    p = Point (p1.x-v.x, p1.y-v.y, p1.z-v.z);
    pt = Point (tt(p));
    pc.push_back(p);
    nm.push_back(Vector(pt,p));
  }
}

void generation_cercles(std::vector<Point> & points, std::vector<Vector> & orthogonaux, std::vector<std::vector<Point>> & cercles,std::vector<std::vector<Vector>> & normales){
  std::vector<Point> pc;
  std::vector<Vector> nm;

  for(unsigned int i = 0; i < orthogonaux.size() - 1; i++){
    pc.clear();
    nm.clear();
    points_cercle(points[i], points[i+1], orthogonaux[i], pc, nm);
    cercles.push_back(pc);
    normales.push_back(nm);
  }
}


void dessine_triangles(Mesh& m, const std::vector<std::vector<Point>> & cercles, const std::vector<std::vector<Vector>> & norm){
  unsigned int i, j,a, b, c, d, la, lb, lc, ld;
  for(i = 0; i < cercles.size() - 1; i++){
    for(j = 0; j < cercles[i].size() - 1; j++){
      a = m.normal(norm[i][j]).vertex(cercles[i][j]);
      b = m.normal(norm[i+1][j]).vertex(cercles[i+1][j]);
      c = m.normal(norm[i+1][j+1]).vertex(cercles[i+1][j+1]);
      d = m.normal(norm[i][j+1]).vertex(cercles[i][j+1]);
      m.triangle(a,c,b);
      m.triangle(a,d,c);
      if(j == cercles[i].size() - 2){
        la = m.normal(norm[i][j+1]).vertex(cercles[i][j+1]);
        lb = m.normal(norm[i+1][j+1]).vertex(cercles[i+1][j+1]);
        lc = m.normal(norm[i+1][0]).vertex(cercles[i+1][0]);
        ld = m.normal(norm[i][0]).vertex(cercles[i][0]);
        m.triangle(la,lc,lb);
        m.triangle(la,ld,lc);
      }
    }
  }
}


class Tube : public AppCamera
{
public:
    // constructeur : donner les dimensions de l'image, et eventuellement la version d'openGL.
    Tube( ) : AppCamera(1024, 640) {}

    int init( )
    {

        // // etape 1 : charger un tube

        points.push_back(Point(0., 0., 1*deplace_p));
        points.push_back(Point(0., 0., 2*deplace_p));
        points.push_back(Point(0., 0., 3*deplace_p));
        points.push_back(Point(0., 1*deplace_p, 3*deplace_p));
        points.push_back(Point(0., 2*deplace_p, 3*deplace_p));
        points.push_back(Point(0., 3*deplace_p, 3*deplace_p));
        points.push_back(Point(1*deplace_p, 3*deplace_p, 3*deplace_p));

        points.push_back(Point(2*deplace_p, 4*deplace_p, 3*deplace_p));
        points.push_back(Point(2*deplace_p, 3*deplace_p, 3*deplace_p));
        points.push_back(Point(1*deplace_p, 3*deplace_p, 3*deplace_p));
        points.push_back(Point(1*deplace_p, 3*deplace_p, 2*deplace_p));
        points.push_back(Point(1*deplace_p, 3*deplace_p, 1*deplace_p));
        points.push_back(Point(1*deplace_p, 3*deplace_p, 0.));
        points.push_back(Point(0, 2*deplace_p, 0.));
        points.push_back(Point(0, 1*deplace_p, 0.));
        points.push_back(Point(0., 0., 0.));
        points.push_back(Point(0., 0., 1*deplace_p));
        points.push_back(Point(0., 0., 2*deplace_p));


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

        alpha = 0;
        niveau = 1;

        tube= Mesh(GL_TRIANGLES);
        //génération et dessin des trianges
        dessine_triangles(tube, cercles, norm);

        int random;
        // Initialisation du tableau d'obstacles
        for(int i = 0; i < 20; i++) {
          random = rand()%(points.size()-1500) + 600;
          obstacles.push_back(random);
          random = rand()% 360;
          angles.push_back(random);
        }

        //charge l'objet
        objet= read_mesh("projet/data/voiture.obj");
        //objet.default_color(Green()) ;

        Point pmin_box, pmax_box ;
        objet.bounds(pmin_box, pmax_box) ;

        b1 = Box(pmin_box, pmax_box) ;

        //charger les obstacles
        obstacle = read_mesh("projet/data/obstacle.obj");
        obstacle.bounds(pmin_box, pmax_box) ;

        for(int i = 0; i < 20; i++) {
          boxes.push_back(Box(pmin_box, pmax_box));
        }

        finJeu = false;

        // etape 1 : creer le shader program
        program= read_program("projet/shaders/tube_color.glsl");
        program_print_errors(program);

        // etape 2 : creer une camera pour observer l'tube
        // construit l'englobant de l'tube, les extremites de sa boite englobante
        Point pmin, pmax;
        tube.bounds(pmin, pmax);
        indice = 100;

        console = create_text();


        // regle le point de vue de la camera pour observer l'tube
        //camera().lookat(pmin, pmax);

        // etat openGL par defaut
        glClearColor(0.2f, 0.2f, 0.2f, 1.f);        // couleur par defaut de la fenetre

        glClearDepth(1.f);                          // profondeur par defaut
        glDepthFunc(GL_LESS);                       // ztest, conserver l'intersection la plus proche de la camera
        glEnable(GL_DEPTH_TEST);                    // activer le ztest

        return 0;   // ras, pas d'erreur
    }

    // destruction des tubes de l'application
    int quit( )
    {
        // etape 3 : detruire le shader program
        release_program(program);
        tube.release();
        objet.release();
        release_text(console);
        return 0;
    }

    // dessiner une nouvelle image
    int render( )
    {

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if(!finJeu){
        indice += 2*niveau + 2;
        std::string str = "Niveau " + std::to_string(niveau);
        char char_array[str.size() + 5];
        strcpy(char_array, str.c_str());
        printf(console, 0, 0, char_array);
        draw(console, window_width(), window_height());
        if(key_state(SDLK_LEFT))
          alpha = (alpha + 2) % 360;     // tourne vers la gauche
        if(key_state(SDLK_RIGHT))
          alpha = (alpha - 2) % 360;      // tourne vers la droite
      }

      if (indice > points.size() - 1) {
        niveau += 1;
        indice = 10;
      }

      Point p = points[indice];
      Vector d(points[(indice+1)], points[indice]);

      Transform R = Rotation(d,alpha);
      Vector n(orthogonaux[indice]);
      Vector na(R(n));
      Point pos_objet = p + r * na;

      Transform m_transform_objet = atlook(pos_objet, pos_objet + d, na)*Translation(0,r/6,0)*Scale(2,2,2);

      Transform m_transform_camera = Translation(2*na)*Translation(50*d);

      // etape 2 : dessiner tube avec le shader program
      // configurer le pipeline
      glUseProgram(program);

      // configurer le shader program
      // . recuperer les transformations
      /*Transform model= Identity();
      Transform view= camera().view();
      Transform projection= camera().projection(window_width(), window_height(), 45);*/

      Transform model= Identity();
      Transform view= Lookat(m_transform_camera(pos_objet),pos_objet, na);
      Transform projection= Perspective(90, (float) window_width() / (float) window_height(), .1f, 1000.f);

      // . composer les transformations : model, view et projection
      Transform mvp= projection * view * model;

      // . parametrer le shader program :
      //   . transformation : la matrice declaree dans le vertex shader s'appelle mvpMatrix
      program_uniform(program, "mvpMatrix", mvp);
      program_uniform(program, "modelMatrix", model);
      program_uniform(program, "viewInvMatrix", Inverse(view));
      //   . ou, directement en utilisant openGL :
      //   int location= glGetUniformLocation(program, "mvpMatrix");
      //   glUniformMatrix4fv(location, 1, GL_TRUE, mvp.buffer());
      // . parametres "supplementaires" :
      //   . couleur des pixels, cf la declaration 'uniform vec4 color;' dans le fragment shader
      //program_uniform(program, "color", vec4(1, 1, 0, 1));
      //   . ou, directement en utilisant openGL :
      //   int location= glGetUniformLocation(program, "color");
      //   glUniform4f(location, 1, 1, 0, 1);

      // go !
      // indiquer quels attributs de sommets du mesh sont necessaires a l'execution du shader.
      // tuto9_color.glsl n'utilise que position. les autres de servent a rien.
      tube.draw(program, /* use position */ true, /* use texcoord */ false, /* use normal */ true, /* use color */ false, /* use material index*/ false);
      draw(objet, m_transform_objet,/*camera()*/ view, projection);


      /////////////////////////////////////  Ajout d'obstacle /////////////////////////////////
      std::vector<Transform> m_transform_obstacles;
      for(int i = 0; i < obstacles.size(); i++) {
        Point p_ob = points[obstacles[i]];
        Vector d_ob(points[obstacles[i]], points[obstacles[i]+1]);
        Transform R_ob = Rotation(d_ob, angles[i]);
        Vector n_ob(orthogonaux[obstacles[i]]);
        Vector na_ob(R_ob(n_ob));
        Point pos_ob = p_ob + r * na_ob;

        boxes[i].T = atlook(pos_ob, pos_ob + d_ob, na_ob)*Translation(0,0.05,0)*Scale(0.1,0.1,0.1);
        draw(obstacle, boxes[i].T,/*camera()*/ view, projection);
      }

      ////////////////////////////////// Collision ///////////////////////////
      if (!finJeu) {
        b1.T = m_transform_objet ;
        for(int i = 0; i < boxes.size(); i++) {
          if(b1.collides(boxes[i])) {
            finJeu = true;
          }
        }
      }

      return 1;
    }

    Transform atlook (const Point & from, const Point & d, const Vector & up)
    {

      Vector dir= Vector(from, d);
      Vector right= cross(dir, normalize(up));
      Vector newUp= cross(right, dir);

      if (dir.x != 0.0 || dir.y != 0.0 || dir.z != 0.0) {
         dir= normalize(dir);
      }
      else {
        dir = Vector(0.0,0.0,0.1);
        dir = normalize(dir);
      }

      if (right.x != 0.0 || right.y != 0.0 || right.z != 0.0) {
        right= normalize(right);
      }
      else {
        right = Vector(0.1,0.0,0.0);
        right = normalize(right);
      }

      if (newUp.x != 0.0 || newUp.y != 0.0 || newUp.z != 0.0) {
        newUp= normalize(newUp);
      }
      else {
        newUp = Vector(0.0,0.1,0.0);
        newUp = normalize(newUp);
      }

      return Transform (
            right.x, newUp.x, -dir.x, from.x,
            right.y, newUp.y, -dir.y, from.y,
            right.z, newUp.z, -dir.z, from.z,
            0,       0,        0,     1);
    }

    Transform Lookat (const Point & from, const Point & d, const Vector & up)
    {

      Vector dir= Vector(from, d);
      Vector right= cross(dir, normalize(up));
      Vector newUp= cross(right, dir);

      if (dir.x != 0.0 || dir.y != 0.0 || dir.z != 0.0) {
         dir= normalize(dir);
      }
      else {
        dir = Vector(0.0,0.0,0.1);
        dir = normalize(dir);
      }

      if (right.x != 0.0 || right.y != 0.0 || right.z != 0.0) {
         right= normalize(right);
      }
      else {
        right = Vector(0.1,0.0,0.0);
        right = normalize(right);
      }

      if (newUp.x != 0.0 || newUp.y != 0.0 || newUp.z != 0.0) {
         newUp= normalize(newUp);
      }
      else {
        newUp = Vector(0.0,0.1,0.0);
        newUp = normalize(newUp);
      }

      Transform m(
            right.x, newUp.x, -dir.x, from.x,
            right.y, newUp.y, -dir.y, from.y,
            right.z, newUp.z, -dir.z, from.z,
            0,       0,        0,     1);

      return m.inverse();
    }


    static double getNorme(const Vector & p1)
    {
        return sqrt( (p1.x) * (p1.x) + (p1.y) * (p1.y) + (p1.z) * (p1.z) );
    }



protected:
    Mesh tube;
    Mesh objet;
    Mesh obstacle;
    GLuint texture;
    GLuint program;
    std::vector<Point> points;
    int derniere_direction;
    std::vector<Vector> orthogonaux;
    double r;
    int niveau;
    int alpha;
    unsigned indice;
    Text console;
    unsigned int lastTime = 0;
    std::vector<int> obstacles;
    std::vector<int> angles;
    bool finJeu;

    Box b1;
    std::vector<Box> boxes;

};

int main( int argc, char **argv )
{
    Tube tube;
    tube.run();

    return 0;
}
