
//! \file tuto9.cpp utilisation d'un shader 'utilisateur' pour afficher un tube Mesh

#include "mat.h"
#include "mesh.h"
#include "wavefront.h"

#include "orbiter.h"
#include "program.h"
#include "uniforms.h"
#include "draw.h"

#include "app_camera.h"      // classe Application a deriver

#include <time.h>
#include <stdlib.h>
#include <iostream>
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
    std::cout << "orthogonaux!" << std::endl;
    v = Vector(1,0,0);
    d = normalize(cross(a0,v));
    if (abs(d.x - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1 && abs(d.y - 0.0) < 0.1) {
      std::cout << "2" << std::endl;
      v = Vector(0,1,0);
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
        // etape 1 : charger un tube
        //srand(time(NULL));
        lastTime = SDL_GetTicks();

        unsigned int i = 0;
        int random;

        /*float tab_indices[3] = {0.0,0.0,0.0};
        unsigned int nb_points = 5;
        points.push_back(Point (tab_indices[0], tab_indices[1], tab_indices[2]));
        while (i < nb_points) {
            random = rand() % 3;
            tab_indices[random] = tab_indices[random] + deplace_p;
            points.push_back(Point (tab_indices[0], tab_indices[1], tab_indices[2]));
            i++;
        }*/
        // // etape 1 : charger un tube
        /*points.push_back(Point(0., 0., 1.));
        points.push_back(Point(0., 0., 2.));
        points.push_back(Point(0., 1., 2.));
        points.push_back(Point(1., 1., 2.));
        points.push_back(Point(1., 1., 3.));
        points.push_back(Point(2., 1., 3.));
        points.push_back(Point(2., 1., 2.));
        points.push_back(Point(2., 1., 1.));
        points.push_back(Point(2., 0., 0.));
        points.push_back(Point(2., -1., 0.));
        points.push_back(Point(1., -1., 0.));
        points.push_back(Point(1., 0., -1.));
        points.push_back(Point(-1, 0., 0.));*/


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

        int i_min = 500;
        double sqrt_min = sqrt( (0-points[i_min].x)*(0-points[i_min].x)
                  + (0-points[i_min].y)*(0-points[i_min].y)
                  + (deplace_p-points[i_min].z)*(deplace_p-points[i_min].z));
        for (unsigned int i = 500; i < points.size(); i++) {
          if (sqrt( (0-points[i].x)*(0-points[i].x)
                    + (0-points[i].y)*(0-points[i].y)
                    + (deplace_p-points[i].z)*(deplace_p-points[i].z)) < sqrt_min) {
                      i_min = i;
                      sqrt_min = sqrt( (0-points[i].x)*(0-points[i].x)
                                + (0-points[i].y)*(0-points[i].y)
                                + (deplace_p-points[i].z)*(deplace_p-points[i].z));
                    }
        }

        std::vector<std::vector<Point>> cercles;
        std::vector<std::vector<Vector>> norm;

        /*std::vector<Point> temp;

        for (unsigned int i = 3456 ; i < points.size(); i++) {
          points[i] = points[i-3456];
          //std::cout << points[i-1] << std::endl;
        }
        //std::cout << points.size() << std::endl;
*/
        //je construis les vecteurs orthogonaux à la courbe
        vecteur_orthogonal(points, orthogonaux);

        i_min = 50;
        sqrt_min = sqrt( (orthogonaux[0].x-points[i_min].x)*(orthogonaux[0].x-points[i_min].x)
                  + (orthogonaux[0].y-points[i_min].y)*(orthogonaux[0].y-points[i_min].y)
                  + (orthogonaux[0].z-points[i_min].z)*(orthogonaux[0].z-points[i_min].z));
        for (unsigned int i = 500; i < points.size(); i++) {
          if (sqrt( (orthogonaux[i].x-points[i_min].x)*(orthogonaux[i].x-points[i_min].x)
                    + (orthogonaux[i].y-points[i_min].y)*(orthogonaux[i].y-points[i_min].y)
                    + (orthogonaux[i].z-points[i_min].z)*(orthogonaux[i].z-points[i_min].z) < sqrt_min)) {
              i_min = i;
              sqrt_min = sqrt( (orthogonaux[i].x-points[i_min].x)*(orthogonaux[i].x-points[i_min].x)
                        + (orthogonaux[i].y-points[i_min].y)*(orthogonaux[i].y-points[i_min].y)
                        + (orthogonaux[i].z-points[i_min].z)*(orthogonaux[i].z-points[i_min].z));
          }
        }
        //std::cout << "best i = " << i_min << " : " << orthogonaux[i_min] << " - " << orthogonaux[0] << std::endl;
        points.erase(points.end() - 1, points.end());

        /*for (unsigned int i = 3456 ; i < orthogonaux.size(); i++) {
          orthogonaux[i] = orthogonaux[i-3456];
          //std::cout << points[i-1] << std::endl;
        }*/

        /*for (unsigned int i = 0 ; i < orthogonaux.size() ; i++) {
          std::cout << points[i] << " - " << orthogonaux[i] << std::endl;
          if (i == 3456) {
            std::cout << std::endl;
          }
        }
        std::cout << std::endl;*/
        /*for (unsigned int i = 3456 ; i < orthogonaux.size(); i++) {
          std::cout << orthogonaux[i] << std::endl;
        }*/
/*
        for (unsigned i = 0; i < points.size() - 3456; i++) {
          std::cout << points[i] << std::endl;
        }

        std::cout << std::endl;
        for (int i = 100; i >= 0; i--) {
          std::cout << points[points.size() - 1] << std::endl;
        }*/



        ///calcul rayon
        r = getNorme(orthogonaux[0]);  //rayon tube
        //génération des cercles
        generation_cercles(points, orthogonaux, cercles, norm);

        alpha = 0;
        niveau = 0;

        tube= Mesh(GL_TRIANGLES);
        //génération et dessin des trianges
        dessine_triangles(tube, cercles, norm);

        //charge l'objet
        objet= read_mesh("data/robot.obj");


        // etape 1 : creer le shader program
        program= read_program("projet/shaders/tube_color.glsl");
        program_print_errors(program);

        // etape 2 : creer une camera pour observer l'tube
        // construit l'englobant de l'tube, les extremites de sa boite englobante
        Point pmin, pmax;
        tube.bounds(pmin, pmax);
        indice = 120;


        // regle le point de vue de la camera pour observer l'tube
        camera().lookat(pmin, pmax);

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
        return 0;
    }

    // dessiner une nouvelle image
    int render( )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        //std::cout << orthogonaux.size() << std::endl;
        std::cout << indice << "/" << points.size() << std::endl;

        indice += niveau + 3;

        //generation_nouveaux_points()
        ;
        if (indice > points.size() - 1) {
          //std::cout << std::endl << points[points.size()-1] << std::endl;
          /*for (int i = 20; i > 0; i--) {
            std::cout << points[points.size() - i] << std::endl;
          }*/
          /*for (unsigned int i = 10; i > 0; i--) {
            std::cout << orthogonaux[indice - i] << std::endl;
          }*/
          /*indice -= 10;
          std::cout <<  Vector (Rotation(Vector (points[(indice+1)], points[indice]),alpha)(orthogonaux[indice])) << std::endl;
          indice +=10;
          std::cout <<  Vector (Rotation(Vector (points[(indice+1)], points[indice]),alpha)(orthogonaux[indice])) << std::endl;
          indice = 128;
          std::cout <<  Vector (Rotation(Vector (points[(indice+1)], points[indice]),alpha)(orthogonaux[indice])) << std::endl;*/

          niveau += 2;
          indice = 0;
          //std::cout <<   orthogonaux[indice] << std::endl;
          //std::cout <<   orthogonaux[indice+20] << std::endl;
          for (unsigned int i = 0; i < orthogonaux.size(); i++) {
            std::cout << orthogonaux[i] << std::endl;
            if (orthogonaux[i].x == orthogonaux[0].x && orthogonaux[i].y == orthogonaux[0].y && orthogonaux[i].z == orthogonaux[0].z) {
              std::cout << "!!!!!!!!!!" << i << "!!!!!!!!!" << std::endl;
            }
          }
          std::cout << std::endl << std::endl;
          //std::cout << points[indice] << std::endl << std::endl;
          /*for (int i = -10; i < 20; i++) {
            std::cout << points[i+indice] << std::endl;
          }*/


        }


        Point p = points[indice];
        Vector d(points[(indice+1)], points[indice]);


        if(key_state(SDLK_LEFT))
            alpha = (alpha + 2) % 360;     // tourne vers la gauche
        if(key_state(SDLK_RIGHT))
            alpha = (alpha - 2) % 360;      // tourne vers la droite

        Transform R = Rotation(d,alpha);
        Vector n(orthogonaux[indice]);
        Vector na(R(n));
        Point pos_objet = p + r * na;

        //std::cout << indice << std::endl;

        Transform m_transform_objet = atlook(pos_objet, pos_objet + d, na)*Translation(0,r/8,0)*Scale(0.09,0.09,0.09);

        Transform m_transform_camera = Translation(2.5*na)*Translation(50*d);
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

        return 1;
    }

    Transform atlook (const Point & from, const Point & d, const Vector & up)
    {
          Vector dir= normalize( Vector(from, d) );
          Vector right= normalize( cross(dir, normalize(up)) );
          Vector newUp= normalize( cross(right, dir) );

          return Transform (
              right.x, newUp.x, -dir.x, from.x,
              right.y, newUp.y, -dir.y, from.y,
              right.z, newUp.z, -dir.z, from.z,
              0,       0,        0,     1);
    }

    static double getNorme(const Vector & p1)
    {
        return sqrt( (p1.x) * (p1.x) + (p1.y) * (p1.y) + (p1.z) * (p1.z) );
    }

    void generation_nouveaux_points(){
        int random;
        unsigned int currentTime = SDL_GetTicks();
        if (currentTime > lastTime + 1000){
            float tab_indices[3] = {points[points.size() - 1].x, points[points.size() - 1].y, points[points.size() - 1].z};

            random = rand() % 3;

            std::vector<Point> tmp;
            tmp.push_back(points[points.size() - 1]);
            if (derniere_direction == 3)
              tab_indices[2] -= deplace_p;
            else
              tab_indices[derniere_direction] += deplace_p;
            tmp.push_back(Point (tab_indices[0], tab_indices[1], tab_indices[2]));


            std::cout << tab_indices[0] << " ; " << tab_indices[1] << " ; " << tab_indices[2] << std::endl;

            tab_indices[random] = tab_indices[random] + deplace_p;
            derniere_direction = random;


            tmp.push_back(Point (tab_indices[0], tab_indices[1], tab_indices[2]));
            //std::cout << tab_indices[0] << " ; " << tab_indices[1] << " ; " << tab_indices[2] << std::endl;

            for (int i = 0; i < 10; i++) {
                chaikin(tmp);
            }

            std::vector<Vector> orthogonaux_tmp;
            std::vector<std::vector<Point>> cercles_tmp;
            std::vector<std::vector<Vector>> norm_tmp;

            //je construis les vecteurs orthogonaux à la courbe
            vecteur_orthogonal(tmp, orthogonaux_tmp);

            tmp.erase(tmp.begin(), tmp.begin() + 2);

            //génération des cercles
            generation_cercles(tmp, orthogonaux_tmp, cercles_tmp, norm_tmp);

            //génération et dessin des trianges
            dessine_triangles(tube, cercles_tmp, norm_tmp);

            int s = points.size() - 1;

            points.insert( points.end(), tmp.begin(), tmp.end() );

            for (int i = s - 0; i < s + 0; i++) {
              std::cout << points[i] << " ; " << std::flush;
            }

            orthogonaux.insert( orthogonaux.end(), orthogonaux_tmp.begin(), orthogonaux_tmp.end() );


            std::cout << std::endl;
            lastTime = currentTime;
        }

    }




protected:
    Mesh tube;
    Mesh objet;
    GLuint texture;
    GLuint program;
    std::vector<Point> points;
    int derniere_direction;
    std::vector<Vector> orthogonaux;
    double r;
    int niveau;
    int alpha;
    unsigned indice;
    unsigned int lastTime = 0;

};

int main( int argc, char **argv )
{
    Tube tube;
    tube.run();

    return 0;
}
