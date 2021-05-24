
//! \file tuto9.cpp utilisation d'un shader 'utilisateur' pour afficher un tube Mesh

#include "mat.h"
#include "mesh.h"
#include "wavefront.h"

#include "orbiter.h"
#include "program.h"
#include "uniforms.h"
#include "draw.h"

#include "app_camera.h"      // classe Application a deriver


void  chaikin(std::vector<Point> & points){
  std::vector<Point> res;
  for(unsigned int i=0; i < points.size() - 1; i++){
    Point m1 = points[i]+ 0.25*Vector(points[i],points[i+1]);
    Point m2 = points[i]+ 0.75*Vector(points[i],points[i+1]);
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
  Vector a0(points[0],points[1]);
  Vector v(1,0,0);
  Vector d = normalize(cross(a0,v))*0.1;
  orthogonaux.push_back(d);
  for(unsigned int i = 0; i < points.size() - 2; i++){
    Vector a0(points[i],points[i+1]);
    Vector a1(points[i+1], points[i+2]);
    d = rotation(a0, a1,d);
    orthogonaux.push_back(d);
  }
}


void points_cercle(Point & p1, Point & p2, Vector & v, std::vector<Point> & pc, std::vector<Vector> & nm){
  Vector axe(p1,p2);
  float angle = 30.0;
  Transform tr = Rotation(axe, angle);
  for(int i = 0; i < 12; i++){
    v = tr(v);
    Transform tt = Translation(v);
    Point p(p1.x-v.x, p1.y-v.y, p1.z-v.z);
    Point pt(tt(p));
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
        points.push_back(Point(0., 1., 0.));
        points.push_back(Point(0., 1., 1.));
        points.push_back(Point(1., 1., 1.));
        points.push_back(Point(1., 0., 1.));
        points.push_back(Point(0., 0., 1.));
        points.push_back(Point(0., 0., 0.));
        points.push_back(Point(1., 0., 0.));
        points.push_back(Point(1., 1., 0.));

        for(unsigned int i=0; i < 10; i++){
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


        tube= Mesh(GL_TRIANGLES);
        //génération et dessin des trianges
        dessine_triangles(tube, cercles, norm);

        //charge l'objet
        objet= read_mesh("data/cube.obj");


        // etape 1 : creer le shader program
        program= read_program("projet/shaders/tube_color.glsl");
        program_print_errors(program);

        // etape 2 : creer une camera pour observer l'tube
        // construit l'englobant de l'tube, les extremites de sa boite englobante
        Point pmin, pmax;
        tube.bounds(pmin, pmax);

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


        // etape 2 : dessiner tube avec le shader program
        // configurer le pipeline
        glUseProgram(program);

        // configurer le shader program
        // . recuperer les transformations
        Transform model= Identity();
        Transform view= camera().view();
        Transform projection= camera().projection(window_width(), window_height(), 45);

        // . composer les transformations : model, view et projection


        Transform mvp= projection * view * model;

        Point p = points[0];
        Vector d(points[1], points[0]);
        double alpha = 0.0;
        Transform R = Rotation(d,alpha);
        Vector n(orthogonaux[0]);
        Vector na(R(n));
        Point pos_objet = p + r * na;


        Transform m_transform_objet = Scale(0.1,0.1,0.1) * atlook(pos_objet, pos_objet + d, na);




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
        draw(objet, m_transform_objet, camera());

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

    static double getNorme(const Vector & p1) {
        return sqrt( (p1.x) * (p1.x) + (p1.y) * (p1.y) + (p1.z) * (p1.z) );
    }



protected:
    Mesh tube;
    Mesh objet;
    GLuint texture;
    GLuint program;
    std::vector<Point> points;
    std::vector<Vector> orthogonaux;
    double r;
};


int main( int argc, char **argv )
{
    Tube tube;
    tube.run();

    return 0;
}
