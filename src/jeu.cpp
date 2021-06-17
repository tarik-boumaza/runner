#include "tube.cpp"
#include "texture.h"

#include <unistd.h>

class Jeu : public AppCamera
{
public:
    // constructeur : donner les dimensions de l'image, et eventuellement la version d'openGL.
    Jeu( ) : AppCamera(1024, 640) {}

    int init( )
    {
      //Initialisation du tube
      tube.init();
      m_tube = tube.getMesh();

      nb_obstacles = 30;

      //charge l'objet
      objet= read_mesh("projet/data/voiture.obj");

      // Initialisation du tableau d'obstacles
      initObstacles();

      //objet.default_color(Green()) ;
      Point pmin_box, pmax_box ;
      objet.bounds(pmin_box, pmax_box) ;
      b1 = Box(pmin_box, pmax_box) ;

      //charger les obstacles
      obstacle = read_mesh("projet/data/obstacle.obj");
      obstacle.default_color(Red());

      obstacle.bounds(pmin_box, pmax_box) ;

      for(unsigned int i = 0; i < nb_obstacles; i++) {
        boxes.push_back(Box(pmin_box, pmax_box));
      }

      //Initialisation des paramètres du jeu
      finJeu = false;
      alpha = 0;
      niveau = 1;
      indice = 10;

      // etape 1 : creer le shader program
      program = read_program("projet/src/shaders/tube_color.glsl");
      program_print_errors(program);

      program_voiture = read_program("projet/src/shaders/texture.glsl");
      program_print_errors(program_voiture);

      program_obstacle = read_program("projet/src/shaders/texture.glsl");
      program_print_errors(program_obstacle);
      // etape 2 : creer une camera pour observer l'tube
      // construit l'englobant de l'tube, les extremites de sa boite englobante
      Point pmin, pmax;
      m_tube.bounds(pmin, pmax);


      console = create_text();
      texture_route = read_texture(0, "projet/data/route3-min.jpg");

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      texture_voiture = read_texture(0, "projet/data/couleur-voiture.jpg");
      texture_obstacle =  read_texture(0, "projet/data/couleur_barriere.jpg");


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
        m_tube.release();
        objet.release();
        release_text(console);
        glDeleteTextures(1, &texture_route);
        glDeleteTextures(1, &texture_voiture);
        return 0;
    }

    // dessiner une nouvelle image
    int render( )
    {

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (!finJeu){
        indice += niveau + 4;
        if (indice > tube.getNbPoints() - 2) {
          niveau += 1;
          indice = 1;
        }
      std::string str = "En cours : NIVEAU " + std::to_string(niveau);
      char char_array[str.size() + 5];
      strcpy(char_array, str.c_str());
      printf(console, 106, 0, char_array);
      draw(console, window_width(), window_height());
      if(key_state(SDLK_LEFT))
        alpha = (alpha + 2) % 360;     // tourne vers la gauche
      if(key_state(SDLK_RIGHT))
        alpha = (alpha - 2) % 360;      // tourne vers la droite
      }
      else {
        sleep(1);
        std::string str = "                    ";
        char char_array[str.size() + 5];
        strcpy(char_array, str.c_str());
        printf(console, 106, 0, char_array);
        str = "Perdu : NIVEAU " + std::to_string(niveau);
        strcpy(char_array, str.c_str());
        printf(console, 55, 11, char_array);
        if (niveau == 1) {
          str = "pas de panique, personne ne le saura...";
          strcpy(char_array, str.c_str());
          printf(console, 0, 0, char_array);
        }
        draw(console, window_width(), window_height());
        return 1;
      }

      Point p = tube.getPoint(indice);
      Vector d(tube.getPoint(indice+1), tube.getPoint(indice));
      Transform R = Rotation(d,alpha);
      Vector n(tube.getOrthogonal(indice));
      Vector na(R(n));
      Point pos_objet = p + tube.getR() * na;

      Transform m_transform_objet = atlook(pos_objet, pos_objet + d, na)*Translation(0,tube.getR()/6,0)*Scale(2,2,2);

      Transform m_transform_camera = Translation(1.5*na)*Translation(50*d);

      // etape 2 : dessiner tube avec le shader program
      // configurer le pipeline
      glUseProgram(program);

      // configurer le shader program
      // . recuperer les transformations
      /*Transform model= Identity();
      Transform view= camera().view();
      Transform projection= camera().projection(window_width(), window_height(), 45);*/

      Transform model= Identity();
      Transform view= lookat(m_transform_camera(pos_objet),pos_objet, na);
      Transform projection= Perspective(90, (float) window_width() / (float) window_height(), .1f, 1000.f);

      // . composer les transformations : model, view et projection
      Transform mvp= projection * view * model;

      // . parametrer le shader program :
      //   . transformation : la matrice declaree dans le vertex shader s'appelle mvpMatrix
      program_uniform(program, "mvpMatrix", mvp);
      program_uniform(program, "modelMatrix", model);
      program_uniform(program, "viewInvMatrix", Inverse(view));
       //   . utilisation d'une texture configuree sur l'unite 0, le fragment shader declare "uniform sampler2D texture0;"
      program_use_texture(program, "texture0", 0, texture_route);
      //   . ou, directement en utilisant openGL :
      //   int location= glGetUniformLocation(program, "mvpMatrix");
      //   glUniformMatrix4fv(location, 1, GL_TRUE, mvp.buffer());
      // . parametres "supplementaires" :
      //   . couleur des pixels, cf la declaration 'uniform vec4 color;' dans le fragment shader
      //program_uniform(program, "color", vec4(1, 1, 0, 1));
      //   . ou, directement en utilisant openGL :
      //   int location= glGetUniformLocation(program, "color");
      //   glUniform4f(location, 1, 1, 0, 1);
      m_tube.draw(program, /* use position */ true, /* use texcoord */true, /* use normal */ true, /* use color */ false, /* use material index*/ false);

      glUseProgram(program_voiture);

      Transform mvp_model = projection * view * m_transform_objet;
      program_uniform(program_voiture, "mvpMatrix", mvp_model);
      program_use_texture(program_voiture, "texture0", 0, texture_voiture);

      // go !
      // indiquer quels attributs de sommets du mesh sont necessaires a l'execution du shader.
      // tuto9_color.glsl n'utilise que position. les autres de servent a rien.
      objet.draw(program_voiture, /* use position */ true, /* use texcoord */true, /* use normal */ false, /* use color */ false, /* use material index*/ false);

      //draw(objet, m_transform_objet,/*camera()*/ view, projection);


      /////////////////////////////////////  Ajout d'obstacle /////////////////////////////////
      glUseProgram(program_obstacle);

      std::vector<Transform> m_transform_obstacles;
      unsigned int i;
      for(i = 0; i < obstacles.size(); i++) {
        Point p_ob = tube.getPoint(obstacles[i]);
        Vector d_ob(tube.getPoint(obstacles[i]), tube.getPoint(obstacles[i]+1));
        Transform R_ob = Rotation(d_ob, angles[i]);
        Vector n_ob(tube.getOrthogonal(obstacles[i]));
        Vector na_ob(R_ob(n_ob));
        Point pos_ob = p_ob + tube.getR() * na_ob;
        boxes[i].T = atlook(pos_ob, pos_ob + d_ob, na_ob)*Translation(0,0.05,0)*Scale(0.1,0.1,0.1);

        Transform mvp_obstacle = projection * view * boxes[i].T;
        program_uniform(program_obstacle, "mvpMatrix", mvp_obstacle);
        program_use_texture(program_obstacle, "texture0", 0, texture_obstacle);

        obstacle.draw(program_obstacle, /* use position */ true, /* use texcoord */true, /* use normal */ false, /* use color */ false, /* use material index*/ false);
        //draw(obstacle, boxes[i].T,/*camera()*/ view, projection);
      }

      if (!finJeu) {
        b1.T = m_transform_objet ;
        for(i = 0; i < boxes.size(); i++) {
          if(b1.collides(boxes[i])) {
            finJeu = true;
            play_sound();
          }
        }
      }
      return 1;
    }

protected:
  Tube tube;
  Mesh m_tube;
  GLuint texture_route;
  GLuint program;

  Mesh objet;
  Mesh obstacle;

  GLuint texture_voiture;
  GLuint texture_obstacle;

  GLuint program_voiture;
  GLuint program_obstacle;



  int niveau;
  int alpha;
  unsigned indice;
  Text console;
  unsigned int nb_obstacles;
  unsigned int lastTime = 0;
  std::vector<int> obstacles;
  std::vector<int> angles;
  bool finJeu;

  Box b1;
  std::vector<Box> boxes;

private:

  void initObstacles() {
    obstacles.clear();
    angles.clear();
    unsigned int i = 1;
    int random = 500;
    obstacles.push_back(random);
    random = rand()% 360;
    angles.push_back(random);
    while(i < nb_obstacles) {
      random = rand()%(tube.getNbPoints()/(nb_obstacles+5)) + i*tube.getNbPoints()/(nb_obstacles+5) + 500;
      obstacles.push_back(random);
      random = rand()% 360;
      angles.push_back(random);
      i++;
    }
  }

};

int main( int argc, char **argv )
{
    Jeu jeu;
    jeu.run();

    return 0;
}
