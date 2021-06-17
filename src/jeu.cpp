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

      srand(time(NULL));
      // Initialisation du tube
      tube.init();
      m_tube = tube.getMesh();

      // Charger la voiture
      objet= read_mesh("projet/data/obj/car.obj");

      // Initialisation du tableau d'obstacles
      nb_obstacles = 35;
      initObstacles();

      // Initialisation des boites englobantes
      Point pmin_box, pmax_box ;
      objet.bounds(pmin_box, pmax_box) ;
      // Boite englobante de la voiture
      b1 = Box(pmin_box, pmax_box) ;

      // Charger les obstacles
      obstacle = read_mesh("projet/data/obj/obstacle.obj");
      obstacle.default_color(Red());

      // Boites englobantes des obstacles
      obstacle.bounds(pmin_box, pmax_box) ;
      for(unsigned int i = 0; i < nb_obstacles; i++) {
        boxes.push_back(Box(pmin_box, pmax_box));
      }

      //Initialisation des paramètres du jeu
      finJeu = false;
      alpha = 0;
      niveau = 1;
      indice = 10;

      // Console pour affichage du niveau
      console = create_text();


      /*** Shader ***/
      // Creer le shader program pour le tube
      program = read_program("projet/src/shaders/tube_color.glsl");
      program_print_errors(program);

      // Creer le shader program pour appliquer une texture à la voiture
      program_voiture = read_program("projet/src/shaders/texture.glsl");
      program_print_errors(program_voiture);

      // Creer le shader program pour appliquer une texture aux obstacles
      program_obstacle = read_program("projet/src/shaders/texture.glsl");
      program_print_errors(program_obstacle);


      /*** Textures ***/

      texture_route = read_texture(0, "projet/data/textures/route.jpg");
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      texture_drapeau = read_texture(1, "projet/data/textures/drapeau.jpg");
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      texture_voiture = read_texture(0, "projet/data/textures/bleu-peint.jpg");

      texture_obstacle =  read_texture(0, "projet/data/textures/couleur_barriere.jpg");


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

      if (!finJeu) {
        indice += niveau + 2;
        if (indice > tube.getNbPoints() - 2) {
          niveau += 1;
          indice = 1;
          alpha = 0;
        }

        // Afichage du niveau
        std::string str = "En cours : NIVEAU " + std::to_string(niveau);
        char char_array[str.size() + 5];
        strcpy(char_array, str.c_str());
        printf(console, 106, 0, char_array);
        draw(console, window_width(), window_height());

        // Deplacement de la voiture (gauche ou droite)
        if(key_state(SDLK_LEFT))
          alpha = (alpha + 2) % 360;     // tourne vers la gauche
        if(key_state(SDLK_RIGHT))
          alpha = (alpha - 2) % 360;      // tourne vers la droite
      }

      else {
        // Affichage texte de fin de partie
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

      /**  Matrice de transformation de la voiture   **/
      Point p = tube.getPoint(indice);
      Vector d(tube.getPoint(indice+1), tube.getPoint(indice));
      Transform R = Rotation(d,alpha);
      Vector n(tube.getOrthogonal(indice));
      Vector na(R(n));
      Point pos_objet = p + tube.getR() * na;
      Transform m_transform_objet = atlook(pos_objet, pos_objet + d, na)*Translation(0,tube.getR()/6,0)*Scale(0.06,0.06,0.06);
      b1.T = m_transform_objet; // ajout de la matrice de transformation à la boite englobante

      // Configurer le pipeline pour dessiner le tube
      glUseProgram(program);

      Transform model= Identity();
      Transform m_transform_camera = Translation(1.5*na)*Translation(50*d);
      Transform view= lookat(m_transform_camera(pos_objet),pos_objet, na);
      Transform projection= Perspective(90, (float) window_width() / (float) window_height(), .1f, 1000.f);

      // Composer les transformations : model, view et projection
      Transform mvp= projection * view * model;

      program_uniform(program, "mvpMatrix", mvp);
      program_uniform(program, "modelMatrix", model);
      program_uniform(program, "viewInvMatrix", Inverse(view));
      program_use_texture(program, "texture0", 0, texture_route);
      program_use_texture(program, "texture1", 1, texture_drapeau);

      m_tube.draw(program, /* use position */ true, /* use texcoord */true, /* use normal */ true, /* use color */ false, /* use material index*/ false);


      // Configurer le pipeline pour dessiner la voiture
      glUseProgram(program_voiture);

      Transform mvp_model = projection * view * m_transform_objet;
      program_uniform(program_voiture, "mvpMatrix", mvp_model);
      program_use_texture(program_voiture, "texture0", 0, texture_voiture);

      objet.draw(program_voiture, /* use position */ true, /* use texcoord */true, /* use normal */ false, /* use color */ false, /* use material index*/ true);


      /**  Gestion des obstacles  **/

      // Configurer le pipeline pour dessiner les obstacles
      glUseProgram(program_obstacle);

      unsigned int i;
      for(i = 0; i < obstacles.size(); i++) {

        /**  Matrice de transformation de l'obstacle  **/
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
      }

      if (!finJeu) {
        /** Gestion des collisions **/
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
  Mesh objet;
  Mesh obstacle;

  GLuint texture_drapeau;
  GLuint texture_route;
  GLuint program;
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

  // Génération aléatoire de la position des obstacles sur le tube
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
