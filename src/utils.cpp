#include "mesh.h"
#include "orbiter.h"
#include "program.h"
#include "uniforms.h"
#include "draw.h"
#include "text.h"
#include "box.hpp"
#include "app_camera.h"      // classe Application a deriver
#include "wavefront.h"
#include "mat.h"

#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>        // std::abs

static void  chaikin(std::vector<Point> & points){
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

static Vector rotation (Vector & v1, Vector & v2, Vector & d){
  Transform t = Rotation(v1,v2);
  return t(d);
}

static void vecteur_orthogonal (std::vector<Point> & points, std::vector<Vector> & orthogonaux){
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


static void points_cercle(Point & p1, Point & p2, Vector & v, std::vector<Point> & pc, std::vector<Vector> & nm){
  Vector axe(p1,p2);
  float angle = 10.0;
  Transform tr = Rotation(axe, angle);
  Transform tt;
  Point p, pt;
  for(int i = 0; i < 36; i++){
    v = tr(v);
    tt = Translation(v);
    p = Point (p1.x-v.x, p1.y-v.y, p1.z-v.z);
    pt = Point (tt(p));
    pc.push_back(p);
    nm.push_back(Vector(pt,p));
  }
}

static void generation_cercles(std::vector<Point> & points, std::vector<Vector> & orthogonaux, std::vector<std::vector<Point>> & cercles,std::vector<std::vector<Vector>> & normales){
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


static float getNormeX(const Point & p1, const Point & p2)
{
    return abs(p1.x - p2.x);
}

static float getNormeY(const Point & p1, const Point & p2)
{
    return abs(p1.y - p2.y);
}

static float getNormeZ(const Point & p1, const Point & p2)
{
    return abs(p1.z - p2.z);
}

static float getNorme(const Point & p1, const Point & p2) {
  return sqrt( (p1.x - p2.x)*(p1.x - p2.x)
              + (p1.y - p2.y)*(p1.y - p2.y)
              + (p1.z - p2.z)*(p1.z - p2.z) );
}


static float longueur_tube(const std::vector<Point> & points) {
  double longueur = 0.0;
  unsigned int i = 0;
  while((abs(getNorme(points[0],points[i])) > 0.01
          || i < 50)
        && i < points.size()) {
    longueur += getNorme(points[i], points[i+1]);
    i++;
  }
  //longueur += getNormeZ(points[0],points[points.size()-1]);
  return longueur;
}

static void dessine_triangles(Mesh& m, const std::vector<std::vector<Point>> & cercles, const std::vector<std::vector<Vector>> & norm, const float & lgt){
  unsigned int i, j,a, b, c, d, la, lb, lc, ld;
  unsigned int repete = 100;
  float lgp = 0.0;
  float lgps = 0.0;
  float ang,angs;

  for(i = 0; i < cercles.size() - 1; i++){
    if(i > 0) {
      lgp += getNorme(cercles[i][1], cercles[i+1][1]);
    }
    lgps = lgp + getNorme(cercles[i][1], cercles[i+1][1]);

    for(j = 0; j < cercles[i].size() - 1; j++){
      ang = (float(j)*30)/360;
      angs = (float(j+1)*30)/360;
      //std::cout<<lgp/lgt * repete << " , " << angs << std::endl;
      a = m.texcoord(lgp/lgt * repete, ang).normal(norm[i][j]).vertex(cercles[i][j]);
      b = m.texcoord(lgps/lgt * repete, ang).normal(norm[i+1][j]).vertex(cercles[i+1][j]);
      c = m.texcoord(lgps/lgt * repete, angs).normal(norm[i+1][j+1]).vertex(cercles[i+1][j+1]);
      d = m.texcoord(lgp/lgt * repete, angs).normal(norm[i][j+1]).vertex(cercles[i][j+1]);

      m.triangle(a,c,b);
      m.triangle(a,d,c);

      if(j == cercles[i].size() - 2){
        la = m.texcoord(lgp/lgt * repete, angs).normal(norm[i][j+1]).vertex(cercles[i][j+1]);
        lb = m.texcoord(lgps/lgt * repete, angs).normal(norm[i+1][j+1]).vertex(cercles[i+1][j+1]);
        lc = m.texcoord(lgps/lgt * repete, 0).normal(norm[i+1][0]).vertex(cercles[i+1][0]);
        ld = m.texcoord(lgp/lgt * repete, 0).normal(norm[i][0]).vertex(cercles[i][0]);

        m.triangle(la,lc,lb);
        m.triangle(la,ld,lc);
      }
    }
  }
}

static Transform atlook (const Point & from, const Point & d, const Vector & up) {

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


static Transform lookat (const Point & from, const Point & d, const Vector & up) {
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

static void play_sound() {
  SDL_AudioSpec wav_spec;
  Uint32 wav_length;
  Uint8 *wav_buffer;
  // Load the WAV //
  if (SDL_LoadWAV("projet/data/crash.wav", &wav_spec, &wav_buffer, &wav_length) == NULL) {
      fprintf(stderr, "Could not open test.wav: %s\n", SDL_GetError());
  } else {
      // Do stuff with the WAV data, and then... //
      SDL_AudioDeviceID deviceId = SDL_OpenAudioDevice(NULL, 0, &wav_spec, NULL, 0);
      int success = SDL_QueueAudio(deviceId, wav_buffer, wav_length);
      SDL_PauseAudioDevice(deviceId, 0);
      SDL_Delay(3000);
      SDL_CloseAudioDevice(deviceId);
      SDL_FreeWAV(wav_buffer);
  }
}
