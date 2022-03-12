#include "casilla.h"

Casilla::Casilla(int row, int col, bool occup, float dist){
    _row = row;
    _col = col;
    _occup = occup;
    _dist = dist;
}


int Casilla::getRow() {
  return _row;
}


int Casilla::getCol() {
  return _col;
}


int Casilla::getOccup() {
  return _occup;
}


float Casilla::getDist() {
  return _dist;
}


bool Casilla::getExpl() {
  return _expl;
}


void Casilla::setOccup(bool occup) {
  _occup = occup;
}


void Casilla::setDist(float dist) {
  _dist = dist;
}


void Casilla::setExpl(bool expl) {
  _expl = expl;
}
