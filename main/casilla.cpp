#include "casilla.h"

Casilla::Casilla(int row, int col, bool occup){
    _row = row;
    _col = col;
    _occup = occup;
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


void Casilla::setOccup(bool occup) {
  _occup = occup;
}
