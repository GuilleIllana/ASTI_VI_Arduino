#ifndef _CASILLA_DISFUNCIONAL
#define _CASILLA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"


class Casilla
{
  public:
    Casilla(int row, int col, bool occup);
    int getRow();
    int getCol();
    int getOccup();
    void setOccup(bool occup);
  private:
    int _row;
    int _col;
    bool _occup;
};

#endif
