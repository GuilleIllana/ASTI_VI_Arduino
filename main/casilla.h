#ifndef _CASILLA_DISFUNCIONAL
#define _CASILLA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"


class Casilla
{
  public:
    Casilla(int row, int col, bool occup, float dist);
    int getRow();
    int getCol();
    int getOccup();
    float getDist();
    bool getExpl();
    void setOccup(bool occup);
    void setDist(float dist);
    void setExpl(bool expl);
  private:
    int _row;
    int _col;
    float _dist;
    bool _occup;
    bool _expl;
};

#endif
