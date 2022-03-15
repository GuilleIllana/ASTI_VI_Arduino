#ifndef _CUADRICULA_DISFUNCIONAL
#define _CUADRICULA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"
#include "casilla.h"


class Cuadricula
{
  public:
    Cuadricula(int rows, int cols, int obs_row[], int obs_col[], int nobs);
    Casilla *Planner(int ro, int co, int rf, int cf);
    int minDistance(int n);
    void printTablero();
    void printDistancia();
    Casilla* Tablero;
  private:
    int _rows;
    int _cols;
    
};

#endif
