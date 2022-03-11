#ifndef _CUADRICULA_DISFUNCIONAL
#define _CUADRICULA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"
#include "casilla.h"


class Cuadricula
{
  public:
    Cuadricula(int rows, int cols, int row_init, int col_init, int obs_row[], int obs_col[], int nobs);
    int *Planner(int ro, int co, int rf, int cf);
    int minDistance(int dist[], bool sptSet[]);
  private:
    int _rows;
    int _cols;
    int _posr;
    int _posc;
    Casilla** Tablero;
};

#endif
