#ifndef _CUADRICULA_DISFUNCIONAL
#define _CUADRICULA_DISFUNCIONAL


#include "robot.h"
#include "casilla.h"


class Cuadricula
{
  public:
    Cuadricula(int rows, int cols, int obs_row[], int obs_col[], int nobs);
    int Planner(int ro, int co, int rf, int cf, int* Recorrido);
    int minDistance(int n);
    void MovGenerator(int nR, int* Recorrido, int* Movimientos);
    void printTablero();
    void printDistancia();
  private:
    int _rows;
    int _cols;
    Casilla* Tablero;  
};

#endif
