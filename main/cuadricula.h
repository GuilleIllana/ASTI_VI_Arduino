#ifndef _CUADRICULA_DISFUNCIONAL
#define _CUADRICULA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"


class Cuadricula
{
  public:
    Cuadricula(int rows, int cols, int row_init, int col_init);
  private:
    int _rows;
    int _cols;
    int _posr;
    int _posc;
};

#endif
