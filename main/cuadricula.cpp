#include "cuadricula.h"
#include <QTRSensors.h>
Cuadricula::Cuadricula(int rows, int cols, int obs_row[], int obs_col[], int nobs) {
    _rows = rows;
    _cols = cols;
    _posr = 0;
    _posc = 0;
    
    Tablero = (Casilla*)malloc(rows * cols * sizeof(Casilla));
        
    // Calling constructor
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
          Tablero[i*rows + j] = Casilla(i,j,true,0);

    // Adición de obstáculos
    for (int i = 0; i < nobs; i++)
      Tablero[rows*obs_row[i]+obs_col[i]].setAvail(false);
    
}
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int Cuadricula::minDistance(int n)  {
    // Initialize min value
    float min = 99.0;
    int min_index;
    int idx[] = {n-_rows, n+_rows, n+1, n-1};
    int count = 4;
    for (int i = 1; i < 4; i++){
      if (idx[i] > _rows*_cols - 1 || idx[i] < 0) {
        for (int j = i; j < 4; ++j)
          idx[j] = idx[j+1];
        count--;
      }
    }
          
    for (int v = 0; v < count; v++)
        if (Tablero[idx[v]].getExpl() == false && Tablero[idx[v]].getDist() <= min && Tablero[idx[v]].getAvail() == true)
            min = Tablero[idx[v]].getDist(), min_index = idx[v];
 
    return min_index;
}

Casilla* Cuadricula::Planner(int ro, int co, int rf, int cf) {
  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation 
  int count = 0, idx = ro*_rows+co;
  Casilla *Recorrido;
  
  // Initialize all distances and stpSet[] as false
  for (int i = 0; i < _rows; i++)
    for (int j = 0; j < _cols; j++) {
      Tablero[i*_rows + j].setDist(sqrt(sq(rf-i)+sq(cf-j)));
      Tablero[i*_rows + j].setExpl(false);
      if (i == 0 || j == 0 || i == _rows-1 || j == _cols-1)
        Tablero[i*_rows + j].setAvail(false);
      else
        Tablero[i*_rows + j].setAvail(true);
    }
    
  Tablero[ro*_rows + co].setAvail(true);
  Tablero[rf*_rows + cf].setAvail(true); 

  // printTablero();
  printDistancia();
  
  Recorrido[count] = Tablero[ro*_rows + co];
  count++;
  
  Serial.print(idx);
  Serial.println();
  // Find shortest path for all vertices
  while (idx != rf*_rows + cf) {
    // Pick the minimum distance vertex from the set of vertices not
    // yet processed. u is always equal to src in the first iteration.
    idx = minDistance(idx);
 
    // Mark the picked vertex as processed
    Tablero[idx].setExpl(true);
 
    Recorrido[count] = Tablero[idx];
    Serial.print(count);
    // Serial.print(Recorrido[count].getRow(),Recorrido[count].getCol());
    Serial.println();
    count++;
 }
 return Recorrido;
}


void Cuadricula::printTablero() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          Serial.print(Tablero[i*_rows + j].getAvail());
          Serial.print('\t');
        }
    Serial.println();
    }
}


void Cuadricula::printDistancia() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          Serial.print(Tablero[i*_rows + j].getDist());
          Serial.print('\t');
        }
    Serial.println();
    }
}
  
