#include "cuadricula.h"
#include <QTRSensors.h>
#include "Arduino.h"
Cuadricula::Cuadricula(int rows, int cols, int obs_row[], int obs_col[], int nobs) {
    _rows = rows;
    _cols = cols;
    
    Tablero = (Casilla*)malloc(rows * cols * sizeof(Casilla));
        
    // Construcción del tablero
    for (int i = 0; i < _rows; i++){
        for (int j = 0; j < _cols; j++){
          Tablero[i*_rows + j] = Casilla(i,j);
          //Serial.println(i*_rows + j);
        }
    }
    // Adición de obstáculos
    for (int i = 0; i < nobs; i++){
      Tablero[rows*obs_row[i]+obs_col[i]].setAvail(false);
      //Serial.println("hola");
    }  
}


// A utility function to find the vertex with minimum distance value
int Cuadricula::minDistance(int n)  {
  // Inicialización de las posibles casillas
  int idx[] = {n-_rows, n+_rows, n+1, n-1};
  int idx_g[4];
  int count = 0;

  // Comprobación de cada casilla
  for (int i = 0; i < 4; i++) {
    int dif_r = abs(Tablero[idx[i]].getRow() - Tablero[n].getRow());
    int dif_c = abs(Tablero[idx[i]].getCol() - Tablero[n].getCol());
    int sum = dif_r + dif_c;
    if ((idx[i] < (_rows*_cols)) ||( idx[i] >= 0 )|| (sum == 1)) {
      idx_g[count] = idx[i];
      count++;
    }
  }
  
  // Initialize min value
  float val_min = Tablero[idx_g[0]].getDist();
  int min_index = idx_g[0];
  
  // Obtención del índice de la casilla con las distancia mínima
  for (int v = 0; v < count; v++) {
    if ((Tablero[idx_g[v]].getExpl() == false) && (Tablero[idx_g[v]].getDist() <= val_min) && (Tablero[idx_g[v]].getAvail() == true)) {
      val_min = Tablero[idx_g[v]].getDist();
      min_index = idx_g[v];
    }
  }
  return min_index;
}
extern int* Recorrido;
int* Cuadricula::Planner(int ro, int co, int rf, int cf) {
  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation 
  int count = 0, idx_ant = ro*_rows+co;
  int idx;
  
  Recorrido = (int*)malloc(36*sizeof(int));
  
  // Initialize all distances and explore as false
  for (int i = 0; i < _rows; i++) {
    for (int j = 0; j < _cols; j++) {
      Tablero[i*_rows + j].setDist(sqrt(sq(rf-i)+sq(cf-j)));
      Tablero[i*_rows + j].setExpl(false);
      if ((i == 0) || (j == 0) || (i == (_rows-1)) || (j == (_cols-1)))
        Tablero[i*_rows + j].setAvail(false);
      else
        Tablero[i*_rows + j].setAvail(true);
    }
  }
  Tablero[ro*_rows + co].setAvail(true);
  Tablero[rf*_rows + cf].setAvail(true); 
  
  Recorrido[0] = ro*_rows + co;
  count++;
  
  // Find shortest path for all vertices
  while (idx != (rf*_rows + cf)) {
    // Pick the minimum distance vertex from the set of vertices not
    // yet processed. u is always equal to src in the first iteration.
    idx = minDistance(idx_ant);
 
    // Mark the picked vertex as processed
    Tablero[idx].setExpl(true);
 
    Recorrido[count] = idx;
    count++;

    idx_ant = idx;
 }
// for (int i = 0; i < count; i++){
//    Serial.print(Recorrido[i]);
//    Serial.print("\t"); 
// }

 return Recorrido;
}


void Cuadricula::printTablero() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          //Serial.print(Tablero[i*_rows + j].getAvail());
          //Serial.print('\t');
        }
    //Serial.println();
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
  
