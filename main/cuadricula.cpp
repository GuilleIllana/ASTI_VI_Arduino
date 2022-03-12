#include "cuadricula.h"
#include <QTRSensors.h>
Cuadricula::Cuadricula(int rows, int cols, int row_init, int col_init, int obs_row[], int obs_col[], int nobs) {
    _rows = rows;
    _cols = cols;
    _posr = row_init;
    _posc = col_init;
    
    Tablero = (Casilla**)malloc(rows * sizeof(Casilla*));
    for (int i = 0; i < rows; i++)
        Tablero[i] = (Casilla*)malloc(cols * sizeof(Casilla));
        
    // Calling constructor
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
          Tablero[i][j] = Casilla(i,j,false,0);

    // Adición de obstáculos
    for (int i = 0; i < nobs; i++)
      Tablero[obs_row[i]][obs_col[i]].setOccup(true);
}
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int Cuadricula::minDistance(int dist[], bool sptSet[])  {
    // Initialize min value
    int min = 99, min_index;
 
    for (int v = 0; v < 4; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
 
    return min_index;
}

int* Cuadricula::Planner(int ro, int co, int rf, int cf) {
  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation 
 
  // Initialize all distances and stpSet[] as false
  for (int i = 0; i < _rows; i++)
    for (int j = 0; j < _cols; j++) {
      Tablero[i][j]->setDist(sqrt(sq(rf-i)+sq(cf-j)));
      Tablero[i][j]->setExpl(false);
    }
    
  // Find shortest path for all vertices
  for (int count = 0; count < V - 1; count++) {
    // Pick the minimum distance vertex from the set of vertices not
    // yet processed. u is always equal to src in the first iteration.
    int u = minDistance(dist, sptSet);
 
    // Mark the picked vertex as processed
    sptSet[u] = true;
 
        // Update dist value of the adjacent vertices of the picked vertex.
    for (int v = 0; v < 4; v++)
 
      // Update dist[v] only if is not in sptSet, there is an edge from
      // u to v, and total weight of path from src to  v through u is
      // smaller than current value of dist[v]
      if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v])
        dist[v] = dist[u] + graph[u][v];
      }
 
    // print the constructed distance array
    printSolution(dist, V);
}


  
