#pragma once

#include "Stack.h"

#define NO_PARENT -1

// Function to print shortest path
// from source to currentVertex
// using parents array
int next_vertex(int currentVertex, int parents[])
{
  // Base case : Source node has
  // been processed
  if (currentVertex == NO_PARENT)
  {
    return -1;
  }

  return parents[currentVertex];
}

template <int GRAPH_SIZE>
void dijkstra(int adjacencyMatrix[GRAPH_SIZE][GRAPH_SIZE], int parents[], Stack<GRAPH_SIZE> &menor_caminho, int startVertex, int destinationVertex)
{
  menor_caminho.clear();

  int nVertices = GRAPH_SIZE;

  // shortestDistances[i] will hold the
  // shortest distance from src to i
  int shortestDistances[nVertices];

  // added[i] will true if vertex i is
  // included / in shortest path tree
  // or shortest distance from src to
  // i is finalized
  bool added[nVertices];

  // Initialize all distances as
  // INFINITE and added[] as false
  for (int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
  {
    shortestDistances[vertexIndex] = INT_MAX;
    added[vertexIndex] = false;
  }

  // Distance of source vertex from
  // itself is always 0
  shortestDistances[startVertex] = 0;

  // The starting vertex does not
  // have a parent
  parents[startVertex] = NO_PARENT;

  // Find shortest path for all
  // vertices
  for (int i = 1; i < nVertices; i++)
  {

    // Pick the minimum distance vertex
    // from the set of vertices not yet
    // processed. nearestVertex is
    // always equal to startNode in
    // first iteration.
    int nearestVertex = -1;
    int shortestDistance = INT_MAX;
    for (int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
    {
      if (!added[vertexIndex] && shortestDistances[vertexIndex] < shortestDistance)
      {
        nearestVertex = vertexIndex;
        shortestDistance = shortestDistances[vertexIndex];
      }
    }

    // Mark the picked vertex as
    // processed
    added[nearestVertex] = true;

    // Update dist value of the
    // adjacent vertices of the
    // picked vertex.
    for (int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
    {
      int edgeDistance = adjacencyMatrix[nearestVertex][vertexIndex];

      if (edgeDistance > 0 && ((shortestDistance + edgeDistance) < shortestDistances[vertexIndex]) && (shortestDistance + edgeDistance >= 0))
      {
        parents[vertexIndex] = nearestVertex;
        shortestDistances[vertexIndex] = shortestDistance + edgeDistance;
      }
    }
  }

  int proximo_vertice = destinationVertex;
  while (proximo_vertice != startVertex)
  {
    menor_caminho.push(proximo_vertice);
    proximo_vertice = parents[proximo_vertice];
  }
}