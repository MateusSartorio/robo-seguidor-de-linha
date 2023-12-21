#pragma once

#include "Stack.h"
#include "orientacao.h"
#include "direcao.h"

#define NO_PARENT -1

#define GRID_WIDTH 3
#define GRID_HEIGHT 5
const int GRAPH_SIZE = GRID_WIDTH * GRID_HEIGHT;

/**
 * @brief Retorna a cordenada i (horizontal) para um vertice de indice n
 *
 * @param n
 * @return int
 */
int get_i(int n)
{
  return n % GRID_WIDTH;
}

/**
 * @brief Retorna a cordenada j (vertical) para um vertice de indice n
 *
 * @param n
 * @return int
 */
int get_j(int n)
{
  return n / GRID_WIDTH;
}

/**
 * @brief Retorna o indice n de um objeto com cordenadas (i, j)
 *
 * @param i
 * @param j
 * @return int
 */
int index(int i, int j)
{
  return j * GRID_WIDTH + i;
}

/**
 * @brief Obtem nova orientacao do robo com base na orientacao atual e no sentido para o qual ele esta virando
 *
 * @param orientacao_atual
 * @param direcao_da_curva
 * @return int
 */
int obtem_nova_orientacao(int orientacao_atual, int direcao_da_curva)
{
  switch (orientacao_atual)
  {
  case norte:
    switch (direcao_da_curva)
    {
    case esquerda:
      return oeste;
    case frente:
      return norte;
    case direita:
      return leste;
    case tras:
      return sul;
    default:
      return -1;
    }
  case sul:
    switch (direcao_da_curva)
    {
    case esquerda:
      return leste;
    case frente:
      return sul;
    case direita:
      return oeste;
    case tras:
      return norte;
    default:
      return -1;
    }
  case leste:
    switch (direcao_da_curva)
    {
    case esquerda:
      return norte;
    case frente:
      return leste;
    case direita:
      return sul;
    case tras:
      return oeste;
    default:
      return -1;
    }
  case oeste:
    switch (direcao_da_curva)
    {
    case esquerda:
      return sul;
    case frente:
      return oeste;
    case direita:
      return norte;
    case tras:
      return leste;
    default:
      return -1;
    }
  default:
    return -1;
  }
}

/**
 * @brief Obtem o vertice a esquerda do robo com base na sua orientacao e posicao no grid (grafo)
 *
 * @param orientacao_atual
 * @param i_atual
 * @param j_atual
 * @return int
 */
int obtem_vertice_a_esquerda(int orientacao_atual, int i_atual, int j_atual)
{
  switch (orientacao_atual)
  {
  case norte:
    if (i_atual > 0)
    {
      return index(i_atual - 1, j_atual);
    }
    else
    {
      return -1;
    }
  case sul:
    if (i_atual < GRID_WIDTH - 1)
    {
      return index(i_atual + 1, j_atual);
    }
    else
    {
      return -1;
    }
  case leste:
    if (j_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual, j_atual + 1);
    }
    else
    {
      return -1;
    }
  case oeste:
    if (j_atual > 0)
    {
      return index(i_atual, j_atual - 1);
    }
    else
    {
      return -1;
    }
  default:
    return -1;
  }
}

/**
 * @brief Obtem o vertice em frente ao robo com base na sua orientacao e posicao no grid (grafo)
 *
 * @param orientacao_atual
 * @param i_atual
 * @param j_atual
 * @return int
 */
int obtem_vertice_em_frente(int orientacao_atual, int i_atual, int j_atual)
{
  switch (orientacao_atual)
  {
  case norte:
    if (j_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual, j_atual + 1);
    }
    else
    {
      return -1;
    }
  case sul:
    if (j_atual > 0)
    {
      return index(i_atual, j_atual - 1);
    }
    else
    {
      return -1;
    }
  case leste:
    if (i_atual < GRID_WIDTH - 1)
    {
      return index(i_atual + 1, j_atual);
    }
    else
    {
      return -1;
    }
  case oeste:
    if (i_atual > 0)
    {
      return index(i_atual - 1, j_atual);
    }
    else
    {
      return -1;
    }
  default:
    return -1;
  }
}

/**
 * @brief Obtem o vertice a direita do robo com base na sua orientacao e posicao no grid (grafo)
 *
 * @param orientacao_atual
 * @param i_atual
 * @param j_atual
 * @return int
 */
int obtem_vertice_a_direita(int orientacao_atual, int i_atual, int j_atual)
{
  switch (orientacao_atual)
  {
  case norte:
    if (i_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual + 1, j_atual);
    }
    else
    {
      return -1;
    }
  case sul:
    if (i_atual > 0)
    {
      return index(i_atual - 1, j_atual);
    }
    else
    {
      return -1;
    }
  case leste:
    if (j_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual, j_atual - 1);
    }
    else
    {
      return -1;
    }
  case oeste:
    if (j_atual > 0)
    {
      return index(i_atual, j_atual + 1);
    }
    else
    {
      return -1;
    }
  default:
    return -1;
  }
}

/**
 * @brief Obtem para qual direcao o robo tem que virar baseado na sua posicao no grid, na sua orientacao atual e para qual vertice ele quer ir
 *
 * @param orientacao_atual
 * @param i_atual
 * @param j_atual
 * @param proximo_vertice_a_andar
 * @return int
 */
int obtem_direcao_de_curva(int orientacao_atual, int i_atual, int j_atual, int proximo_vertice_a_andar)
{
  int i_proximo_vertice = get_i(proximo_vertice_a_andar);
  int j_proximo_vertice = get_j(proximo_vertice_a_andar);

  switch (orientacao_atual)
  {
  case norte:
    if (i_proximo_vertice > i_atual)
    {
      return direita;
    }
    else if (i_proximo_vertice < i_atual)
    {
      return esquerda;
    }
    else if (j_proximo_vertice > j_atual)
    {
      return frente;
    }
    else
    {
      return tras;
    }
  case sul:
    if (i_proximo_vertice > i_atual)
    {
      return esquerda;
    }
    else if (i_proximo_vertice < i_atual)
    {
      return direita;
    }
    else if (j_proximo_vertice > j_atual)
    {
      return tras;
    }
    else
    {
      return frente;
    }
  case leste:
    if (i_proximo_vertice > i_atual)
    {
      return frente;
    }
    else if (i_proximo_vertice < i_atual)
    {
      return tras;
    }
    else if (j_proximo_vertice > j_atual)
    {
      return esquerda;
    }
    else
    {
      return direita;
    }
  case oeste:
    if (i_proximo_vertice > i_atual)
    {
      return tras;
    }
    else if (i_proximo_vertice < i_atual)
    {
      return frente;
    }
    else if (j_proximo_vertice > j_atual)
    {
      return direita;
    }
    else
    {
      return esquerda;
    }
  default:
    return -1;
  }
}

/**
 * @brief Inicializa o grafo supondo que todos as os vertices adjacentes no grid estao conectados (supoem nao existem obstaculos)
 *
 * @param graph
 */
void inicializa_grafo(int graph[GRAPH_SIZE][GRAPH_SIZE])
{
  for (int k = 0; k < GRAPH_SIZE; k++)
  {
    int i = get_i(k);
    int j = get_j(k);

    if (i > 0)
    {
      graph[k][index(i - 1, j)] = 1;
    }
    else
    {
      graph[k][index(i - 1, j)] = 0;
    }

    if (i < GRID_WIDTH - 1)
    {
      graph[k][index(i + 1, j)] = 1;
    }
    else
    {
      graph[k][index(i + 1, j)] = 0;
    }

    if (j > 0)
    {
      graph[k][index(i, j - 1)] = 1;
    }
    else
    {
      graph[k][index(i, j - 1)] = 0;
    }

    if (j < GRID_HEIGHT - 1)
    {
      graph[k][index(i, j + 1)] = 1;
    }
    else
    {
      graph[k][index(i, j + 1)] = 0;
    }
  }
}

/**
 * @brief Retorna o proximo vertice na arvore de caminho minimo gerada pelo algoritimo de Dijkstra
 *
 * @param currentVertex
 * @param parents
 * @return int
 */
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

/**
 * @brief Algoritimo de Dijkstra. O menor caminho encontrado eh empilhado em menor_caminho
 *
 * @tparam GRAPH_SIZE
 * @param adjacencyMatrix
 * @param parents
 * @param menor_caminho
 * @param startVertex
 * @param destinationVertex
 */
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