#include <MPU6050_light.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <iostream>
#include <string>
#include <stdio.h>

#define INT_MAX 2147483647

#define GRID_WIDTH 3
#define GRID_HEIGHT 3

int i_atual = 0;
int j_atual = 0;
int i_saida = GRID_WIDTH - 1;
int j_saida = GRID_HEIGHT - 1;
const int GRAPH_SIZE = GRID_WIDTH * GRID_HEIGHT;

Stack menor_caminho;

int NO_PARENT = -1;

const int STACK_MAX_SIZE = GRAPH_SIZE; // Maximum size of the stack

class Stack
{
private:
  int arr[STACK_MAX_SIZE];
  int top;

public:
  Stack()
  {
    top = -1;
  } // Initialize top to -1 to indicate an empty stack

  bool isEmpty()
  {
    return (top == -1);
  }

  bool isFull()
  {
    return (top == STACK_MAX_SIZE - 1);
  }

  void push(int element)
  {
    if (!isFull())
    {
      top++;
      arr[top] = element;
    }
    else
    {
      Serial.printf("Stack is full. Cannot push element %d\n", element);
    }
  }

  void pop()
  {
    if (!isEmpty())
    {
      int poppedElement = arr[top];
      top--;
    }
    else
    {
      Serial.printf("Stack is empty. Cannot pop from an empty stack.\n");
    }
  }

  int topElement()
  {
    if (!isEmpty())
    {
      return arr[top];
    }
    else
    {
      Serial.printf("Stack is empty.\n");
      return -1; // In this example, we consider -1 as an invalid value.
    }
  }

  void clear()
  {
    while (!isEmpty)
    {
      pop();
    }
  }
};

int parents[GRAPH_SIZE] = {0};

// Function to print shortest path
// from source to currentVertex
// using parents array
int next_vertex(int currentVertex)
{

  // Base case : Source node has
  // been processed
  if (currentVertex == NO_PARENT)
  {
    return -1;
  }

  return parents[currentVertex];
}

void dijkstra(int adjacencyMatrix[GRAPH_SIZE][GRAPH_SIZE], Stack &menor_caminho, int startVertex, int destinationVertex)
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

  int proximo_vertice = index(i_saida, j_saida);
  while (proximo_vertice != index(i_atual, j_atual))
  {
    menor_caminho.push(proximo_vertice);
    proximo_vertice = parents[proximo_vertice];
  }
}

enum orientacao
{
  norte,
  sul,
  leste,
  oeste
};

enum direcao
{
  esquerda,
  frente,
  direita,
  tras
};

orientacao orientacao_atual = norte;

int graph[GRAPH_SIZE][GRAPH_SIZE];

int get_i(int n)
{
  return n % GRID_WIDTH;
}

int get_j(int n)
{
  return n / GRID_WIDTH;
}

int index(int i, int j)
{
  return j * GRID_WIDTH + i;
}

int novo_i(orientacao o, direcao d, int i)
{
  switch (o)
  {
  case norte:
    switch (d)
    {
    case esquerda:
      return i - 1;
    case frente:
      return i;
    case direita:
      return i + 1;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case sul:
    switch (d)
    {
    case esquerda:
      return i + 1;
    case frente:
      return i;
    case direita:
      return i - 1;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case leste:
    switch (d)
    {
    case esquerda:
      return i;
    case frente:
      return i + 1;
    case direita:
      return i;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case oeste:
    switch (d)
    {
    case esquerda:
      return i;
    case frente:
      return i - 1;
    case direita:
      return i;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  default:
    Serial.printf("Problema em algum switch\n");
  }
}

int novo_j(orientacao o, direcao d, int j)
{
  switch (o)
  {
  case norte:
    switch (d)
    {
    case esquerda:
      return j;
    case frente:
      return j + 1;
    case direita:
      return j;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case sul:
    switch (d)
    {
    case esquerda:
      return j;
    case frente:
      return j - 1;
    case direita:
      return j;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case leste:
    switch (d)
    {
    case esquerda:
      return j + 1;
    case frente:
      return j;
    case direita:
      return j - 1;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  case oeste:
    switch (d)
    {
    case esquerda:
      return j - 1;
    case frente:
      return j;
    case direita:
      return j + 1;
    default:
      Serial.printf("Problema em algum switch\n");
    }
  default:
    Serial.printf("Problema em algum switch\n");
  }
}

int obtem_vertice_a_esquerda()
{
  switch (orientacao_atual)
  {
  case norte:
    if (i_atual > 0)
    {
      return index(i_atual - 1, j);
    }
    else
    {
      return -1;
    }
  case sul:
    if (i_atual < GRID_WIDTH - 1)
    {
      return index(i_atual + 1, j);
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
    Serial.printf("Problema em algum switch\n");
  }
}

int obtem_vertice_em_frente()
{
  switch (orientacao_atual)
  {
  case norte:
    if (j_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual, j + 1);
    }
    else
    {
      return -1;
    }
  case sul:
    if (j_atual > 0)
    {
      return index(i_atual, j - 1);
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
    Serial.printf("Problema em algum switch\n");
  }
}

int obtem_vertice_a_direita()
{
  switch (orientacao_atual)
  {
  case norte:
    if (i_atual < GRID_HEIGHT - 1)
    {
      return index(i_atual + 1, j);
    }
    else
    {
      return -1;
    }
  case sul:
    if (i_atual > 0)
    {
      return index(i_atual - 1, j);
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
    Serial.printf("Problema em algum switch\n");
  }
}

direcao obtem_direcao_de_curva(int proximo_vertice_a_andar)
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
    Serial.printf("Problema em algum switch\n");
  }
}

#define DISTANCIA_OBSTACULO 22
#define QTD_MEDIDAS_ULTRASOM 3

#define SOUND_SPEED 0.034

// Canais de PWM dos motores
#define PWM1_Ch 2
#define PWM2_Ch 3
#define PWM1_Res 8
#define PWM1_Freq 1000

#define CS_Sensors 5

#define SR 39
#define SL 35
#define SLC 36
#define SRC 32
#define SC 34

#define E_CH1 33
#define CHA_M1 25
#define E_CH2 26
#define CHA_M2 27

// Giroscopio
#define SDA 21
#define SCL 22

// Servo
#define PIN_SG90 13

// Ultrasom
#define MISO_ECHO 19
#define EN_TRIG 16

// Objeto usado para controlador o giroscopio
MPU6050 mpu(Wire);

// Objeto usado para controlar o servo motor
Servo sg90;
// Constantes usadas para fazer o controle PWM de velocidade dos motores
const int BASE_VEL = 170;
const int SLOW_VEL = 160;
const int DELTA_VEL = 20;

// Quandos os sensores IR da frente encontram um cruzamento, o robo entra no modo slow, ate que os sensores de cruzamento encontrem a linha
bool slow = false;

// Armazena o valor de tempo dede a ultima curva feita pelo robo
// Apos o robo fazer uma curva em um cruzamento, ele conta 1000 ms ate que possa detectar outro cruzamento
// Isso evita que o robo continue detectando que encontrou um cruzamento apos fazer uma curva
unsigned long ultima_curva = 0;

void processa_cruzamento();
void anda_reto();
void vira_esquerda();
void vira_direita();
void vira_180();

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // para tudo se nao conseguiu se conectar ao MPU6050

  Serial.println(F("Calculando offsets, nao mova o MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // descomente essa linha se o MPU6050 esta montado de cabeca para baixo
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Pronto!\n");

  // Sensores IR
  pinMode(CS_Sensors, OUTPUT);
  pinMode(SR, INPUT);
  pinMode(SL, INPUT);
  pinMode(SLC, INPUT);
  pinMode(SLC, INPUT);
  pinMode(SC, INPUT);

  // Giroscopio
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);

  // Motores da roda
  pinMode(E_CH1, OUTPUT);
  pinMode(E_CH2, OUTPUT);

  ledcAttachPin(CHA_M1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  ledcAttachPin(CHA_M2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);

  // Servo motor
  sg90.setPeriodHertz(50);          // frequencia de PWM para SG90
  sg90.attach(PIN_SG90, 500, 2400); // Minimo and maximo comprimento do pulso (em µs) para ir de 0° a 180°

  // Ultrasom
  pinMode(EN_TRIG, OUTPUT);
  pinMode(MISO_ECHO, INPUT);

  // Coloca o servo apontando para frente antes de comecar a mover o robo
  sg90.write(80);

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

void loop()
{
  digitalWrite(CS_Sensors, HIGH);

  digitalWrite(E_CH1, HIGH);
  digitalWrite(E_CH2, HIGH);

  int inputL = analogRead(SL);
  int inputR = analogRead(SR);
  int inputC = analogRead(SC);
  int inputSLC = analogRead(SLC);
  int inputSRC = analogRead(SRC);

  if (inputL > 2000 && inputR > 2000 && inputC > 2000)
  {
    slow = true;
  }

  if ((inputSLC > 2000 && inputSRC > 2000) && (millis() - ultima_curva > 1000))
  {
    slow = false;
    processa_cruzamento();
  }
  else
  {
    int deltaL = (int)(((float)inputL / 4000.0) * DELTA_VEL);
    int deltaR = (int)(((float)inputR / 4000.0) * DELTA_VEL);

    int pwmL = 0;
    int pwmR = 0;

    if (slow)
    {
      pwmL = SLOW_VEL - deltaL + deltaR;
      pwmR = SLOW_VEL - deltaR + deltaL;
    }
    else
    {
      pwmL = BASE_VEL - deltaL + deltaR;
      pwmR = BASE_VEL - deltaR + deltaL;
    }

    // motor direito
    ledcWrite(PWM1_Ch, pwmR);

    // motor esquerdo (invertido)
    ledcWrite(PWM2_Ch, 255 - pwmL);
  }
}

/**
 * @brief toma decisoes ao encontrar cruzamento
 * @return void
 */
void processa_cruzamento()
{
  float distancias[3] = {0};

  // motor esquerdo (invertido)
  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  // aponta sensor ultrasom para a esquerda
  sg90.write(180);
  delay(1000);
  distancias[0] = le_distancia();
  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  // aponta sensor ultrasom para o meio
  sg90.write(80);
  delay(1000);
  distancias[1] = le_distancia();

  // aponta sensor ultrasom para a direita
  sg90.write(0);
  delay(1000);
  distancias[2] = le_distancia();

  // Apos leitura, ajudar o servo motor para frente novamente
  sg90.write(80);
  delay(100);

  bool caminho_alterado = false;
  if (distancias[0] > DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_esquerda = obtem_vertice_a_esquerda;
    if (vertice_a_esquerda != -1)
    {
      graph[index(i_atual, j_atual), vertice_a_esquerda] = 0;
      graph[vertice_a_esquerda, index(i_atual, j_atual)] = 0;
    }
  }

  if (distancias[1] > DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_em_frente = obtem_vertice_em_frente;
    if (vertice_em_frente != -1)
    {
      graph[index(i_atual, j_atual), vertice_em_frente] = 0;
      graph[vertice_em_frente, index(i_atual, j_atual)] = 0;
    }
  }

  if (distancias[2] > DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_direita = obtem_vertice_a_direita;
    if (vertice_a_direita != -1)
    {
      graph[index(i_atual, j_atual), vertice_a_direita] = 0;
      graph[vertice_a_direita, index(i_atual, j_atual)] = 0;
    }
  }

  if (caminho_alterado)
  {
    dijkstra(graph, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));
  }

  int proximo_vertice_a_andar = menor_caminho.topElement();
  menor_caminho.pop();

  direcao direcao_da_curva = obtem_direcao_de_curva(proximo_vertice_a_andar);

  switch (direcao_da_curva)
  {
  case esquerda:
    vira_esquerda();
    break;
  case frente:
    anda_reto();
    break;
  case direita:
    vira_direita();
    break;
  case tras:
    vira_180();
    break;
  default:
    Serial.printf("Problema em algum switch\n");
  }

  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);
  delay(1000);

  // Armazena o valor de tempo desde que terminou de fazer a curva
  ultima_curva = millis();
}

/**
 * @brief o robo anda reto no cruzamento
 * @return void
 */
void anda_reto()
{
  return;
}

/**
 * @brief vira o robo para direita
 * @return void
 */
void vira_direita()
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em -90º
  while (delta > -90)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, 255 - BASE_VEL);
    ledcWrite(PWM2_Ch, 255 - BASE_VEL);
  }
}

/**
 * @brief vira o robo para esquerda
 * @return void
 */
void vira_esquerda()
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 90º
  while (delta < 90)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

/**
 * @brief vira o robo em 180º
 * @return void
 */
void vira_180()
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 180º
  while (delta < 180)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

/**
 * @brief le a distancia de objetos em relacao ao sensor ultrasom
 * @return float
 * @retval distancia em cm
 */
float le_distancia()
{
  long durations[QTD_MEDIDAS_ULTRASOM] = {0};

  // Le o tempo de percurso da onda 3 vezes
  for (int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++)
  {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(2);
    // Seta o EN_TRIG em HIGH por 10 microssegundos
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, LOW);

    // Le o MISO_ECHO, returna o tempo de percurso em microssegundos
    durations[i] = pulseIn(MISO_ECHO, HIGH);
  }

  // Calcula a media dos tempos
  long soma = 0;
  for (int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++)
  {
    soma += durations[i];
  }

  double media = (double)soma / (double)QTD_MEDIDAS_ULTRASOM;

  // Calcula a distancia em cm a partir do tempo
  float distanceCm = media * SOUND_SPEED / 2.0;

  return distanceCm;
}
