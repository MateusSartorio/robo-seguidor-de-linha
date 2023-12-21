#include <MPU6050_light.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Stack.h"
#include "graph.h"
#include "orientacao.h"
#include "direcao.h"
#include "pinout.h"
#include "controle_motores.h"
#include "sensor_ultrassom.h"

// Grafo que representa o labirinto
int graph[GRAPH_SIZE][GRAPH_SIZE];

// Variaveis que indicam onde o robo esta no labirinto e com qual orientacao
int i_inicial = 0;
int j_inicial = 0;

int i_saida = GRID_WIDTH - 1;
int j_saida = GRID_HEIGHT - 1;

int i_atual = i_inicial;
int j_atual = j_inicial;

int orientacao_atual = norte;

// Armazena o caminho mínimo até a saída a partir da posição atual
Stack<GRAPH_SIZE> menor_caminho;

// Array auxiliar que armazena informações da árvore de caminho mínimo obtida pelo algorítimo de Dijkstra
int parents[GRAPH_SIZE] = {0};

// Após o robo ter feito o caminho uma vez, ele já conhece a rota ideal, e nao refaz calculos de rota
bool fez_caminho_uma_vez = false;

// Objeto usado para controlador o giroscopio
MPU6050 mpu(Wire);

// Objeto usado para controlar o servo motor
Servo sg90;

// Quandos os sensores IR da frente encontram um cruzamento, o robo entra no modo slow, ate que os sensores de cruzamento encontrem a linha
bool slow = false;

// Armazena o valor de tempo dede a ultima curva feita pelo robo
// Apos o robo fazer uma curva em um cruzamento, ele conta 1000 ms ate que possa detectar outro cruzamento
// Isso evita que o robo continue detectando que encontrou um cruzamento apos fazer uma curva
unsigned long ultima_curva = 0;

void mede_distancias(double distancias[]);
void processa_cruzamento();

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  // para tudo se nao conseguiu se conectar ao MPU6050
  while (status != 0)
  {
  }

  Serial.println(F("Calculando offsets, nao mova o MPU6050"));
  delay(1000);

  // Descomente essa linha se o MPU6050 esta montado de cabeca para baixo
  // mpu.upsideDownMounting = true;

  // calcula offsets do giroscopio e acelerometro
  mpu.calcOffsets();
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
  // frequencia de PWM para SG90
  sg90.setPeriodHertz(50);

  // Minimo and maximo comprimento do pulso (em µs) para ir de 0° a 180°
  sg90.attach(PIN_SG90, 500, 2400);

  // Ultrasom
  pinMode(EN_TRIG, OUTPUT);
  pinMode(MISO_ECHO, INPUT);

  // Coloca o servo apontando para frente antes de comecar a mover o robo
  sg90.write(80);

  // Inicializa todas as arestas do grafo, supondo que não existe nenhum obstáculo
  inicializa_grafo(graph);

  // Calcula o menor caminho ate a saida inicialmente
  dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));
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

  // Caso os sensores da frente detectem o cruzamento o robo entra no modo lento ("slow")
  if (inputL > 2000 && inputR > 2000 && inputC > 2000)
  {
    slow = true;
  }

  // Quando os sensores de cruzamento detectam o cruzamento, o robo checa se chegou ao final do percurso
  // Caso sim, ele fica parado por 20 segundos para que seja reposicionado no comeco do labirinto novamente
  // Caso nao, ele processa o cruzamento para descobrir para qual lado deve seguir
  if ((inputSLC > 2000 && inputSRC > 2000) && (millis() - ultima_curva > 1000))
  {
    slow = false;

    if (index(i_atual, j_atual) == index(i_saida, j_saida))
    {
      Serial.printf("Chegou ao final do percurso\n");

      ledcWrite(PWM1_Ch, 127);
      ledcWrite(PWM2_Ch, 127);

      fez_caminho_uma_vez = true;
      i_atual = i_inicial;
      j_atual = j_inicial;
      orientacao_atual = norte;

      dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));

      delay(20000);
    }
    else
    {
      processa_cruzamento();
    }
  }
  else
  {
    segue_linha(inputL, inputR, slow);
  }
}

/**
 * @brief Toma decisoes ao encontrar cruzamento
 */
void processa_cruzamento()
{
  double distancias[3] = {0};
  mede_distancias(distancias);

  // Caso algum objeto seja identificado em alguma direcao, a aresta equivalente eh retirada do grafo e o algoritimo de Dijkstra eh executado novamente
  bool caminho_alterado = false;
  if (distancias[0] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_esquerda = obtem_vertice_a_esquerda(orientacao_atual, i_atual, j_atual);
    if (vertice_a_esquerda != -1)
    {
      graph[index(i_atual, j_atual)][vertice_a_esquerda] = 0;
      graph[vertice_a_esquerda][index(i_atual, j_atual)] = 0;
    }
  }

  if (distancias[1] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_em_frente = obtem_vertice_em_frente(orientacao_atual, i_atual, j_atual);
    if (vertice_em_frente != -1)
    {
      graph[index(i_atual, j_atual)][vertice_em_frente] = 0;
      graph[vertice_em_frente][index(i_atual, j_atual)] = 0;
    }
  }

  if (distancias[2] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_direita = obtem_vertice_a_direita(orientacao_atual, i_atual, j_atual);
    if (vertice_a_direita != -1)
    {
      graph[index(i_atual, j_atual)][vertice_a_direita] = 0;
      graph[vertice_a_direita][index(i_atual, j_atual)] = 0;
    }
  }

  // O algoritimo de Dijkstra so eh executado na primeira vez que o robo esta resolvendo o labirinto
  if (caminho_alterado && !fez_caminho_uma_vez)
  {
    dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));
    caminho_alterado = false;
  }

  // Calcula-se para qual vertice o robo deve ir, e esta informacao eh traduzida para uma direcao para qual ele deve virar
  int proximo_vertice_a_andar = menor_caminho.topElement();
  menor_caminho.pop();

  int direcao_da_curva = obtem_direcao_de_curva(orientacao_atual, i_atual, j_atual, proximo_vertice_a_andar);

  switch (direcao_da_curva)
  {
  case esquerda:
    vira_esquerda(mpu);
    break;
  case frente:
    anda_reto(mpu);
    break;
  case direita:
    vira_direita(mpu);
    break;
  case tras:
    vira_180(mpu);
    break;
  }

  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  i_atual = get_i(proximo_vertice_a_andar);
  j_atual = get_j(proximo_vertice_a_andar);

  orientacao_atual = obtem_nova_orientacao(orientacao_atual, direcao_da_curva);

  delay(1000);

  // Armazena o valor de tempo desde que terminou de fazer a curva
  ultima_curva = millis();
}

/**
 * @brief Mede a distancias aos obstaculos da esquerda, frente e direita, nesta ordem
 */
void mede_distancias(double distancias[])
{
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
}