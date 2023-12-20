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

bool fez_caminho_uma_vez = false;
int i_inicial = 0;
int j_inicial = 0;
int i_saida = GRID_WIDTH - 1;
int j_saida = GRID_HEIGHT - 1;
int i_atual = i_inicial;
int j_atual = j_inicial;
Stack<GRAPH_SIZE> menor_caminho;
int parents[GRAPH_SIZE] = {0};
int orientacao_atual = norte;
int graph[GRAPH_SIZE][GRAPH_SIZE];
MPU6050 mpu(Wire);
// Objeto usado para controlador o giroscopio

// Objeto usado para controlar o servo motor
Servo sg90;
// Constantes usadas para fazer o controle PWM de velocidade dos motores
bool slow = false;
unsigned long ultima_curva = 0;

void mede_distancias(double distancias[]);

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

  inicializa_grafo(graph);
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

  if (inputL > 2000 && inputR > 2000 && inputC > 2000)
  {
    slow = true;
  }

  if ((inputSLC > 2000 && inputSRC > 2000) && (millis() - ultima_curva > 1000))
  {
    slow = false;

  if (index(i_atual, j_atual) == index(i_saida, j_saida))
  {
    Serial.printf("parou\n");

    ledcWrite(PWM1_Ch, 127);
    ledcWrite(PWM2_Ch, 127);
    
    fez_caminho_uma_vez = true;
    i_atual = i_inicial;
    j_atual = j_inicial;
    orientacao_atual = norte;

    dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida)); 

    delay(15000);
  }
  else {
    processa_cruzamento();
  }
  }
  else
  {
    segue_linha(inputL, inputR, slow);
  }
}

/**
 * @brief toma decisoes ao encontrar cruzamento
 * @return void
 */
void processa_cruzamento()
{
  double distancias[3] = {0};

  mede_distancias(distancias); 

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

  if (caminho_alterado && !fez_caminho_uma_vez)
  {
    dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));
    caminho_alterado = false;
  }

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

void mede_distancias(double distancias[]) {
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