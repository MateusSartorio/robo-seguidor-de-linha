#include <MPU6050_light.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Stack.h"
#include "dijkstra.h"
#include "orientacao.h"
#include "direcao.h"

#define GRID_WIDTH 3
#define GRID_HEIGHT 5
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

bool fez_caminho_uma_vez = false;
int i_inicial = 0;
int j_inicial = 0;
int i_saida = GRID_WIDTH - 1;
int j_saida = GRID_HEIGHT - 1;
int i_atual = i_inicial;
int j_atual = j_inicial;
const int GRAPH_SIZE = GRID_WIDTH * GRID_HEIGHT;
Stack<GRAPH_SIZE> menor_caminho;
int parents[GRAPH_SIZE] = {0};
int orientacao_atual = norte;
int graph[GRAPH_SIZE][GRAPH_SIZE];
MPU6050 mpu(Wire);
// Objeto usado para controlador o giroscopio

// Objeto usado para controlar o servo motor
Servo sg90;
// Constantes usadas para fazer o controle PWM de velocidade dos motores
const int BASE_VEL = 200;
const int SLOW_VEL = 180;
const int DELTA_VEL = 40;
bool slow = false;
unsigned long ultima_curva = 0;

int get_i(int n);
int get_j(int n);
int index(int i, int j);
int obtem_nova_orientacao(int d);
int obtem_vertice_a_esquerda();
int obtem_vertice_em_frente();
int obtem_vertice_a_direita();
int obtem_direcao_de_curva(int proximo_vertice_a_andar);
// Quandos os sensores IR da frente encontram um cruzamento, o robo entra no modo slow, ate que os sensores de cruzamento encontrem a linha
// Armazena o valor de tempo dede a ultima curva feita pelo robo
// Apos o robo fazer uma curva em um cruzamento, ele conta 1000 ms ate que possa detectar outro cruzamento
// Isso evita que o robo continue detectando que encontrou um cruzamento apos fazer uma curva
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
    int deltaL = (int)(((double)inputL / 4000.0) * DELTA_VEL);
    int deltaR = (int)(((double)inputR / 4000.0) * DELTA_VEL);

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
  double distancias[3] = {0};

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

  Serial.printf("\ne: %lf, f: %lf, d: %lf\n", distancias[0], distancias[1], distancias[2]);

  bool caminho_alterado = false;
  if (distancias[0] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_esquerda = obtem_vertice_a_esquerda();
    if (vertice_a_esquerda != -1)
    {
      graph[index(i_atual, j_atual)][vertice_a_esquerda] = 0;
      graph[vertice_a_esquerda][index(i_atual, j_atual)] = 0;
      Serial.printf("Encontrou vertice %d a esquerda, recalculando caminho\n", vertice_a_esquerda);
    }

  }

  if (distancias[1] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_em_frente = obtem_vertice_em_frente();
    if (vertice_em_frente != -1)
    {
      graph[index(i_atual, j_atual)][vertice_em_frente] = 0;
      graph[vertice_em_frente][index(i_atual, j_atual)] = 0;
      Serial.printf("Encontrou vertice %d em frente, recalculando caminho\n", vertice_em_frente);
    }
  }

  if (distancias[2] < DISTANCIA_OBSTACULO)
  {
    caminho_alterado = true;
    int vertice_a_direita = obtem_vertice_a_direita();
    if (vertice_a_direita != -1)
    {
      graph[index(i_atual, j_atual)][vertice_a_direita] = 0;
      graph[vertice_a_direita][index(i_atual, j_atual)] = 0;
      Serial.printf("Encontrou vertice %d a direita, recalculando caminho\n", vertice_a_direita);
    }
  }

  if (caminho_alterado && !fez_caminho_uma_vez)
  {
    dijkstra(graph, parents, menor_caminho, index(i_atual, j_atual), index(i_saida, j_saida));
    caminho_alterado = false;
  }

  Serial.printf("Orientacao atual: ");
  switch(orientacao_atual) {
    case norte:
      Serial.printf("norte");
      break;
    case sul:
      Serial.printf("sul");
      break;
    case leste:
      Serial.printf("leste");
      break;
    case oeste:
      Serial.printf("oeste");
      break;
    default:
      Serial.printf(", Orientacao invalida");
  }

  Serial.printf("vertice atual: %d\n", index(i_atual, j_atual));

  int proximo_vertice_a_andar = menor_caminho.topElement();
  Serial.printf("Desempilhou: %d\n", proximo_vertice_a_andar);
  menor_caminho.pop();

  int direcao_da_curva = obtem_direcao_de_curva(proximo_vertice_a_andar);

  switch (direcao_da_curva)
  {
  case esquerda:
    Serial.printf("Andou para a esquerda\n");
    vira_esquerda();
    break;
  case frente:
    Serial.printf("Andou frente\n");
    anda_reto();
    break;
  case direita:
    Serial.printf("Andou para a direita\n");
    vira_direita();
    break;
  case tras:
    Serial.printf("Andou para tras\n");
    vira_180();
    break;
  default:
    Serial.printf("Problema em algum switch\n");
  }

  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  i_atual = get_i(proximo_vertice_a_andar);
  j_atual = get_j(proximo_vertice_a_andar);

  orientacao_atual = obtem_nova_orientacao(direcao_da_curva);
  
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
 * @return double
 * @retval distancia em cm
 */
double le_distancia()
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
  double distanceCm = media * SOUND_SPEED / 2.0;

  return distanceCm;
}




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

int obtem_nova_orientacao(int d)
{
  switch (orientacao_atual)
  {
  case norte:
    switch (d)
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
      Serial.printf("deu bosta norte\n");
      return -1;
    }
  case sul:
    switch (d)
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
      Serial.printf("deu bosta sul\n");
      return -1;
    }
  case leste:
    switch (d)
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
      Serial.printf("deu bosta oeste\n");
      return -1;
    }
  case oeste:
    switch (d)
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
      Serial.printf("deu bosta default\n");
      return -1;
    }
  default:
    Serial.printf("deu bosta\n");
    return -1;
  }
}

int obtem_vertice_a_esquerda()
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
    Serial.printf("Problema em algum switch\n");
    return -1;
  }
}

int obtem_vertice_em_frente()
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
    Serial.printf("Problema em algum switch\n");
    return -1;
  }
}

int obtem_vertice_a_direita()
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
    Serial.printf("Problema em algum switch\n");
    return -1;
  }
}

int obtem_direcao_de_curva(int proximo_vertice_a_andar)
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
    return -1;
  }
}
