#include <MPU6050_light.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define DISTANCIA_OBSTACULO 22
#define QTD_MEDIDAS_ULTRASOM 3

#define SOUND_SPEED 0.034

// Canais de PWM dos motores
#define PWM1_Ch    2
#define PWM2_Ch    3
#define PWM1_Res   8
#define PWM1_Freq  1000

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status != 0) {} // para tudo se nao conseguiu se conectar ao MPU6050

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
  sg90.setPeriodHertz(50); // frequencia de PWM para SG90
  sg90.attach(PIN_SG90, 500, 2400); // Minimo and maximo comprimento do pulso (em µs) para ir de 0° a 180°

  // Ultrasom
  pinMode(EN_TRIG, OUTPUT);
  pinMode(MISO_ECHO, INPUT);

  // Coloca o servo apontando para frente antes de comecar a mover o robo
  sg90.write(80);
}

void loop() {
  digitalWrite(CS_Sensors, HIGH);
  
  digitalWrite(E_CH1, HIGH);
  digitalWrite(E_CH2, HIGH);
  
  int inputL = analogRead(SL);
  int inputR = analogRead(SR);
  int inputC = analogRead(SC);
  int inputSLC = analogRead(SLC);
  int inputSRC = analogRead(SRC);

  if(inputL > 2000 && inputR > 2000 && inputC > 2000) {
    slow = true;
  }

  if((inputSLC > 2000 && inputSRC > 2000) && (millis() - ultima_curva > 1000)) {
    slow = false;
    processa_cruzamento();
  }
  else {
    int deltaL = (int) (((float) inputL/4000.0)*DELTA_VEL);
    int deltaR = (int) (((float) inputR/4000.0)*DELTA_VEL);

    int pwmL = 0;
    int pwmR = 0;

    if(slow) {
      pwmL = SLOW_VEL - deltaL + deltaR;
      pwmR = SLOW_VEL - deltaR + deltaL;
    }
    else {
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
void processa_cruzamento() {
  float distancias[3] = { 0 };
  
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

  // O robo vira para esquerda, frente e direita, na seguinte prioridade
  // 1: esquerda
  // 2: frente
  // 3: direita
  // 4: vira para tras (180º)
  if(distancias[0] > DISTANCIA_OBSTACULO) {
    vira_esquerda();
  }
  else if(distancias[1] > DISTANCIA_OBSTACULO) {
    anda_reto();
  }
  else if(distancias[2] > DISTANCIA_OBSTACULO) {
    vira_direita();
  }
  else {
    vira_180();
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
void anda_reto() {
  return;
}

/**
 * @brief vira o robo para direita
 * @return void 
 */
void vira_direita() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em -90º
  while(delta > -90) {
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
void vira_esquerda() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 90º
  while(delta < 90) {
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
void vira_180() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 180º
  while(delta < 180) {
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
float le_distancia() {
  long durations[QTD_MEDIDAS_ULTRASOM] = { 0 };

  // Le o tempo de percurso da onda 3 vezes  
  for(int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++) {
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
  for(int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++) {
    soma += durations[i];
  }

  double media = (double) soma / (double) QTD_MEDIDAS_ULTRASOM;

  // Calcula a distancia em cm a partir do tempo
  float distanceCm = media * SOUND_SPEED/2.0;

  return distanceCm;
}