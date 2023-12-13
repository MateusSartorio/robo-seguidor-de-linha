#include <MPU6050_light.h>
#include <Wire.h>
#include <ESP32Servo.h>

// Frente: 80, esquerda: 180, direita: 0
// Pino 13
// Ultrasom miso echo 19, entrigger 16
// dividir o que achar por 58
// problema no pwm do servo

#define SOUND_SPEED 0.034

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
#define PIN_SG90 13 // Output pin used

// Ultrasom
#define MISO_ECHO 19
#define EN_TRIG 16

const int BASE_VEL = 170;
const int SLOW_VEL = 150;
const int DELTA_VEL = 20;

bool slow = false;

MPU6050 mpu(Wire);
unsigned long ultima_curva = 0;

Servo sg90;

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
  while(status!=0){ } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

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

  // Motores
  pinMode(E_CH1, OUTPUT);
  pinMode(E_CH2, OUTPUT);
  
  ledcAttachPin(CHA_M1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  ledcAttachPin(CHA_M2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);

  sg90.setPeriodHertz(50); // PWM frequency for SG90
  sg90.attach(PIN_SG90, 500, 2400); // Minimum and maximum pulse width (in µs) to go from 0° to 180

  // Ultrasom
  pinMode(EN_TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(MISO_ECHO, INPUT); // Sets the echoPin as an Input

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

    Serial.printf("L: %d, pwmL: %d, R: %d, pwmR: %d, slc: %d\n", inputL, pwmL, inputR, pwmR, inputSLC);

    // delay(1000);
    
    // direito
    ledcWrite(PWM1_Ch, pwmR);
    // ledcWrite(PWM1_Ch, 127);

    // esquerdo (invertido)
    ledcWrite(PWM2_Ch, 255 - pwmL);
    // ledcWrite(PWM2_Ch, 127);
  }
}

void processa_cruzamento() {
  float distancias[3] = { 0 };
  
  // esquerdo (invertido)
  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  // esquerda
  sg90.write(180);
  delay(1000);
  distancias[0] = le_distancia();
    ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);
  // meio
  sg90.write(80);
  delay(1000);
  distancias[1] = le_distancia();

  // direita
  sg90.write(0);
  delay(1000);
  distancias[2] = le_distancia();

  // Serial.printf("e: %f, c: %f, d: %f\n", distancias[0], distancias[1], distancias[2]);

  if(distancias[0] > 20) {
    vira_esquerda();
  }
  else if(distancias[1] > 20) {
    anda_reto();
  }
  else if(distancias[2] > 20){
    vira_direita();
  }
  else {
    vira_180();
  }

  sg90.write(80);
  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);
  delay(1000);
  ultima_curva = millis();
}

void anda_reto() {
  return;
}

void vira_direita() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  while(delta > -90) {
    mpu.update();
    angulo_atual = mpu.getAngleZ();
    // Serial.printf("init: %d, atual: %d, delta: %d\n", angulo_inicial, angulo_atual, angulo_atual - angulo_inicial);
    
    delta = angulo_atual - angulo_inicial;
    
    ledcWrite(PWM1_Ch, 255 - BASE_VEL);
    ledcWrite(PWM2_Ch, 255 - BASE_VEL);
  }
}

void vira_esquerda() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  while(delta < 90) {
    mpu.update();
    angulo_atual = mpu.getAngleZ();
    // Serial.printf("init: %d, atual: %d, delta: %d\n", angulo_inicial, angulo_atual, angulo_atual - angulo_inicial);
    
    delta = angulo_atual - angulo_inicial;
    
    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

void vira_180() {
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  int delta = angulo_atual - angulo_inicial;

  while(delta < 180) {
    mpu.update();
    angulo_atual = mpu.getAngleZ();
    // Serial.printf("init: %d, atual: %d, delta: %d\n", angulo_inicial, angulo_atual, angulo_atual - angulo_inicial);
    
    delta = angulo_atual - angulo_inicial;
    
    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

float le_distancia() {
  long durations[3] = { 0 };
  
  for(int i = 0; i < 3; i++) {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(2);
    // Sets the EN_TRIG on HIGH state for 10 micro seconds
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durations[i] = pulseIn(MISO_ECHO, HIGH);
  }

  long media = durations[0] + durations[1] + durations[2];
  media = media / 3;

  float distanceCm = media * SOUND_SPEED/2.0;

  return distanceCm;
}