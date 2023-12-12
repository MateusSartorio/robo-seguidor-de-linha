#include <MPU6050_light.h>
#include <Wire.h>

#define PWM1_Ch    0
#define PWM2_Ch    1
#define PWM1_Res   8
#define PWM1_Freq  1000

#define CS_Sensors 5

#define SR 39
#define SL 35
#define SLC 36

#define E_CH1 33
#define CHA_M1 25
#define E_CH2 26
#define CHA_M2 27

#define SDA 21
#define SCL 22

const int BASE_VEL = 200;
const int DELTA_VEL = 30;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void vira_robo();

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
}

void loop() {
  digitalWrite(CS_Sensors, HIGH);
  digitalWrite(E_CH1, HIGH);
  digitalWrite(E_CH2, HIGH);
  int inputL = analogRead(SL);
  int inputR = analogRead(SR);
  int inputSLC = analogRead(SLC);
  
  if(inputSLC > 2000) {
    vira_robo();
  }
  else {
    int deltaL = (int) (((float) inputL/4095.0)*30);
    int deltaR = (int) (((float) inputR/4095.0)*30);

    int pwmL = BASE_VEL - deltaL + deltaR;
    int pwmR = BASE_VEL - deltaR + deltaL;

    Serial.printf("L: %d, pwmL: %d, R: %d, pwmR: %d, slc: %d\n", inputL, pwmL, inputR, pwmR, inputSLC);

    // delay(1000);
    
    // esquerdo (invertido)
    ledcWrite(PWM1_Ch, 255 - pwmL);

    // direito
    ledcWrite(PWM2_Ch, pwmR);
  }
}

void vira_robo() {
  // esquerdo (invertido)
  delay(200);
  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);
  delay(1000);
  
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;
  
  while(angulo_atual - angulo_inicial < 90) {
    mpu.update();
    angulo_atual = mpu.getAngleZ();
    // Serial.printf("init: %d, atual: %d, delta: %d\n", angulo_inicial, angulo_atual, angulo_atual - angulo_inicial);
    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }

  ledcWrite(PWM1_Ch, 127);
  ledcWrite(PWM2_Ch, 127);

  delay(1000);
}