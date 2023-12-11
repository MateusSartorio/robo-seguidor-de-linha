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

const int BASE_VEL = 200;
const int DELTA_VEL = 30;

void setup() {
  Serial.begin(115200);

  pinMode(CS_Sensors, OUTPUT);
  pinMode(SR, INPUT);
  pinMode(SL, INPUT);
  pinMode(SLC, INPUT);
  
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

  int deltaL = (int) (((float) inputL/4095.0)*30);
  int deltaR = (int) (((float) inputR/4095.0)*30);

  int pwmL = BASE_VEL - deltaL + deltaR;
  int pwmR = BASE_VEL - deltaR + deltaL;

  Serial.printf("L: %d, pwmL: %d, R: %d, pwmR: %d, slc: %d\n", inputL, pwmL, inputR, pwmR, inputSLC);

  // delay(1000);
  
  // esquerdo (invertido)
  // ledcWrite(PWM1_Ch, 255 - pwmL);
  ledcWRite(PWM1_Ch, 127);

  // direito
  // ledcWrite(PWM2_Ch, pwmR);
  ledcWrite(PWM2_Ch, 127);
}