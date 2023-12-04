#define MOTOR_ESQUERDO 10
#define MOTOR_DIREITO 11

int leitura_sensor0 = 0;
int leitura_sensor1 = 0;

void anda_reto(int velocidade)
{
  analogWrite(MOTOR_DIREITO, velocidade);
  analogWrite(MOTOR_ESQUERDO, velocidade);
}

void vira_esquerda(int velocidade)
{
  analogWrite(MOTOR_DIREITO, 0);
  analogWrite(MOTOR_ESQUERDO, velocidade);
}

void vira_direita(int velocidade)
{
  analogWrite(MOTOR_ESQUERDO, 0);
  analogWrite(MOTOR_DIREITO, velocidade);
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(MOTOR_ESQUERDO, OUTPUT);
  pinMode(MOTOR_DIREITO, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly

  leitura_sensor0 = analogRead(A0);
  leitura_sensor1 = analogRead(A1);

  Serial.print(leitura_sensor0);
  Serial.print(", ");
  Serial.println(leitura_sensor1);
  delay(200);
}

// #define CANAL_A 2
// #define CANAL_B 3

// #define OUTPUT_1 4
// #define OUTPUT_2 4
// #define PWM 6

// volatile int position = 0;
// unsigned long time = 0;
// int rpm = 0;

// // Nao sei porque, mas o valor de velocidade calculado
// // eh em torno de 16.8 vezes menor que o valor real de rotacao
// double proportionality_constant = 16.8;

// void A_interrupt()
// {
//   int b = digitalRead(CANAL_B);

//   if (b > 0)
//   {
//     position++;
//   }
//   else
//   {
//     position--;
//   }
// }

// void vira_esquerda()
// {
// }

// void setup()
// {
//   Serial.begin(9600);
//   pinMode(CANAL_A, INPUT_PULLUP);
//   pinMode(CANAL_B, INPUT_PULLUP);

//   pinMode(OUTPUT_1, OUTPUT);
//   pinMode(OUTPUT_2, OUTPUT);
//   pinMode(PWM, OUTPUT);

//   digitalWrite(OUTPUT_1, LOW);
//   digitalWrite(OUTPUT_2, HIGH);
//   digitalWrite(PWM, LOW);

//   pinMode(A0, INPUT);

//   attachInterrupt(digitalPinToInterrupt(CANAL_A), A_interrupt, RISING);

//   Serial.println("RPM");
// }

// void loop()
// {
//   int input = analogRead(A0);
//   int pwm = (int)(((float)input / 1023.0) * 255.0);

//   analogWrite(PWM, pwm);

//   unsigned long new_time = millis();
//   unsigned long delta_time = new_time - time;
//   if (delta_time >= 1000)
//   {
//     time = new_time;
//     rpm = (int)(proportionality_constant * ((float)position / (float)delta_time) * 60.0);
//     position = 0;
//     Serial.println(rpm);
//   }

//   delay(1000);
// }

#define CANAL_A 2
#define CANAL_B 3

#define OUTPUT_1 4
#define OUTPUT_2 4
#define PWM 6

volatile int position = 0;
unsigned long time = 0;
int rpm = 0;

// Nao sei porque, mas o valor de velocidade calculado
// eh em torno de 16.8 vezes menor que o valor real de rotacao
double proportionality_constant = 16.8;

int leitura_sensor0 = 0;
int leitura_sensor1 = 0;

void A_interrupt()
{
  int b = digitalRead(CANAL_B);

  if (b > 0)
  {
    position++;
  }
  else
  {
    position--;
  }
}

void vira_esquerda()
{
}

void setup()
{
  Serial.begin(9600);
  pinMode(CANAL_A, INPUT_PULLUP);
  pinMode(CANAL_B, INPUT_PULLUP);

  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);
  pinMode(PWM, OUTPUT);

  digitalWrite(OUTPUT_1, LOW);
  digitalWrite(OUTPUT_2, HIGH);
  digitalWrite(PWM, LOW);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  attachInterrupt(digitalPinToInterrupt(CANAL_A), A_interrupt, RISING);

  // Serial.println("RPM");
}

void loop()
{
  leitura_sensor0 = analogRead(A1);
  leitura_sensor1 = analogRead(A2);

  Serial.println(leitura_sensor0);
  Serial.println(leitura_sensor1);

  /*

  int input = analogRead(A0);
  int pwm = (int)(((float) input/1023.0)*255.0);

  analogWrite(PWM, pwm);

  unsigned long new_time = millis();
  unsigned long delta_time = new_time - time;
  if(delta_time >= 1000) {
    time = new_time;
    rpm = (int) (proportionality_constant*((float) position / (float) delta_time)*60.0);
    position = 0;
    Serial.println(rpm);
  }


  delay(1000);
  */
}