// Roda invertida eh a da esquerda

#define PWM1_Ch 0
#define PWM2_Ch 1
#define PWM1_Res 8

#define PWM1_Freq 1000

#define CS_Sensors 29

#define SLC 36
#define SR 39
#define SC 34
#define SL 35
#define SRC 32

// #define MISO_ECHO 19

#define ECH_1 33
#define CHA_M1 25
#define ECH_2 26
#define CHA_M2 27

#define RS 17
#define E 16
const int DATA[] = {4, 0, 2, 15};

#define DATA_SIZE 4

// Commands: [- - - RS D7 D6 D5 D4]
#define FUNCTION_SET 0x20    // 0010 0000 - 4 bits
#define DISPLAY_CONTROL 0x0C // 0000 1100
#define CLEAR_DISPLAY 0x01   // 0000 0001
#define RETURN_HOME 0x02     // 0000 0010
#define ENTRY_MODE_SET 0x06  // 0000 0110

// States
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define DISPLAY_FUNCTION 0x08

#define kp 1
#define ki 1

void pulseEnable();
void initLCD();
void write4bits(int value);
void write8bits(int value);
void writeData(char *value);
void anda_reto(int velocidade);
void vira_esquerda(int velocidade);
void vira_direita(int velocidade);

void setup()
{
  Serial.begin(115200);

  /*
  // put your setup code here, to run once:
  pinMode(RS, OUTPUT);
  pinMode(E, OUTPUT);
  for (int i = 0; i < DATA_SIZE; i++){
    pinMode(DATA[i], OUTPUT);
  }
  */

  // initLCD();

  /*
  // Always good to clear and return home
  write8bits(CLEAR_DISPLAY);
  write8bits(RETURN_HOME);
  delay(2); // 1.52ms delay needed for the Return Home command
  writeData("Ain ze da manga");
  */

  pinMode(SLC, INPUT);
  pinMode(SR, INPUT);
  pinMode(SC, INPUT);
  pinMode(SL, INPUT);
  pinMode(SRC, INPUT);
  pinMode(CS_Sensors, OUTPUT);
  digitalWrite(CS_Sensors, HIGH);
  digitalWrite(ECH_1, HIGH);
  digitalWrite(ECH_2, HIGH);

  /*
  pinMode(ECH_1, OUTPUT);
  pinMode(CHA_M1, OUTPUT);
  pinMode(ECH_2, OUTPUT);
  pinMode(CHA_M2, OUTPUT);
*/
  /*
  ledcAttachPin(CHA_M1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  ledcAttachPin(CHA_M2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);
  */
}

void loop()
{
  // int input = analogRead(A0);
  // int pwm = (int)(((float) input/1023.0)*255.0);

  /*
  unsigned long new_time = millis();
  unsigned long delta_time = new_time - time;
  if(delta_time >= 1000) {
    time = new_time;
    rpm = (int) (proportionality_constant*((float) position / (float) delta_time)*60.0);
    position = 0;
    Serial.println(rpm);
  }
  */

  // leitura_sensor0 = analogRead(A1);
  // leitura_sensor1 = analogRead(A2);

  // Serial.println(leitura_sensor0);
  // Serial.println(leitura_sensor1);

  // analogWrite(PWM1, pwm);
  // analogWrite(PWM2, pwm);

  /*
  int slc = analogRead(SLC);
  int sr = analogRead(SR);
  int sc = analogRead(SC);
  int sl = analogRead(SL);
  int src = analogRead(SRC);
  int echo = analogRead(MISO_ECHO);

  digitalWrite(ECH_1, HIGH);
  digitalWrite(ECH_2, HIGH);
  ledcWrite(PWM1_Ch, 0);
  ledcWrite(PWM2_Ch, 10000);

  delay(2000);

  Serial.print("slc");
  Serial.print(slc);

  Serial.print("sr");
  Serial.print(sr);

  Serial.print("sc");
  Serial.print(sc);

  Serial.print("sl");
  Serial.print(sl);

  Serial.print("src");
  Serial.print(src);

  Serial.print("echo");
  Serial.print(echo);
*/

  /*
    digitalWrite(CS_Sensors,HIGH);
    digitalWrite(ECH_1, HIGH);
    digitalWrite(ECH_2, HIGH);
    ledcWrite(PWM1_Ch, 50);
    ledcWrite(PWM2_Ch, 200);
    delay(1000);
    ledcWrite(PWM1_Ch, 200);
    ledcWrite(PWM2_Ch, 50);
    delay(1000);
  */

  int slc = analogRead(SLC);
  int sr = analogRead(SR);
  int sc = analogRead(SC);
  int sl = analogRead(SL);
  int src = analogRead(SRC);

  int sr_anterior = 0;
  int sc_anterior = 0;
  int sl_anterior = 0;

  int erro_esquerdo = 4095 - sl;
  int erro_direito = 4095 - sr;

  int pwm_esquerdo = 63;
  int pwm_direito = 192;

  // int pwm_esquerdo = -1*((int) ((float) kp*(erro_esquerdo))*(127.0/4095.0)) + 127;
  // int pwm_direito = (int) ((float) kp*(erro_direito))*(127.0/4095.0);

  /*
  digitalWrite(ECH_1, HIGH);
  digitalWrite(ECH_2, HIGH);
  */

  Serial.printf("sr: %d, src: %d, sc: %d, slc: %d, sl: %d", sr, src, sc, slc, sl);
  // Serial.printf("pwd_dir: %d, pdw_esq: %d\n", pwm_esquerdo, pwm_direito);

  /*
  if(sl < 100) {
    vira_direita(60);
  }
  else if(sr < 100) {
    vira_esquerda(60);
  }
  else {
    anda_reto(60);
  }
  */

  delay(2000);
  // writeData(buf);
}

void anda_reto(int velocidade)
{
  ledcWrite(PWM1_Ch, 255 - velocidade);
  ledcWrite(PWM2_Ch, velocidade);
}

void vira_esquerda(int velocidade)
{
  ledcWrite(PWM1_Ch, velocidade);
  ledcWrite(PWM2_Ch, velocidade);
}

void vira_direita(int velocidade)
{
  ledcWrite(PWM1_Ch, 255 - velocidade);
  ledcWrite(PWM2_Ch, 255 - velocidade);
}

/* ------------------------------------------------------
  Pulses the Enable pin to send data to the display
*/
void pulseEnable()
{
  // Making sure the pin is LOW at first
  digitalWrite(E, LOW);
  delayMicroseconds(1);

  // Pulse the Enable pin
  digitalWrite(E, HIGH);
  delayMicroseconds(1);
  digitalWrite(E, LOW);
  delayMicroseconds(100);
}

/* ------------------------------------------------------
  Initializes the display as in Figure 24 of the
    HD44780U datasheet requests
*/
void initLCD()
{
  // Waiting at first
  delay(40);

  // Serial.println("Function set 4 bits 0b0010.");
  digitalWrite(RS, LOW);

  // Function set the interface with 4 bits
  write4bits(FUNCTION_SET >> 4);
  delayMicroseconds(4500); // A little more than 4.1 ms

  // Now, we set:
  // - Number of lines in the display (2 lines: 16x2)
  // - Size of the pixel matrix (5x8)
  // RS remains 0 (only is 1 when writing)
  // Serial.println("Function set 4 bits 0b0010 1000.");
  write8bits(FUNCTION_SET | DISPLAY_FUNCTION);

  // Display OFF
  // Serial.println("Display ON/OFF control 0b0000 1100.");
  write8bits(DISPLAY_CONTROL);

  // Entry mode set
  // Serial.println("Entry mode set 0b0000 0110.");
  write8bits(ENTRY_MODE_SET);

  // Clearing and returning home
  write8bits(CLEAR_DISPLAY);
  write8bits(RETURN_HOME);
  delay(2); // 1.52ms delay needed for the Return Home command

  // Now you're free to use the display
}

/* ------------------------------------------------------
  Actually sends the commands to the display
*/
void write4bits(int value)
{
  for (int i = 0; i < 4; i++)
  {
    // Only the value corresponding to the bit of interest
    digitalWrite(DATA[i], (value >> (3 - i)) & 0x1);
  }
  pulseEnable();
}

/* ------------------------------------------------------
  Writes first half of data, than second half
*/
void write8bits(int value)
{
  // Sends first half of the data (upper part):
  write4bits(value >> 4);
  // Sends last half of the data (lower part):
  write4bits(value);
}

void writeData(const char *value)
{
  char c;
  for (int i = 0; i < strlen(value); i++)
  {
    c = value[i];

    digitalWrite(RS, HIGH);
    write8bits(c);
    digitalWrite(RS, LOW);
  }
}