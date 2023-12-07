// Roda invertida eh a da esquerda

#define PWM_Ch1 0
#define PWM_Ch2 1
#define PWM_Ch3 2
#define PWM_Ch4 3

#define PWM_Res 8
#define PWM_Freq 1000

#define CS_Sensors 29

#define SLC 36
#define SR 34
#define SC 39
#define SL 35
#define SRC 32

// #define MISO_ECHO 19

#define M1_PWM1 23
#define M1_PWM2 25
#define M2_PWM1 26
#define M2_PWM2 27

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

  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);
  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);

  ledcAttachPin(M1_PWM1, PWM_Ch1);
  ledcAttachPin(M1_PWM2, PWM_Ch2);
  ledcAttachPin(M2_PWM1, PWM_Ch3);
  ledcAttachPin(M2_PWM2, PWM_Ch4);

  ledcSetup(PWM_Ch1, PWM_Freq, PWM_Res);
  ledcSetup(PWM_Ch2, PWM_Freq, PWM_Res);
  ledcSetup(PWM_Ch3, PWM_Freq, PWM_Res);
  ledcSetup(PWM_Ch4, PWM_Freq, PWM_Res);

  pinMode(SR, INPUT);
}

void loop()
{
  ledcWrite(PWM_Ch1, 0);
  ledcWrite(PWM_Ch2, 255);

  // direita
  ledcWrite(PWM_Ch3, 0);
  ledcWrite(PWM_Ch4, 255);

  int input = analogRead(SR);
  Serial.println(input);

  delay(1000);
}

// void anda_reto(int velocidade)
// {
//   ledcWrite(PWM1_Ch, 255 - velocidade);
//   ledcWrite(PWM2_Ch, velocidade);
// }

// void vira_esquerda(int velocidade)
// {
//   ledcWrite(PWM1_Ch, velocidade);
//   ledcWrite(PWM2_Ch, velocidade);
// }

// void vira_direita(int velocidade)
// {
//   ledcWrite(PWM1_Ch, 255 - velocidade);
//   ledcWrite(PWM2_Ch, 255 - velocidade);
// }

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