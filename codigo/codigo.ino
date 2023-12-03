
int leitura_sensor0;
int leitura_sensor1;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly

  leitura_sensor0 = analogRead(A0);
  leitura_sensor1 = analogRead(A1);
  
  Serial.print(leitura_sensor0);
  Serial.print(", ");
  Serial.println(leitura_sensor1);
  delay(200);
}