void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial.begin(9600);
}

void loop() {
 //Print X-pos analog 10-bit værdi (0-1023)
 Serial.print("X-pos= ");
 Serial.print(analogRead(A0));

  //Print X-pos analog 10-bit værdi (0-1023)
 Serial.print("          Y-pos= ");
 Serial.println(analogRead(A1));

}
