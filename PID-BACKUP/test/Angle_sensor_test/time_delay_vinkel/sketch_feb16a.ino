

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print(millis());Serial.print(",");Serial.println(Serial3.parseFloat());
}
