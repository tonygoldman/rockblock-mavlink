void setup() {
  // put your setup code here, to run once:
  pinMode(D4, OUTPUT); // Configure the Pixhawk pin
  Serial.begin(115200);
  Serial.println("delay 10 seconds");
  delay(10000);
  Serial.println("D4 HIGH");
  digitalWrite(D4, HIGH);
  delay(5000);
  Serial.println("D4 LOW");
  digitalWrite(D4, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200000);
}
