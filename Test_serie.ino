String nom = "Arduino";
String msg;

void readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    Serial.flush();
  }
}

void sendData() {
  Serial.print(digitalRead(LED_BUILTIN));
  Serial.print("x");
  Serial.print(analogRead(A0));
  Serial.print("x");
  Serial.print(analogRead(A1)); 
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  readSerialPort();

  if (msg == "data"){
    sendData();

  }
  else if (msg == "led0"){
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print(" Arduino set led to LOW");
  }
  else if (msg == "led1"){
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print(" Arduino set led to HIGH");
  }
  delay(500);   
}
