// In terminal set line ending to Both NL + CR

void setup() {
  Serial.begin(115200);

  // AT BAUD 38400
  Serial1.begin(115200);
}

void loop() {

  if (Serial1.available()){

    Serial.write(Serial1.read());


  }
  
  if (Serial.available()){
    char a = Serial.read();
    Serial.write(a);
    Serial1.write(a);
  }
  // put your main code here, to run repeatedly:

}
