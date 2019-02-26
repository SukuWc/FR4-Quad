/*
Connect PPM signal to Arduino Micro Pin 10.
*/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

int cntr = 0;
long int val  = 0;

void loop() {
  
  val = pulseIn(10, HIGH);
  
  if (val>2500){
    cntr = 0;  
    Serial.println("");
    
    Serial.print("Sync: ");
    Serial.print(val);
    Serial.print(" CH:  ");
  }
  else{
    Serial.print(val); 
    Serial.print("  ");
    cntr++;
  }

}
