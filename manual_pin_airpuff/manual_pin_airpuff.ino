static byte pin = 8; 

void setup() {
  // put your setup code here, to run once:
pinMode(pin, OUTPUT);



}

 void loop() {
   digitalWrite(pin, HIGH);
   delay(5000);
   digitalWrite(pin, LOW);
   delay(20000);
   }
