#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop() {
  int var = 0;
  while (var < 1){ //repeat while 1,000 milliseconds
    digitalWrite(PIN_LED, 0); //update LED status
    delay(1000); //wait for 1,000 milliseconds
    Serial.println(var++);
  }
  while (var < 2){
    for (int i = 0; i <= 4; i++){
      digitalWrite(PIN_LED, 1);//update LED status
      delay(100);//wait for 100 milliseconds
      digitalWrite(PIN_LED, 0);//update LED status
      delay(100);//wait for 100 milliseconds
    }
    Serial.println(var++);
  }
  while (var >= 2){
    digitalWrite(PIN_LED, 1);
    delay(1000);//wait for 1,000 milliseconds
    Serial.println(var++);
  }
}
