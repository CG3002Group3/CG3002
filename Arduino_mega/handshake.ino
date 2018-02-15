int isConnected = 0;

void setup() {
// initialize both serial ports: 
  Serial.begin(9600); 
  Serial1.begin(9600);
}
void loop() {
// read from port 1, send to port 0:
//    Serial1.write("Hello\r"); 
  if (Serial1.available()) {
//    int inByte = Serial1.read();
    if (isConnected == 0) {
       int inByte = Serial1.read();
       if (inByte == 'H') {
        Serial.write("Received HELLO from Pi\n\r");
        Serial1.write('A');
      } else if (inByte == 'A') {
        Serial.write("Received ACK from Pi\n\r");
        isConnected = 1;
      }
    } else {
//      if (inByte == 'A') {
//        Serial.write("Received ACK to receive data from Pi\n\r");
//      }
        String receivedString = Serial1.readString();
        // say what you got:
        Serial.print("I received: ");
        Serial.println(receivedString);
    }
  }
  if (isConnected == 1) {
      delay(1000);
      Serial1.write("This is a very looooooooooooooooooooooooooooooooooooooog line to test.\n\r");
  }
    
}
