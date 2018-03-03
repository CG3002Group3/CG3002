void setup() {
// initialize both serial ports: 
  Serial.begin(115200);
  Serial1.begin(115200);

  connectToPi();
}

void loop() {

    
}

void connectToPi() {
  int handShakeFlag = 0;
  int ackFlag = 0;

  //receive Handshake from Rpi
  while (handShakeFlag == 0) {
    if (Serial1.available()) {
      if (Serial1.read() == 'H') {
        handShakeFlag = 1;
        Serial1.write('B');  //send a B to Pi
      }
    }
  }

  //Receive Ack from Rpi
  while (ackFlag == 0) {
    if (Serial1.available()) {
      int incoming = Serial1.read();
      Serial.println(char(incoming));
      if (incoming == 'F') {
        ackFlag = 1;
      }
    } else {
      Serial1.write('B');
    }
  }
  Serial.println("Arduino is Ready!");
}

