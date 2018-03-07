#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define STACK_SIZE 200
#define OUTPUT_READABLE_ACCELGYRO
#define SAMPLE_SIZE 50

// Constants
const int SENSOR_PIN = A0;  // Input pin for measuring Vout
const float RS = 0.1;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read

// Global Variables
float sensorValue = 0;   // Variable to store value from analog read
float sensorValue2 = 0;
float voltage = 0;
float current = 0;       // Calculated current value from current sensor
float power = 0;  // Calculated Power Value
float sum=0;
int handShakeFlag = 0;
int ackFlag = 0;
unsigned char deviceCode = 0;
bool blinkState = false;

MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69); // <-- use for AD0 high

// Accel+Gyro1
int16_t ax1_i, ay1_i, az1_i;
int16_t gx1_i, gy1_i, gz1_i;
float ax1, ay1, az1;
float gx1, gy1, gz1;

// Accel+Gyro2
int16_t ax2_i, ay2_i, az2_i;
int16_t gx2_i, gy2_i, gz2_i;
float ax2, ay2, az2;
float gx2, gy2, gz2;

// testing structures for serializing
typedef struct testData{
  byte numberArray[10];
} TestConfigPacket;

TestConfigPacket testPkt;

// structures for serializing
typedef struct data{
    float x_accel_1;
    float y_accel_1;
    float z_accel_1;
    float x_accel_2;
    float y_accel_2;
    float z_accel_2;
    float x_gyro_1;
    float y_gyro_1;
    float z_gyro_1;
    float x_gyro_2;
    float y_gyro_2;
    float z_gyro_2;
    float voltage_output;
    float current_output;
    float power_output;
} TConfigPacket;

TConfigPacket cfgPkt;

// Testing values
byte testArray[10] = {12,34,15,24,73,25,15,15,36,19};
char testCharArray[10] = {'A','B','C','D','E','F','G','H','I','J'};

void connectToPi() {
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
  
  while(1){
    if (Serial1.available()) {
      int incoming = Serial1.read();
      Serial.println(char(incoming));
      if (incoming == 'R') {
        //sendCharArray();
        testingPacketReading();
        //packetReading()
        //sendCurrentReading();
      }
    }
    delay(1000);
  }

}

void sendCurrentReading(){
  sensorValue2 = analogRead(A2);

  //for(int i = 0; i< 10; i++){
    sensorValue = analogRead(SENSOR_PIN);
    //sum += sensorValue;
  //}
  //sensorValue = sum/10.0;
  //sum = 0;
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023.0;
  Serial.print(sensorValue, 3);
  Serial.print(" ");
  Serial.println("A");
  
  sensorValue2 = (sensorValue2 * VOLTAGE_REF) / 1023.0;
  Serial.print("sensorValue2: ");
  Serial.print(sensorValue2);
  Serial.println(" V");

  char currentChar[4], voltageChar[4];
  itoa(sensorValue, currentChar, 10);
  itoa(sensorValue2, voltageChar, 10);
  Serial1.write(currentChar);
  Serial1.write(voltageChar);
}

void sendCharArray(){
  Serial1.write(testCharArray, 10);
}

void testingPacketReading(){
  char buffer[64];
  for(int i = 0; i < 10; i++){
    testPkt.numberArray[i] = testArray[i];
  }
  unsigned len = serialize(buffer, &testPkt, sizeof(testPkt));
  sendSerialData(buffer, len); 
}

void packetReading(){
  char buffer[128];
  cfgPkt.x_accel_1 = ax1;
  cfgPkt.y_accel_1 = ay1;
  cfgPkt.z_accel_1 = az1;
  cfgPkt.x_accel_2 = ax2;
  cfgPkt.y_accel_2 = ay2;
  cfgPkt.z_accel_2 = az2;
  cfgPkt.x_gyro_1 = gx1;
  cfgPkt.y_gyro_1 = gy1;
  cfgPkt.z_gyro_1 = gz1;
  cfgPkt.x_gyro_2 = gx2;
  cfgPkt.y_gyro_2 = gy2;
  cfgPkt.z_gyro_2 = gz2;
  cfgPkt.voltage_output = voltage;
  cfgPkt.current_output = current;
  cfgPkt.power_output = power;

  unsigned len = serialize(buffer, &cfgPkt, sizeof(cfgPkt));
  sendSerialData(buffer, len);        
}

unsigned int serialize(char *buf, void *p, size_t size){
  char checksum = 0;
  buf[0] = size;
  memcpy(buf+1, p, size); //copies number of bytes from location pointed to by the source
  for(int i = 1; i <= size; i++){
    checksum ^= buf[i];
  }
  buf[size+1] = checksum; //store checksum value as last value in buffer
  return size+2;
}

void sendSerialData(char *buf, int len){
  for(int i = 0; i < len; i ++){
    Serial1.write(buf[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // ACC setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro1.initialize();
  accelgyro2.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro1.testConnection() ? "MPU6050 - accelgyro1 connection successful" : "MPU6050 - accelgyro1 connection failed");
  Serial.println(accelgyro2.testConnection() ? "MPU6050 - accelgyro2 connection successful" : "MPU6050 - accelgyro2 connection failed");

  connectToPi();
  xTaskCreate(packetReading, "PacketReading", 3000, NULL, 1, NULL); // Only one task sending 50 samples
}

void loop() {
  // read raw accel/gyro measurements from device
    //accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    //accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    
    accelgyro1.getAcceleration(&ax1_i, &ay1_i, &az1_i);
    accelgyro1.getRotation(&gx1_i, &gy1_i, &gz1_i);
    accelgyro2.getAcceleration(&ax2_i, &ay2_i, &az2_i);
    accelgyro2.getRotation(&gx2_i, &gy2_i, &gz2_i);
    
    // Convert into readable value (G + Degree/sec)
    ax1 = ax1_i/16384.0;
    ay1 = ay1_i/16384.0;
    az1 = az1_i/16384.0;
    gx1 = gx1_i/131.0;
    gy1 = gy1_i/131.0;
    gz1 = gz1_i/131.0;

    ax2 = ax2_i/16384.0;
    ay2 = ay2_i/16384.0;
    az2 = az2_i/16384.0;
    gx2 = gx2_i/131.0;
    gy2 = gy2_i/131.0;
    gz2 = gz2_i/131.0;
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax1); Serial.print("\t");
        Serial.print(ay1); Serial.print("\t");
        Serial.print(az1); Serial.print("\t");
        Serial.print(gx1); Serial.print("\t");
        Serial.print(gy1); Serial.print("\t");
        Serial.print(gz1);
        
        Serial.print("\t - \t");
        Serial.print(ax2); Serial.print("\t");
        Serial.print(ay2); Serial.print("\t");
        Serial.print(az2); Serial.print("\t");
        Serial.print(gx2); Serial.print("\t");
        Serial.print(gy2); Serial.print("\t");
        Serial.println(gz2);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
    /*
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    */
    #endif
}
