#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define STACK_SIZE 200
#define OUTPUT_READABLE_ACCELGYRO
#define SAMPLE_SIZE 4

MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69);


// Constants
const int SENSOR_PIN = A1;  // Input pin for measuring Vout
const float RS = 0.1;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read

// Global Variables
float currSensor = 0;   // Variable to store value from analog read
float voltSensor = 0;
float current = 0;       // Calculated current value from current sensor
float power = 0;  // Calculated Power Value
float energy = 0;
unsigned long prev_time, diff_time;
unsigned long curr_time;

int handShakeFlag = 0;
int ackFlag = 0;

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

// Buffer
char* ax1Char; char* ay1Char; char* az1Char; char* gx1Char;
char* gy1Char; char* gz1Char; char* ax2Char; char* ay2Char; 
char* az2Char; char* gx2Char; char* gy2Char; char* gz2Char;
char* voltChar; char* currentChar; char* powerChar;
char dataBuffer[3000];

// Checksum
int checkSum = 0;
int checkSum2;
char checksumChar[4];

// Semaphores
SemaphoreHandle_t dataSemaphore = xSemaphoreCreateBinary();

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
}

void sendToPi() {
  int incomingByte;
  unsigned int len;
  char s[4];
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS; // send data to PI every 200ms

  xLastWakeTime = xTaskGetTickCount();

  for(;;){
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      if (Serial1.available()) {       // Check if message available
        incomingByte = Serial1.read();
      }    
    }
    if(incomingByte == 'R'){
      strcpy(dataBuffer, ""); //clear the dataBuffer
      Serial.println("Sending data to PI");

      len = strlen(dataBuffer);
      for (int i = 0; i < len; i++) {
        checkSum += dataBuffer[i];
      }
      Serial.print("Checksum is ");
      Serial.println(checkSum);

      len = strlen(dataBuffer);

      //Send message to rpi
      for (int j = 0; j < len + 1; j++) {
        Serial1.write(dataBuffer[j]);
      }
      incomingByte = 0;
      checkSum = 0;
      xSemaphoreGive(dataSemaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void readAcc() {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // read acc every 10ms

  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      processAcc();
      ACCMessageFormat();
      Serial.println("Read ACC");
      xSemaphoreGive(dataSemaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void processAcc(){
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

void ACCMessageFormat(){
  char buffer[100];
  // ACC 1
  ax1Char = dtostrf(ax1, 3, 2, buffer);
  strcat(dataBuffer, ax1Char);
  strcat(dataBuffer, ",");
  ay1Char = dtostrf(ay1, 3, 2, buffer);
  strcat(dataBuffer, ay1Char);
  strcat(dataBuffer, ",");
  az1Char = dtostrf(az1, 3, 2, buffer);
  strcat(dataBuffer, az1Char);
  strcat(dataBuffer, ",");
  
  // ACC 1
  ax2Char = dtostrf(ax2, 3, 2, buffer);
  strcat(dataBuffer, ax2Char);
  strcat(dataBuffer, ",");
  ay2Char = dtostrf(ay2, 3, 2, buffer);
  strcat(dataBuffer, ay2Char);
  strcat(dataBuffer, ",");
  az2Char = dtostrf(az2, 3, 2, buffer);
  strcat(dataBuffer, az2Char);
  strcat(dataBuffer, ",");

  // Gyro 1
  gx1Char = dtostrf(gx1, 3, 2, buffer);
  strcat(dataBuffer, gx1Char);
  strcat(dataBuffer, ",");
  gy1Char = dtostrf(gy1, 3, 2, buffer);
  strcat(dataBuffer, gy1Char);
  strcat(dataBuffer, ",");
  gz1Char = dtostrf(gz1, 3, 2, buffer);
  strcat(dataBuffer, gz1Char);
  strcat(dataBuffer, ",");

  // Gyro 2
  gx2Char = dtostrf(gx2, 3, 2, buffer);
  strcat(dataBuffer, gx2Char);
  strcat(dataBuffer, ",");
  gy2Char = dtostrf(gy2, 3, 2, buffer);
  strcat(dataBuffer, gy2Char);
  strcat(dataBuffer, ","); 
  gz2Char = dtostrf(gz2, 3, 2, buffer);
  strcat(dataBuffer, gz2Char);
  strcat(dataBuffer, ",");

  //char buffer[10];
  voltChar = dtostrf(voltSensor, 3, 2, buffer);
  strcat(dataBuffer, voltChar);
  strcat(dataBuffer, ",");
  currentChar = dtostrf(currSensor, 3, 2, buffer);
  strcat(dataBuffer, currentChar);
  strcat(dataBuffer, ",");
  powerChar = dtostrf(power, 3, 2, buffer);
  strcat(dataBuffer, powerChar);
  strcat(dataBuffer, "\n");
}

void readVI() {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS; // read power every 1s

  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      processPower();
      Serial.println("Read power!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      xSemaphoreGive(dataSemaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void processPower(){
  prev_time = micros();
  voltSensor = analogRead(A0);
  currSensor = analogRead(A1);
  voltSensor = ((voltSensor * 5) / 1023) * 2;  //278.333333 = V * R1 / R1 + R2
  currSensor = ((currSensor * 5) / 1023);
  //current = currSensor / (10.0 * 0.1);
  power = voltSensor * currSensor;

  Serial.print(diff_time);
  Serial.println(" *");
  
  Serial.print("Voltage: ");
  Serial.print(voltSensor);
  Serial.println(" V");

  Serial.print("Current: ");
  Serial.print(currSensor);
  Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(power);
  Serial.println(" W");

  curr_time = micros();
  diff_time = curr_time - prev_time;
  energy += power * diff_time / 1000000;
  Serial.print("Energy: ");
  Serial.print(energy);
  Serial.println(" J");
  Serial.println(" ");
}

void setupACC(){
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

  // use the code below to change accel/gyro offset values
  //*
  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  Serial.println("Accelgyro1: ");
  Serial.print(accelgyro1.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro1.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro1.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro1.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro1.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro1.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  accelgyro1.setXAccelOffset(-4155);
  accelgyro1.setYAccelOffset(346);
  accelgyro1.setZAccelOffset(1316);
  accelgyro1.setXGyroOffset(-7);
  accelgyro1.setYGyroOffset(-38);
  accelgyro1.setZGyroOffset(25);

  Serial.println("Accelgyro2: ");
  Serial.print(accelgyro2.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro2.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro2.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro2.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro2.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro2.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  accelgyro2.setXAccelOffset(-1705);
  accelgyro2.setYAccelOffset(-3766);
  accelgyro2.setZAccelOffset(826);
  accelgyro2.setXGyroOffset(38);
  accelgyro2.setYGyroOffset(-7);
  accelgyro2.setZGyroOffset(-3);
  //*/
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Test test");

  setupACC();
  connectToPi();

  xTaskCreate(sendToPi, "Sending data packets", 400, NULL, 3, NULL);
  xTaskCreate(readAcc, "Read accelerometer values", 400, NULL, 2, NULL);
  xTaskCreate(readVI, "Read voltage, current and calculate power", 400, NULL, 1, NULL);
}

void loop() {
  

}
