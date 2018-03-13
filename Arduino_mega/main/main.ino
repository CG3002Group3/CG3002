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

MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69);

// Constants
const int SENSOR_PIN = A1;  // Input pin for measuring Vout
const float RS = 0.1;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read

// Global Variables
float currSensor = 0;   // Variable to store value from analog read
float voltSensor = 0;
float current;       // Calculated current value from current sensor
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

// Semaphores
SemaphoreHandle_t dataSemaphore = xSemaphoreCreateBinary();

// Buffer
char ax1Char[4], ay1Char[4], az1Char[4], gx1Char[4], 
gy1Char[4], gz1Char[4], ax2Char[4], ay2Char[4], 
az2Char[4], gx2Char[4], gy2Char[4], gz2Char[4], 
voltChar[4], currentChar[4], powerChar[4];
char dataBuffer[3000];

// Checksum
char checkSum = 0;
int checkSum2;
char checksumChar[4];

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

void readAcc(void *p){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS; // read acc every 50ms

  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    processAcc();
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      // ACC 1
      itoa(ax1, ax1Char, 10);    //convert int to char[]
      strcat(dataBuffer, ax1Char);
      strcat(dataBuffer, ",");
      itoa(ay1, ay1Char, 10);
      strcat(dataBuffer, ax1Char);
      strcat(dataBuffer, ",");
      itoa(az1, az1Char, 10);
      strcat(dataBuffer, az1Char);
      strcat(dataBuffer, ",");
  
      // ACC 1
      itoa(ax2, ax2Char, 10);    //convert int to char[]
      strcat(dataBuffer, ax2Char);
      strcat(dataBuffer, ",");
      itoa(ay2, ay2Char, 10);
      strcat(dataBuffer, ax2Char);
      strcat(dataBuffer, ",");
      itoa(az2, az2Char, 10);
      strcat(dataBuffer, az2Char);
      strcat(dataBuffer, ",");

      // Gyro 1
      itoa(gx1, gx1Char, 10);    //convert int to char[]
      strcat(dataBuffer, gx1Char);
      strcat(dataBuffer, ",");
      itoa(gy1, gy1Char, 10);
      strcat(dataBuffer, gx1Char);
      strcat(dataBuffer, ",");
      itoa(gz1, gz1Char, 10);
      strcat(dataBuffer, gz1Char);
      strcat(dataBuffer, ",");

      // Gyro 2
      itoa(gx2, gx2Char, 10);    //convert int to char[]
      strcat(dataBuffer, gx2Char);
      strcat(dataBuffer, ",");
      itoa(gy2, gy2Char, 10);
      strcat(dataBuffer, gx2Char);
      strcat(dataBuffer, ",");
      itoa(gz2, gz2Char, 10);
      strcat(dataBuffer, gz2Char);
      strcat(dataBuffer, ",");
      
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
void readVI(void *p){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS; // read power every 1s

  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      processPower();
      itoa(voltSensor, voltChar, 10);    //convert int to char[]
      strcat(dataBuffer, voltChar);
      strcat(dataBuffer, ",");
      itoa(currSensor, currentChar, 10);
      strcat(dataBuffer, currentChar);
      strcat(dataBuffer, ",");
      itoa(power, powerChar, 10);
      strcat(dataBuffer, powerChar);
      strcat(dataBuffer, ",");
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

void sendToPi(void *p){
  char buffer[128];
  int incoming = Serial1.read();
  unsigned len;
  unsigned long timeNow;
  char s[4];
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS; // send data to PI every 200ms

  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if (xSemaphoreTake (dataSemaphore, 1) == pdTRUE) {
      Serial.println("I'm SENDING!");       
      len = strlen(dataBuffer);
      for (int i = 0; i < len; i++) {
        checkSum ^= dataBuffer[i];
      }

      // check sum
      checkSum2 = (int)checkSum;
      itoa(checkSum2, checksumChar, 10);
      strcat(dataBuffer, ","); 
      strcat(dataBuffer, checksumChar);

      // get current timestamp
      timeNow = millis();
      itoa(timeNow, s, 10);
      strcat(dataBuffer, ","); 
      strcat(dataBuffer, s);

      len = strlen(dataBuffer);
      dataBuffer[len + 1] = '\n'; // new line character 
      
      //Send message to Rpi
//      for (int j = 0; j < len; j++) {
//        Serial1.write(dataBuffer[j]);
//      }
      memset(dataBuffer, 0, sizeof dataBuffer);
      
      xSemaphoreGive(dataSemaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
  
  //connectToPi();
  xSemaphoreGive(dataSemaphore);
  xTaskCreate(sendToPi, "Sending data packets", 400, NULL, 3, NULL);
  Serial.println("Task1 activated!");
  xTaskCreate(readAcc, "Read accelerometer values", 400, NULL, 2, NULL);
  Serial.println("Task2 activated!");
  xTaskCreate(readVI, "Read VIP", 400, NULL, 1, NULL);
  Serial.println("Task3 activated!");
}

void loop() {

}
