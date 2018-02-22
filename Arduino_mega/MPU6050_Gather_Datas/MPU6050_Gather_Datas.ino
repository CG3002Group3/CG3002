// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69); // <-- use for AD0 high

// Accel+Gyro1
int16_t ax1_i, ay1_i, az1_i;
int16_t gx1_i, gy1_i, gz1_i;
float ax1, ay1, az1;
float gx1, gy1, gz1;

// Accel+Gyro2
int16_t ax2_i, ay2_i, az2_i;
float ax2, ay2, az2;
int16_t gx2_i, gy2_i, gz2_i;
float gx2, gy2, gz2;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz
    // Please check.
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro1.initialize();
    accelgyro2.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro1.testConnection() ? "MPU6050 - accelgyro1 connection successful" : "MPU6050 - accelgyro1 connection failed");
    Serial.println(accelgyro2.testConnection() ? "MPU6050 - accelgyro2 connection successful" : "MPU6050 - accelgyro2 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro1.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro1.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro1.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro1.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro1.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro1.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    //accelgyro.setXGyroOffset(220);
    //accelgyro.setYGyroOffset(76);
    //accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro2.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro2.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro2.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro2.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro2.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro2.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
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

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
