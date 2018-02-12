#include <croutine.h>
#include <event_groups.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
#include <list.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <StackMacros.h>
#include <task.h>
#include <timers.h>

#include <Wire.h>

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_WHO_AM_I           0x75   // R


#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_I2C_ADDRESS2 0x69

#define MPU6050_ACCEL_XOUT_H       0x3B   // R 


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};


void setup()
{      
  int error, error2;
  uint8_t c;


  Serial.begin(9600);
  Serial.println(F("InvenSense MPU-6050"));
  Serial.println(F("June 2012"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  error2 = MPU6050_read2 (MPU6050_PWR_MGMT_2, &c, 1);

  
  Serial.print(F("PWR_MGMT_1 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  MPU6050_write_reg2 (MPU6050_PWR_MGMT_2, 0);
}


void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  accel_t_gyro_union accel_t_gyro2;


//  Serial.println(F(""));
//  Serial.println(F("MPU-6050"));

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  error = MPU6050_read2 (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro2, sizeof(accel_t_gyro2));
//  Serial.print(F("Read accel, temp and gyro, error = "));
//  Serial.println(error,DEC);


  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  
  SWAP (accel_t_gyro2.reg.x_accel_h, accel_t_gyro2.reg.x_accel_l);
  SWAP (accel_t_gyro2.reg.y_accel_h, accel_t_gyro2.reg.y_accel_l);
  SWAP (accel_t_gyro2.reg.z_accel_h, accel_t_gyro2.reg.z_accel_l);
  SWAP (accel_t_gyro2.reg.t_h, accel_t_gyro2.reg.t_l);
  SWAP (accel_t_gyro2.reg.x_gyro_h, accel_t_gyro2.reg.x_gyro_l);
  SWAP (accel_t_gyro2.reg.y_gyro_h, accel_t_gyro2.reg.y_gyro_l);
  SWAP (accel_t_gyro2.reg.z_gyro_h, accel_t_gyro2.reg.z_gyro_l);


  // Print the raw acceleration values
  
//****************************************************************

//    Serial.println("ACC#1");
//    Serial.print(accel_t_gyro.value.x_accel, DEC);
//    Serial.print(" ");
//    Serial.print(accel_t_gyro.value.y_accel, DEC);
//    Serial.print(" ");
//    Serial.println(accel_t_gyro.value.z_accel, DEC);
//   
////****************************************************************
//
//    Serial.println("ACC#2");
//    Serial.print(accel_t_gyro2.value.x_accel, DEC);
//    Serial.print(" ");
//    Serial.print(accel_t_gyro2.value.y_accel, DEC);
//    Serial.print(" ");
//    Serial.println(accel_t_gyro2.value.z_accel, DEC);

//****************************************************************

  // Print the raw gyro values.

//****************************************************************
//
//    Serial.println("gyro#1");
//    Serial.print(accel_t_gyro.value.x_gyro, DEC);
//    Serial.print(" ");
//    Serial.print(accel_t_gyro.value.y_gyro, DEC);
//    Serial.print(" ");
//    Serial.println(accel_t_gyro.value.z_gyro, DEC);


//****************************************************************

    Serial.println("gyro#2");
    Serial.print(accel_t_gyro2.value.x_gyro, DEC);
    Serial.print(" ");
    Serial.print(accel_t_gyro2.value.y_gyro, DEC);
    Serial.print(" ");
    Serial.println(accel_t_gyro2.value.z_gyro, DEC);

//****************************************************************

  delay(20);
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

int MPU6050_read2(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS2);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS2, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

int MPU6050_write2(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS2);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

int MPU6050_write_reg2(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write2(reg, &data, 1);

  return (error);
}

