/*
 *  RazorGyro.ino
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Our Razor IMU suffered damage in a crash while testing kybernetes.  This
 *  crash destroyed the magnetometer on the board.  This code is a workaround
 *  to provide a heading given that the initial heading is known and also 
 *  estimates the tilt of the platforms
 */

//The Wire library is used for I2C communication
#include <Wire.h>
#include <stdint.h>

//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;

//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.
char itgAddress = 0x68;

int32_t posX = 0;
int32_t posY = 0;
int32_t posZ = 0;
unsigned long lastGyroUpdate = 0;

#define GYROX_CALIBRATION 30
#define GYROY_CALIBRATION 25
#define GYROZ_CALIBRATION 6

#define GYROX_MIN -15
#define GYROX_MAX 15
#define GYROY_MIN -15
#define GYROY_MAX 15
#define GYROZ_MIN -15
#define GYROZ_MAX 15

// Command system variables
unsigned char  command = 0;        // stores a processed command
unsigned char  commandBytes = 0;   // stores the bytes this command expects
unsigned long  lastUpdate = 0;

//In the setup section of the sketch the serial port will be configured, the i2c communication will be initialized, and the itg-3200 will be configured.
void setup()
{
  //Create a serial connection using a 9600bps baud rate.
  Serial.begin(57600);
  
  //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
  Wire.begin();
  
  //Read the WHO_AM_I register and print the result
  char id=0; 
  id = itgRead(itgAddress, 0x00);  
  Serial.print("ID: ");
  Serial.println(id, HEX);
  
  //Configure the gyroscope
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  
  //Set the sample rate to 100 hz
  itgWrite(itgAddress, SMPLRT_DIV, 9);
}

//The loop section of the sketch will read the X,Y and Z output rates from the gyroscope and output them in the Serial Terminal
void loop()
{
  // Check if we are collecting bytes for a command and if we have achieved that number
  if((commandBytes > 0) && Serial.available() >= commandBytes) 
  {
    // If this is the set gyro value
    if(command == 3)
    {
      // Buffer to store the received value
      float Z = 0.0;
      Serial.readBytes((char *) &Z, 4);  
      
      // Set the gyro integral value
      posZ = (int32_t)(Z * 1437.50);
    }
    
    // Clear command bytes
    commandBytes = 0;
  } 
  
  // Else if we aren't waiting for comand bytes and there is a command pair in the buffer
  else if((commandBytes == 0) && Serial.available() >= 2) 
  {
    // Check that the byte is the prefix of a command
    if(Serial.read() == '#')
    {
      // Get the command
      char cmd = Serial.read();
      
      // Send a synchronization token
      if(cmd == 'a')
      {
        command = 1;
        commandBytes = 0;
        
        // Write a string into the byte stream to look for
        Serial.print("#SYNCH");
        Serial.println();
      } 
      
      // Command to reset the Z axis gyro (reset the heading)
      else if(cmd == 'z')
      {
        // Set command
        command = 2;
        commandBytes = 0;
         
        // Reset the gyro values
        posZ = 0;
      }
      
      // Command to set the Z axis gyro to a given value
      else if(cmd == 's')
      {
        // Set command to set gyro
        command = 3;
        commandBytes = 4;
      
        // Wait for data reception to complete by setting command bytes  
      }
    }
  }
  
  // Check if it is time to update the gyro
  if((micros() - lastGyroUpdate) >= 10000)
  {
    // Store the last update time
    lastGyroUpdate = micros();
    
    // Temporary variables to hold rate calculations.
    int32_t xRate, yRate, zRate;
  
    // Read the rates from the gyroscope and adjust for known bias.
    xRate = readX() - GYROX_CALIBRATION;
    yRate = readY() - GYROY_CALIBRATION;
    zRate = readZ() - GYROZ_CALIBRATION;
    //xRate = readX();
    //yRate = readY();
    //zRate = readZ();
    
    // There is a minimal amount of noise coming out of the gyro when static, so adjust for it
    if(xRate < GYROX_MIN || xRate > GYROX_MAX) posX += -xRate;
    if(yRate < GYROY_MIN || yRate > GYROY_MAX) posY += -yRate;
    if(zRate < GYROZ_MIN || zRate > GYROZ_MAX) posZ += -zRate;
    
    // Adjust the position values into degrees
    float X = posX / 1437.50;
    float Y = posY / 1437.50;
    float Z = posZ / 1437.50;
    
    // Normalize angles to standard values 
    //    0 degrees = north, or in this case starting orientation of the gyro
    //    negative degrees = turned left
    //    positive degrees = turned right
    while(X > 180.0)  X -= 360.0;
    while(X < -180.0) X += 360.0;
    while(Y > 180.0)  Y -= 360.0;
    while(Y < -180.0) Y += 360.0;
    while(Z > 180.0)  Z -= 360.0;
    while(Z < -180.0) Z += 360.0;
    
    // Write the floating point data to our host
    Serial.write((uint8_t *) &X, 4);
    Serial.write((uint8_t *) &Y, 4);
    Serial.write((uint8_t *) &Z, 4);
    //Serial.print(X);
    //Serial.print(' ');
    //Serial.print(Y);
    //Serial.print(' ');
    //Serial.println(Z);
  }
  
  // If it is time for a data output, output data
  if((millis() - lastUpdate) > 25)
  {
    // Reset the next update value
    lastUpdate = millis();  
    
    // Output data
  }
}

//This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int readX(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_XOUT_L);  
  
  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int readY(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_YOUT_L);  
  
  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);  
  
  return data;
}
