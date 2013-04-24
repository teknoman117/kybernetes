/*
 *  SensorController.ino
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
 */

// Bumper
#define BUMPER_FL PINB0
#define BUMPER_FR PINB1
#define BUMPER_BL PINB2
#define BUMPER_BR PINB3
#define BUMPER_DATA (PINB & 0x0F)      // Isolate the lower four bits, which represent the bumpers
#define BUMPER_DDR  DDRB

// Sonars
#define SONAR_COUNT 5
const int       sonars[SONAR_COUNT]      = {A0, A1, A2, A3, A4};
const int      _sonars[SONAR_COUNT]      = {6, 5, 4, 3, 2};
unsigned short  sonarValues[SONAR_COUNT] = {0, 0, 0, 0, 0};
unsigned char   currentSonar             = 0;
unsigned long   lastMeasurement          = 0;

// Status stuff
const int statusLED = 13;
const int extraLED = 12;
const int pushbuttonInput = 7;

// Command system
unsigned char  command = 0;        // stores a processed command
unsigned char  commandBytes = 0;   // stores the bytes this command expects
unsigned long  lastUpdate = 0;
unsigned char  state = 0;

// Setup the initial state of the controller
void setup() {
  // Start the serial uplink
  Serial.begin(57600); 

  // Configure status lights
  DDRB |= _BV(PORTB4) | _BV(PORTB5);
  DDRD &= ~_BV(PIND7);
  PORTD &= ~_BV(PIND4) & ~_BV(PIND5);
  
  // Configure the bumpers
  BUMPER_DDR &= ~_BV(BUMPER_FL) & ~_BV(BUMPER_FR) & ~_BV(BUMPER_BL) & ~_BV(BUMPER_BR); 

  // Configure the sonars
  DDRC = 0x00;
  for(unsigned char i = 0; i < 5; i++)
  {
    pinMode(_sonars[i], OUTPUT);
    digitalWrite(_sonars[i], LOW);
  }
  
  // Initialize each sonar, one by one
  digitalWrite(statusLED, HIGH);
  for(unsigned char i = 0; i < 5; i++)
  {
    digitalWrite(_sonars[i], HIGH);
    delay(400);
    digitalWrite(_sonars[i], LOW);
    delay(100);
  }
  digitalWrite(statusLED, LOW);
}

// Main loop of execution
void loop() {
  // Check if we are collecting bytes for a command and if we have achieved that number
  if((commandBytes > 0) && Serial.available() >= commandBytes) 
  {
    
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
    }
  }
  
  // Check if we need to measure the next sonar
  if((millis() - lastMeasurement) >= 50)
  {
    // Push the last measuremen forward
    lastMeasurement = millis();
    
    // Enable the sonar
    digitalWrite(_sonars[currentSonar], HIGH);
    delayMicroseconds(20);
    
    // Disable the sonar
    digitalWrite(_sonars[currentSonar], LOW);
    
    // Do shit
    if(state)
    {
      digitalWrite(extraLED, LOW);
      state = 0;
    } else {
      digitalWrite(extraLED, HIGH);
      state = 1;
    }
    
    // Setup next measurement
    if(++currentSonar >= SONAR_COUNT) currentSonar = 0;
    
    // Record the sonar values
    for(int i = 0; i < SONAR_COUNT; i++)
    {
        // Initialize variables
        sonarValues[i] = 0;
        long int total = 0;
        
        // Collect the average value
        for(unsigned char j = 0; j < 8; j++) {
          total += analogRead(sonars[i]);
          //delayMicroseconds(125);
        }
        sonarValues[i] = total / 8;
    }
  }

  // Check if we should upload a telemetry packet
  if((millis() - lastUpdate) >= 25)
  {
    // Store the current time
    lastUpdate = millis();
    
    // Print a binary representation of all of this data
    unsigned char b = BUMPER_DATA;
    Serial.write(&b, 1);
    Serial.write((unsigned char *) sonarValues, 10);
  }
}


