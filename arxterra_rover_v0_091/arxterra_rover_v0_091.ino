
/***********************************
 * Reminder: Serial Monitor Baud Rate set to 57600
 * to do list
 * pingers code to stop rover included but not tested/debugged
 * code for pan/tilt offset angle
 * continue to clean up code
 *
 * Servo Note:
 * the min pulse width (default is 544 microseconds) corresponds to the minimum angle (0-degree), shaft rotates fully left. 
 * the max pulse width, (defaults to 2400 microseconds) corresponds to the maximum angle (180-degree), shaft rotates fully right.  
 * rotation is left-to-right clockwise
 ***********************************/

/* The following two libraries are currently not 
 * implemented on the Pathfinder Rover
 * #include <Wire.h>      // I2C support
 * #include <L3G4200D.h>  // 3-axis Gyro
 */
/* The following two libraries are included with the Arduino IDE  */  
#include <SPI.h>

/* The Adb library has the following modifications made to bring
 * it into compliance Arduino version 1.04 (Compiles without errors)
 *  1. Adb.h, usb.cpp, and max3421e.cpp
 *  2. Change the line: #include "wiring.h" to #include "Arduino.h"
 * Runing on ADK generates a "OSCOKIRQ failed to assert" error.
 * Problem and possible solution provided here.
 *  http://forum.arduino.cc/index.php?topic=68205.0
 */
 
#include <Adb.h>

#define FALSE 0
#define TRUE  1

// Mapping pins 
// ATmega2560-Arduino http://arduino.cc/en/Hacking/PinMapping2560
// ATmega328-Arduino  http://arduino.cc/en/Hacking/PinMapping168
#include "pinouts_pathfinder.h"

// Commands to Numeric Value Mapping
//               Data[0] =   CMD TYPE | Qual
//                        bit  7654321   0      
#define MOVE         0x01   // 0000000   1        
#define CAMERA_MOVE  0x02   // 0000001   0                  
#define CAMERA_HOME  0x04   // 0000010   0

/* future
 * #define ATMEGA_TEMP    8
 * ATmega internal temperature sensor ... Need to research how to read
 *   http://forum.arduino.cc/index.php/topic,38043.0.html
 *   http://forum.arduino.cc/index.php/topic,26299.0.html
 *   Problem is time required when you switch between voltage reference sources
 *   for ADC to stabilize. This would limit how often we check this internal sensor.
 * cell phone temperature...     ArxRover needs to add
 */

// Adb connection.
Connection * connection;   // the connection variable holds a pointer (i.e. an address) to a variable of data type Connection

/* Saved in UserState File on Android Phone
 * public var cameraAdjustForMotion:Boolean = false;
 * public var cameraCanPan:Boolean = true;
 * public var cameraCanTilt:Boolean = true;
 * public var cameraConfigDefault:CameraConfig;
 * public var cameraConfigMotion:CameraConfig;
 * public var cameraIndex:int;
 * public var roverName:String;
 */
 
/* Defaults set in the Control Panel's Control Options pop-up Window 
 * Duty Cycle Steps      6 (4 -12)
 * Polling msec          500 (300 - 800)
 * Minimum Duty Cycle    100 (10 - 140)   *** ArxRover *** 
 * Top Duty Cycle        212 (180 - 255)
 * Motion State Display  ON  (Motion Text Display)
 * Control Tips Display  ON  (Tool Tips)  
 * Range Sensor Display  ON  (Numeric and Graphical Icon of Ultrasonic Data)
 */

// Timers and Sensor Variables and Default Values
unsigned long timer1;
unsigned long timer2;

// Battery Voltage Data
//Voltage Divider
const double R1 = 22000;  // 22K resistor from battery to analog input
const double R2 = 12000;  // 12K resistor from analog input to ground
const double K = (R2/(R1 + R2))*(1024/5.0); 
// LiPO
double  LiPO_V1 = 4.0;    // volts/cell, 90% point on LiPO curve at 1C
double  LiPO_V0 = 3.4;    // volts/cell, 10% point on LiPO curve at 1C
double  LiPO_Vsafe = 3.2; // volts/cell, 2.7 volts/cell = discharged voltage 

// Motor Battery
// Default = 7S NiMH
double  cells_per_motor_battery = 2;            // 2S or 3S
double  battery_motor_V1 = cells_per_motor_battery * LiPO_V1;    // volts, 90% point on LiPO curve at 1C
double  battery_motor_V0 = cells_per_motor_battery * LiPO_V0;    // error volts, 10% point on LiPO curve at 1C
double  battery_motor_dV  = battery_motor_V1 - battery_motor_V0;
double  battery_motor_Vsafe = cells_per_motor_battery * LiPO_Vsafe;  // volts, 2.7 volts/cell = discharged voltage

// Digital Battery
// Default = 8S NiMH
double  cells_per_digital_battery = 3;            // 2S or 3S
double  battery_digital_V1 = cells_per_digital_battery * LiPO_V1;       // volts, 90% point on LiPO curve at 1C
double  battery_digital_V0 = cells_per_digital_battery * LiPO_V0;       // volts, 10% point on LiPO curve at 1C
double  battery_digital_dV  = battery_digital_V1 - battery_digital_V0;  // difference
double  battery_digital_Vsafe = cells_per_digital_battery * LiPO_Vsafe; // volts, 2.7 volts/cell = discharged voltage 

boolean collisionDetection = FALSE;

// Event handler for the shell connection. 
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)    // declared void in version 1.00
{
  // Serial.print("adbEventHandler");
  // Data packets contain two bytes, one for each servo, in the range of [0..180]
  if (event == ADB_CONNECTION_RECEIVE)
  {
    Serial.print("rover received command ");
    uint8_t cmd = data[0];
    Serial.print(", ");
    Serial.print(cmd, HEX);
    Serial.print(", ");
    Serial.print(data[1], HEX);
    Serial.print(", ");
    Serial.print(data[2], HEX);
    Serial.print(", ");
    Serial.print(data[3], HEX);
    Serial.print(", ");
    Serial.println(data[4], HEX);
    // assumes only one command per packet
    
    if (cmd == MOVE) {
      /***********************************
      * motion command
      * motordata[1]    left run    (FORWARD, BACKWARD, BRAKE, RELEASE)
      * motordata[2]    left speed  0 - 255
      * motordata[3];   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
      * motordata[4];   right speed 0 - 255
      ***********************************/
      move(data);
      collisionDetection = (data[1] == 1) || (data[2] == 1);  // set to true if any motor is moving forward
    }
    else if (cmd == CAMERA_MOVE){
      /***********************************
       * pan and tilt command 
       * data[1]    0
       * data[2]    pan degrees (0 to 180)
       * data[3]    0
       * data[4]    tilt degrees (0 to 180)
       ***********************************/
      move_camera(data[2],data[4]);
      //Serial.print("Pan Servo To Position: ");
      //Serial.print(data[2]);
      //Serial.print(", ")
      //Serial.println(data[4]);
    }
    else if (cmd == CAMERA_HOME){
      /***********************************
       * camera home command 
       * pan position = 90 (default), tilt position = 90 (default)
       ***********************************/ 
      home_camera();
      // Serial.println("Camera Home");
   }
 }
}

void setup()
{
  Serial.begin(57600);
  Serial.print("\r\nStart\n");
  
  init_servos();
  
  ADB::init();     // source of OSCOKIRQ error

  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:4567", true, adbEventHandler); 
  timer1 = millis();
  timer2 = millis();
  
  // insure motors are off and turn off collisionDetection
  // stopMotors();
}

void loop()
{ 
  // Serial.println("I am alive");  
  sendData();
  
  if (collisionDetection){
    if (checkSonar()) {
        // insure motors are off and turn off collisionDetection
        stopMotors();
        collisionDetection = FALSE;   // rover is not moving
        // **** send Emergency Stop message ****
    }
  }  
  // Poll the ADB subsystem.
  ADB::poll();
}

int cleanBat = 1023;   //************************* REMOVE ************************

void sendData(){
   if(millis() - timer2 > 250) {            // Has it been over 300ms since last send? **** convert to interrupt? ****
    int motor1 = analogRead(MOTOR1CURRENT);
    int motor2 = analogRead(MOTOR2CURRENT);
    int tempRaw = analogRead(TEMPERATURE);
    int tempSensor = (500*tempRaw) >> 10;
    //cleanBat -= 100;
    //if (cleanBat < 0) cleanBat = 1023;
    // Serial.println(cleanBat);
    int cleanBat = analogRead(CLEAN_BAT);
    int dirtyBat = analogRead(DIRTY_BAT);
    
    int cleanBat_tp = 100*(cleanBat/K-battery_digital_V0)/battery_digital_dV;  // test point
    int dirtyBat_tp = 100*(dirtyBat/K-battery_motor_V0)/battery_motor_dV; 
    Serial.print("Clean Voltage= ");
    Serial.println(cleanBat_tp);
    Serial.print("Dirty Voltage= ");
    Serial.println(dirtyBat_tp);
    byte sendData[26];
    sendData[0]= 0x1;              // motor 1 current data package 
    sendData[1]= motor1 >> 8;
    sendData[2]= motor1 & 0xff;
    
    sendData[3]= 0x2;              // motor 2 current data package
    sendData[4]= motor2 >> 8;      
    sendData[5]= motor2 & 0xff;
    
    sendData[6]= 0x3;             // temperature sensor data package
    sendData[7]= tempSensor >> 8;
    sendData[8]= tempSensor & 0xff;
   
    //uint16_t range;              // local variable for range data  
    uint16_t range;
    sendData[9]= 0x4;            // distance sensor data package
    range = getRangeLeft();
    sendData[10]= range >> 8;   // C++ rangeLeft >> 8
    sendData[11]= range & 0xff;    // C++ rangeLeft & 0xff

    
    sendData[12]= 0x5;            // distance sensor data package
    range = getRangeRight();
    sendData[13]= range >> 8;   // C++ rangeLeft >> 8
    sendData[14]= range & 0xff;    // C++ rangeLeft & 0xff
    sendData[15]= 0x6;            // clean battery data package
    sendData[16]= cleanBat_tp >> 8;
    sendData[17]= cleanBat_tp & 0xff;
    
    sendData[18]= 0x7;            // dirty battery data package
    sendData[19]= dirtyBat_tp >> 8;
    sendData[20]= dirtyBat_tp & 0xff;
    
    sendData[21]= 0x8;            // pan and tilt angle data package
    int pan = getPanPosition(); 
    sendData[22]= pan >> 8;
    sendData[23]= pan & 0xff; 
    sendData[24]= 0x9;
    int tilt = getTiltPosition();
    sendData[25]= tilt >> 8;
    sendData[26]= tilt & 0xff;
    
    connection->write(27, sendData);   // see adb header file
    timer2 = millis(); //reset the timer
  }
   
}

