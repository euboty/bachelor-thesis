
// Libraries
#include <HX711.h>    
#include <Servo.h>    

// Pi Value
#define PI 3.1415926535897932384626433832795

// Force-Sensing Objects
HX711 FORCE_RIGHT;           
HX711 FORCE_LEFT;      

// Servo Objects
Servo OS_RIGHT;                      
Servo OS_LEFT;                      
Servo WS_RIGHT;                  
Servo WS_LEFT;                    

// Force-Sensor Pinout
const byte FORCE_RIGHT_DOUT_PIN = 3;
const byte FORCE_RIGHT_SCK_PIN = 4;
const byte FORCE_LEFT_DOUT_PIN = 12; 
const byte FORCE_LEFT_SCK_PIN = 11;

// Servo Pinout
const byte OS_RIGHT_PIN = 5;
const byte OS_LEFT_PIN = 6;
const byte WS_RIGHT_PIN = 9;
const byte WS_LEFT_PIN = 10;

// Timing Vars
int mister_t; //Iterator in Motorcontroll Loops KANN WEG ???
const int SERVO_TIMEOUT = 3; //Step in Motorcontroll Loops

// Force-Sensing Vars
long force_prev_value_right = 0;
long force_value_right = 0;
long force_prev_value_left = 0;
long force_value_left = 0;
long force_prev_value_avg;
long force_value_avg;

// Program (Mode)
int Program = 1;

/*
 * Callibration:
 * Following values describe the opening and closing positions of the servos (PWM signal).
 * The sketch "Servo_callibration" was used to figure those values.
 * Perspective is mover to foil:
 * OS_LEFT (OS Left)
 *  open 1020
 *  closed 1620
 * OS_RIGHT (OS Right)
 *  open 1700
 *  closed 1094
 * WS_LEFT (WS Left)
 *  extended 2260
 *  pull 1200
 * WS_RIGHT (WS Right)
 *  extended 0
 *  pull 1505
 *  
 *  Those are set as postions and a motion range is set (f.e. OS_MOTION_RANGE = 1700 - 1094). This way both servos have to move the same amount.
 */

// OS Servo Callibration
int OS_MOTION_RANGE = 605;                                                // same for both servos
int os_pos_right;                                                         // tracks is position
int OS_POS_OPEN_RIGHT = 1700;                                             // callibration postition
int OS_POS_CLOSED_RIGHT = OS_POS_OPEN_RIGHT-OS_MOTION_RANGE;              // callibration postition
int os_pos_left;                                                          // tracks is position
int OS_POS_OPEN_LEFT = 1020;                                              // callibration postition
int OS_POS_CLOSED_LEFT = OS_POS_OPEN_LEFT + OS_MOTION_RANGE;              // not in use, just as explaination


// WS Servo Callibration
int WS_MOTION_RANGE = 1060; //Werte des PWM Signal mit dem Sketch Servo_Callibration ermittelt
int ws_pos_right; 
int ws_pos_ext_right = 445;
int ws_pos_left; 
int ws_pos_ext_left = 2200;
int WS_STEP = 18; 

//*** Preparation before loop*** 
void setup() {
  Serial.begin(57600);   //Baudrate (Communication Speed) of serial port (:= 57,6 kbps)
  delay(10);
  
  // configuration of servos
  OS_RIGHT.attach(OS_RIGHT_PIN);
  OS_LEFT.attach(OS_LEFT_PIN);
  WS_RIGHT.attach(WS_RIGHT_PIN);
  WS_LEFT.attach(WS_LEFT_PIN);

  // move servos in starting position and save as is position
  OS_RIGHT.writeMicroseconds(OS_POS_OPEN_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_OPEN_LEFT);
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);
  os_pos_right = OS_POS_OPEN_RIGHT;
  os_pos_left = OS_POS_OPEN_LEFT;
  ws_pos_right = ws_pos_ext_right;
  ws_pos_left = ws_pos_ext_left; 

  // attach buildin LED as status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /*
  // Configuration of Force-Sensors
  FORCE_RIGHT.begin(FORCE_RIGHT_DOUT_PIN, FORCE_RIGHT_SCK_PIN);
  FORCE_RIGHT.is_ready();
  FORCE_LEFT.begin(FORCE_LEFT_DOUT_PIN, FORCE_LEFT_SCK_PIN);
  FORCE_LEFT.is_ready();
  ledDelayBlink(2000, 10);                                                                 // warmup time for strain gauges with blinking indator led - delay 2000 milliseconds with 10 blinks                                                         
  FORCE_RIGHT.set_scale();
  FORCE_RIGHT.tare();
  FORCE_RIGHT.get_units(10);
  FORCE_RIGHT.set_scale();
  FORCE_LEFT.set_scale();
  FORCE_LEFT.tare();
  FORCE_LEFT.get_units(10);
  FORCE_LEFT.set_scale();
  ledDelayBlink(1000, 5);

  //average of force is value 
  force_prev_value_left = FORCE_LEFT.read_average(20);
  force_prev_value_right = FORCE_RIGHT.read_average(20);
  force_prev_value_avg = (force_prev_value_left/1000 + force_prev_value_right/100)/2;
  ledDelayBlink(500, 20);
  */   
}

//*** Main Loop*** 
void loop() {
  // Communication with Web-Server
  recvOneChar();
  if  (Program == 1) {
    openOS();
  } 
 
  if  (Program == 2) {
    closeOS();
  } 
  
  else if (Program == 3) {
    //adaptiveWS();
  }
  
  else if (Program == 4) {
    configuredWS();
  }
  /*
  // THIS HERE IS JUST FOR TESTING AND CAUSES EXTREM JITTER IN ALL SERVOS --> Trying out other HX711 library next to fix jitter
  force_prev_value_left = FORCE_LEFT.read();
  force_prev_value_right = FORCE_RIGHT.read();
  Serial.print(force_prev_value_left);
  Serial.print(" ");
  Serial.println(force_prev_value_right);
  */
}
   

// Communication with Web-Server
void recvOneChar() {
  char recievedChar;
  if (Serial.available() > 0) {
    recievedChar = Serial.read();
    Program = recievedChar;
  }
}


// *** PROGRAMS ***
void openOS() {  
  // postioning of ws servos
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);
  
  // positioning of os servos following a sine accelaration curve
  if ( os_pos_right == OS_POS_CLOSED_RIGHT && os_pos_left == OS_POS_CLOSED_LEFT) {
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + SERVO_TIMEOUT){
        os_pos_right = OS_POS_CLOSED_RIGHT + (OS_MOTION_RANGE * sin((PI/162) * mister_t)); 
        os_pos_left = OS_POS_CLOSED_LEFT -(OS_MOTION_RANGE * sin((PI/162) * mister_t)); 
        OS_RIGHT.writeMicroseconds(os_pos_right);                                     
        OS_LEFT.writeMicroseconds(os_pos_left); 
        delay(SERVO_TIMEOUT);  
    }
  } 
}  

void closeOS() {
  // postioning of ws servos
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);

  // positioning of os servos following a sine accelaration curve
  if ( os_pos_right == OS_POS_OPEN_RIGHT && os_pos_left == OS_POS_OPEN_LEFT){
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + 3){
        os_pos_right = OS_POS_OPEN_RIGHT -(OS_MOTION_RANGE * sin((PI/162) * mister_t));
        os_pos_left = OS_POS_OPEN_LEFT + (OS_MOTION_RANGE * sin((PI/162) * mister_t));        
        OS_RIGHT.writeMicroseconds(os_pos_right);
        OS_LEFT.writeMicroseconds(os_pos_left);
        delay(SERVO_TIMEOUT);
    }
  }
}

void configuredWS() {
  // postioning of os servos
  OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);

  // positioning of ws servos following a sine accelaration curve
  for (mister_t = 0; mister_t <= 144; mister_t = mister_t + 3){
      ws_pos_right = (int) (ws_pos_ext_right + (WS_MOTION_RANGE * sin((PI/144) * mister_t)));
      ws_pos_left = (int) (ws_pos_ext_left - (WS_MOTION_RANGE * sin((PI/144) * mister_t))); 
      WS_RIGHT.writeMicroseconds(ws_pos_right);      
      WS_LEFT.writeMicroseconds(ws_pos_left);
      delay(SERVO_TIMEOUT);
  }   
}


void adaptiveWS() {
  /*
   * OLD FUNCTION THAZ DOES NOT WORK PROPERLY WITH NEW HARDWARE -- IGNORE
   */
  // postioning of os servos
  OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT); 
  
  // read force values
  force_value_left = FORCE_LEFT.read(); 
  force_value_right = FORCE_RIGHT.read();

  //Berechnen des gleitenden Durchnschnitts (Mit Offset)
  force_value_avg = (force_value_left/1000 + force_value_right/100)/2;

  //Aktueller Durchnitt größer als der davor gemessene
  if (force_value_avg > force_prev_value_avg) { 
    //Positionierung der ÖS-Motoren WARUM?
    //OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    //OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);
    
    //Anpassen der WS-Position
    ws_pos_right = ws_pos_right + WS_STEP;
    ws_pos_left = ws_pos_left - WS_STEP;     
    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);

    //Überschreiben des gleitenden Durchschnitts
    force_prev_value_avg = force_value_avg;

    //Wartezeit, bis der nächste FORCE-Wert verfügbar wird    
    delay(100);
  }

  //Aktueller Durchnitt kleiner als der davor gemessene
  else if (force_value_avg < force_prev_value_avg) { 

    //Positionierung der ÖS-Motoren
    OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);
      
    //Anpassen der WS-Position
    ws_pos_right = ws_pos_right - WS_STEP;
    ws_pos_left = ws_pos_left + WS_STEP;

    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);
     
    //Überschreiben des gleitenden Durchschnitts
    force_prev_value_avg = force_value_avg;

    //Wartezeit, bis der nächste FORCE-Wert verfügbar wird WARUM
    delay(100);
  }
  
  //Aktueller Durchnitt gleich dem davor gemessenen
  else if (force_value_avg == force_prev_value_avg) { 

    //Positionierung der ÖS-Motoren
    OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);

    //Erhalten der WS-Position
    ws_pos_right = ws_pos_right;
    ws_pos_left = ws_pos_left;     

    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);

    //Überschreiben des gleitenden Durchschnitts
    force_prev_value_avg = force_value_avg;

    //Wartezeit, bis der nächste FORCE-Wert verfügbar wird    
    delay(100);
  }
}

void ledDelayBlink(int delay_time, int blink_times){
  for (int i = 0; i < blink_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time/(blink_times*2));
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time/(blink_times*2));
  }         
}