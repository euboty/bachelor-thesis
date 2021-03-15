
/*
 * Florian Geissler 2020 
 * Controlling multipe servos using w,s and q on the keyboard of your pc via serial communication 
 * with PuTTY or any other serial terminal. Print position on serial monitor.
 */

/*
   FOR THIS PARTICULAR POJECT ONLY:
   Perspective is mover to foil:
   Os_Right (OS Right)
    open 112
    closed 53
   Os_Left (OS Left)
    open 47
    closed 106
   //DEEGREES FOR WS
   Ws_Left (WS Left)
    extended 135
    pull 33
    middle 86
   Ws_Right (WS Right)
    extended 43
    pull 148
    middle 88
*/
 
#include <ServoEasing.h>  

// Fill to your needs:
const String SERVO_NAMES[] = {"OS_LEFT", "OS_RIGHT", "WS_LEFT", "WS_RIGHT"};
const int SERVO_PINOUT[] = {26, 27, 32, 33};
const int ARRAY_LENGTH = 4;
const double MAX_SERVO_RANGE[] = {360, 360, 360, 360};
const double SERVO_STARTING_POS[] = {50, 50, 50, 50};             // values for this particular project, yours might be different

char servo_pointer_char;
int servo_pointer;

char servo_direction;
double servo_pos[ARRAY_LENGTH];
double servo_pos_prev[ARRAY_LENGTH];

Servo SERVO[ARRAY_LENGTH];

void setup() {
    Serial.begin(57600); 
 
    memcpy(servo_pos, SERVO_STARTING_POS, sizeof(SERVO_STARTING_POS));        //copy array
    memcpy(servo_pos_prev, servo_pos, sizeof(servo_pos));                     //copy array
 
  // attach all servos
  for (int i = 0; i < ARRAY_LENGTH; i++){
    SERVO[i].attach(SERVO_PINOUT[i]);
  }
  
  while (Serial.available() == 0) {
    Serial.println("SERVO CALIBRATION PROGRAM. Press any key to start!");
    delay(1000);  
  }
  while (Serial.available() > 0) {
    serialFlush();                                                               //delete serial buffer
  }
}

void loop() {
  chooseServo();
  calibrateServo();
}

void chooseServo() {
  /*
   * Serial dialog. Whole pupose is to get var: servo_pointer
   */
  Serial.println();
  Serial.println("To control servos later on: press w and s to move servo, q to quit...");
  Serial.println("What servo do you want to calibrate? Your options:");
  
  for (int i = 0; i < ARRAY_LENGTH; i++){
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(SERVO_NAMES[i]);
  }
  
  Serial.println();
  Serial.println("Type the desired servo number:");
  while (Serial.available() == 0) {};                   // wait till user enters servo number
  servo_pointer_char = Serial.read();
  servo_pointer = servo_pointer_char - '0';                // putty sends as ASCII char number. To get back we have to subtract '0', works for 0-9, (reference: ASCII table)
  Serial.println(servo_pointer);
  Serial.println();
  
  if (servo_pointer < ARRAY_LENGTH && servo_pointer >= 0){
    Serial.print("You chose ");
    Serial.println(SERVO_NAMES[servo_pointer]);
  }
  else{
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("Please enter valid servo name: ");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    chooseServo();
  }
  Serial.println();
}

void calibrateServo(){
  /*
   * Does the movement and serial control with keyboard.
   */
  while(1){
    servo_direction = Serial.read();
    if (servo_direction == 'w'){
      if (servo_pos[servo_pointer] < MAX_SERVO_RANGE[servo_pointer]){
        servo_pos[servo_pointer] += 1;
      }  
    }
    else if (servo_direction == 's'){
      if (servo_pos[servo_pointer] > 0){
        servo_pos[servo_pointer] -= 1;  
      }
    }
    else if (servo_direction == 'q'){
      break;
    }
    Serial.println(servo_pos[servo_pointer]);
    if (servo_pos[servo_pointer] != servo_pos_prev[servo_pointer]){
      SERVO[servo_pointer].write(servo_pos[servo_pointer]);
      servo_pos_prev[servo_pointer] = servo_pos[servo_pointer];
    }
    delay(1);
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   
