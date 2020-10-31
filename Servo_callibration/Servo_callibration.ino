
/*
 *SERVO CALLIBRATION: 
 * Controlling a servo position using a potentiometer (variable resistor) and print postion is serial monitor.
 * Move your servo with potentiometer in desired spot and write down the pwm signal (SERVO_VALUE) for your later use in programs.
 */

#include <Servo.h>  

Servo TEST_SERVO;   

const byte TEST_SERVO_PIN = 6;  
const byte POT_PIN = A0;        

int POT_VALUE;             
int SERVO_VALUE;                

void setup() {
  TEST_SERVO.attach(TEST_SERVO_PIN); 
  Serial.begin(57600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  POT_VALUE = analogRead(POT_PIN);
  SERVO_VALUE = map(POT_VALUE, 0, 1023, 0, 4000);      // scale it to use it with the servo (value between 1000 and 2000 --> depends on servo motion range);
  TEST_SERVO.writeMicroseconds(SERVO_VALUE);
  Serial.print(POT_VALUE);
  Serial.print(" ; ");
  Serial.println(SERVO_VALUE);
  delay(15);                                           // waits for the servo to get there
}
