/*
 * Author: Florian Geissler
 * Usecase: Test Bench for KSA system of Mechatronic Gripper
 * Last change: 13.03.2021
 */

const int DIR_PIN = 2;  // pinout for tb6600 motor driver
const int STEP_PIN = 3; // pinout for tb6600 motor driver
const int START_SIGNAL = 7; // pinout octocoupler

long steps_should; 
long steps_now = 0; 
long steps_should_before = 0;
long steps_total = 0;
const int speed_ = 50;  // motor and motor driver specific
unsigned long t = 0;
unsigned long t_start = 0;

const long amplitude_steps = 730; // amplitude steps for sine
double ksa_time = 200; // ksa time in milliseconds
double ff = 1/(ksa_time*1000);  //frequency

void setup()
{
    Serial.begin(115200);
    pinMode(DIR_PIN,OUTPUT);
    pinMode(STEP_PIN,OUTPUT);
    pinMode(START_SIGNAL,INPUT_PULLUP);
}

void loop(){  
  
  if (!digitalRead(START_SIGNAL)) {  // internal pullup resistor inverts logic
    t_start = micros();
  }
  
  t = micros(); 
  
  if ((t-t_start)<(ksa_time*1000)){
    steps_total = 0;
    while(steps_total < amplitude_steps*2){   // ksa time in micros, just do one iteration
      steps_should=long(amplitude_steps*sin((PI*ff)*(t-t_start)));   // position curve of motor shaft
      Serial.println(steps_should);

      if(steps_should>steps_should_before){  // direction forwards
          digitalWrite(DIR_PIN,LOW);
  
          while(steps_now<steps_should)
          {
              digitalWrite(STEP_PIN,LOW);
              delayMicroseconds(10);
              digitalWrite(STEP_PIN,HIGH);
              delayMicroseconds(speed_);
              steps_now +=1;
              steps_total +=1;
          }
          steps_should_before=steps_should;
  
      } else if (steps_should<steps_should_before){  // direction backwards                              
          digitalWrite(DIR_PIN,HIGH);
  
          while(steps_now>steps_should)
          {
              digitalWrite(STEP_PIN,LOW);
              delayMicroseconds(10);
              digitalWrite(STEP_PIN,HIGH);
              delayMicroseconds(speed_);
              steps_now -=1;
              steps_total +=1;
          }
          steps_should_before=steps_should;
      } 
    t = micros(); 
    }
    t_start = 0;
  }
}
