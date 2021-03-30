/*
 * Author: Florian Geissler
 * Usecase: Test Bench for KSA system of Mechatronic Gripper
 * Last change: 25.03.2021
 */

const int DIR_PIN = 2;  // pinout for tb6600 motor driver
const int STEP_PIN = 3; // pinout for tb6600 motor driver
const int START_SIGNAL = 7; // pinout octocoupler
const int START_SWITCH = 8; // pinout manual switch

// set to your needs
const double moverspeed = 500; // in mm/s
const double ksa_way_length = 142; // in mm
double ksa_time = (ksa_way_length/moverspeed)*1000; // ksa time in milliseconds
double ff = 1/(ksa_time*1000);  //frequency

const double ksa_amplitude = 8.4; // in mm
const int steps_per_rev = 3200;
const double diameter_roll = 12; // in mm
const long amplitude_steps = (long)(ksa_amplitude*(steps_per_rev/(diameter_roll*PI))); // step amplitude

void setup()
{
    Serial.begin(115200);
    pinMode(DIR_PIN,OUTPUT);
    pinMode(STEP_PIN,OUTPUT);
    pinMode(START_SIGNAL,INPUT_PULLUP);
    pinMode(START_SWITCH,INPUT_PULLUP);
    Serial.println(amplitude_steps);
}

void loop(){  
  
  static long steps_should; 
  static long steps_now = 0; 
  static long steps_should_before = 0;
  static long steps_total = 0;
  static const int speed_ = 50;  // motor and motor driver specific
  static unsigned long t = 0;
  static unsigned long t_start = 100000000000000000;  // big value so it doesnt start by itself on boot, gets resetted anyway
  
  if (!digitalRead(START_SIGNAL)||!digitalRead(START_SWITCH)) {  // internal pullup resistor inverts logic
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
