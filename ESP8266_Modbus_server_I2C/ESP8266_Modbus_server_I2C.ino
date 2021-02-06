#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ModbusIP_ESP8266.h>     //https://github.com/emelianov/modbus-esp8266
#include <Wire.h>
#include <PolledTimeout.h>

// I2C Pins
#define SDA_PIN 4
#define SCL_PIN 5

//I2C Adresses
const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x08;

// WIFI
WiFiManager wifiManager;

// Modbus Registers Offsets

// Coil (READ&WRITE)
// none

// Discrete Input (READ)
const int SETUP_FAILURE = 0;
const int OS_ENDSWITCH_LEFT = 1;
const int OS_ENDSWITCH_RIGHT = 2;

// Holding Register (READ&WRITE)
const int MODE = 0;
const int FORCE_TARGET = 1;
const int POSITION_WS = 2;
const int P = 3;
const int I = 4;
const int D = 5;
const int MAX_INFLUENCE_CONTROLLER = 6; // how much influence can the PID Controller take on WS position in %

// Input Register (READ)
const int FORCE_LEFT_REAL = 0;
const int FORCE_RIGHT_REAL = 1;

// Modbus Receiving Buffers
byte setup_failure;
byte os_endswitch_left;
byte os_endswitch_right;
short force_left_real;
short force_right_real;

//I2C
byte payload[2];  // array used in int2bytes has to be global

// Modbus object
ModbusIP mb;

void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("Establishing wifi connection");
  wifiManager.autoConnect("KontiBat_Greifer_WlanConfig");   //access @ 192.168.4.1

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("Connected!");
  Serial.println();
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();  // start modbus server

  // Discrete Input - bool - read only
  mb.addIsts(SETUP_FAILURE);
  mb.addIsts(OS_ENDSWITCH_LEFT);
  mb.addIsts(OS_ENDSWITCH_RIGHT);

  // Holding Register - int - read and write
  mb.addHreg(MODE);
  mb.addHreg(FORCE_TARGET);
  mb.addHreg(POSITION_WS);
  mb.addHreg(P);
  mb.addHreg(I);
  mb.addHreg(D);
  mb.addHreg(MAX_INFLUENCE_CONTROLLER );

  // Input Register - int - read only
  mb.addIreg(FORCE_LEFT_REAL);
  mb.addIreg(FORCE_RIGHT_REAL);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER); // initialzie I2C connection to slave
}

void loop() {

  mb.task();  // call once inside loop - all modbus magic here
  
  //I2C
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(5); // i2c polling intervall to Arduino Nano

  if (nextPing) {
    i2c_communication();
  }

  debug_print_serial();
}

void i2c_communication() {
  // send to i2c slave
  Wire.beginTransmission(I2C_SLAVE); // transmit to device #8
  Wire.write((byte)mb.Hreg(MODE));
  int2bytes(mb.Hreg(FORCE_TARGET));
  Wire.write(payload , 2);
  int2bytes(mb.Hreg(POSITION_WS));
  Wire.write(payload , 2);
  int2bytes(mb.Hreg(P));
  Wire.write(payload , 2);
  int2bytes(mb.Hreg(I));
  Wire.write(payload , 2);
  int2bytes(mb.Hreg(D));
  Wire.write(payload , 2);
  int2bytes(mb.Hreg(MAX_INFLUENCE_CONTROLLER));
  Wire.write(payload , 2);
  Wire.endTransmission();    // stop transmitting

  // receive from i2c slave
  Wire.requestFrom(I2C_SLAVE, 7);    // request 7 bytes (3 bytes and 2 ints (which is 4 bytes)) from slave device #8
  setup_failure = Wire.read();
  mb.Ists(SETUP_FAILURE, setup_failure);
  os_endswitch_left = Wire.read();
  mb.Ists(OS_ENDSWITCH_LEFT, os_endswitch_left);
  os_endswitch_right = Wire.read();
  mb.Ists(OS_ENDSWITCH_RIGHT, os_endswitch_right);

  byte buffer_[2];
  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  force_left_real = ((short)(buffer_[0]) << 8) + buffer_[1];
  mb.Ireg(FORCE_LEFT_REAL, force_left_real);
  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  force_right_real = ((short)(buffer_[0]) << 8) + buffer_[1];
  mb.Ireg(FORCE_RIGHT_REAL, force_right_real);
}

void int2bytes(int split_me) {
  /*
     Splits int into an array of bytes which is defined out of this function,
     since C++ does not support return of arrays.
  */
  payload[0] = highByte(split_me);
  payload[1] = lowByte(split_me);
}

void debug_print_serial() {
  Serial.print("I2C IN: ");
  Serial.print(setup_failure);
  Serial.print(" ; ");
  Serial.print(os_endswitch_left);
  Serial.print(" ; ");
  Serial.print(os_endswitch_right);
  Serial.print(" ; ");
  Serial.print(force_left_real);
  Serial.print(" ; ");
  Serial.println(force_right_real);
}
