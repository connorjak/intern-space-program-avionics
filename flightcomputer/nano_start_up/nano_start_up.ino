#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "Pixy.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <RH_RF95.h>

// *** PINS ***
#define BOOSTER_IIST              A6 //Analog  pin 6  //TODO update to reflect actual pin
#define I2C_SCL                   -1 //Digital pin #  //TODO update to reflect actual pin
#define I2C_SDA                   -1 //Digital pin #  //TODO update to reflect actual pin
#define SERVO_LEFT                 9 //Digital pin 9
#define SERVO_RIGHT               10 //Digital pin 10
#define SERVO_BACK                11 //Digital pin 10
#define MPU_INT_PIN                2 //MPU interrupt pin
#define TRX_INT_PIN                3 //tranciever interrupt pin

// *** OTHER ***
#define LOCK                       1 //mutux lock
#define UNLOCK                     0 //mutux unlock
#define SETUP_WAIT_TIME         1000 //milliseconds
#define BAUD_RATE               9600 //agreed baud rate
#define SERVO_CENTER              90 // Starting Angle
#define SERVO_MAX                130 // Artifical max for servos (mech. limits TBD)
#define SERVO_MIN                 50 // Artifical min for servos (mech. limits TBD)

// *** VARIABLES ***************************************************************

Pixy pixy; // This is the main Pixy object

// *** IIST Sensors ***
bool boosterIIST = true;

// *** SERVOS ***
Servo servoLeft;          //create servo
int servoLeftAngle = SERVO_CENTER;   // servo position in degrees

Servo servoRight;          //create servo
int servoRightAngle = SERVO_CENTER;    //servo position in degrees

Servo servoBack;          //create servo
int servoBackAngle = SERVO_CENTER;    //servo position in degrees


// *** MPU ***

MPU6050 mpu; //mpu object
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

// *** TRX ***
volatile bool trxInterrupt = false;  // indicates whether TRX interrupt pin has gone high
#define RFM95_CS 16
#define RFM95_RST 5
#define RFM95_INT 4
#define RF95_FREQ 915.0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE); //baud rate needs to be agreed upon
  //driver.init() is Transceiver setup
  pinMode(4, OUTPUT);
  pi_go();
  initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(4, HIGH);
 delay(1000);
 digitalWrite(4,LOW);
 delay(1000);
}
