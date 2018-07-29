#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <string.h>


#define ADC_REF 5 //5v
#define ROTARY_ANGLE_SENSOR A0 //Analog pin 0
#define GROVE_VCC 5 //VCC of the grove interface is normally 5v
#define FULL_ANGLE 300 //full value of the rotary angle is 300 degrees
#define LOCK 1 //mutux lock
#define UNLOCK 0 //mutux unlock
#define BAUD_RATE 115200 //agreed baud rate
#define MPU_INT_PIN 2 //MPU interrupt pin
#define TRX_INT_PIN 3 //tranciever interrupt pin

const int colorR = 255;    //red
const int colorG = 0;      //green
const int colorB = 0;      //blue


Pixy pixy; // This is the main Pixy object

int servoLeftPin = 9;      //digital pin 9
Servo servoLeft;          //create servo
int servoLeftAngle = 0;   // servo position in degrees

int servoRightPin = 10;   //digital pin 10
Servo servoRight;          //create servo
int servoRightAngle = 0;    //servo position in degrees

int servoBackPin = 11;   //digital pin 10
Servo servoBack;          //create servo
int servoBackAngle = 0;    //servo position in degrees

MPU6050 mpu; //mpu object
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int STATE;
bool AUTOSTATE;

void setup() {
  //pi_go();
  //    wait for pi approval
  //    When character recieved from pi, continue
  //initialize();
  //    Initialize all peripherals and communications
  //    MPU setup
  //    Pixy Setup
  //    SPI and I2C setup
  //diagnostic();
  //    Check and diagnose crucial operations
  //    sensor sanity check
  //    servo check
  //    Pixy check
  //    commincation_handshake();
  //         arduino to pi check; transciever check;
  //Set_state_based_on_pass_fail();

    Serial.begin(BAUD_RATE); //baud rate needs to be agreed upon
    pi_go();
    initialize();
    diagnostic();
}

void loop() {
  // put your main code here, to run repeatedly:
loop_start: collect_data();

  switch(STATE){
  //0 SCRUB
  //1 GROUND_READY
  //2 BOOST
  //3 SEPARATE
  //4 STABILIZE
  //5 SEARCH
  //6 ORBIT
  //7 RECOVER_ABORT
  //8 LANDED
    case 0:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 1:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 2:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 3:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 4:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 5:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 6:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 7:
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 8:
      if (AUTOSTATE){
        //make state change
      }
      break;
    default:
  }
}

void mutux(int lock){
  //used for interrupt-dependent functions
  if (lock == 1){
    noInterrupts();
  }
  if (lock == 0){
    interrupts();
  }
}

void pi_go(){
  // wait for ready
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
}
void initialize(){
  //    MPU setup
  //    Pixy Setup
  //    SPI and I2C setup

  //Transciever Setup
  //enable interrupt attach
  attachInterrupt(digitalPinToInterrupt(digitalPinToInterrupt(TRX_INT_PIN)), trans_interrupt, RISING);
  //    communications setup
  
  //MPU Setup
    mpu.initialize();
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        //REPORT ERROR HERE OVER TRANSCIVER
    }
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Servo Setup
  pinMode(ROTARY_ANGLE_SENSOR, INPUT);
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  servoBack.attach(servoBackPin);

  //Pixy Initialization
  pixy.init();
}

void diagnostic(){
  int error_sum = 0;
  
  //Tranciever Handshake
  
  //PI to nano handshake
  //send "handshake" to pi and wait to recieve "sure"
  //if sure not recieved, wait 100 ms then send again
  //resend 3 times before throwing error\
  int packet_loss = 0;
  char target[4] = "sure"
  Serial.println("handshake"); //sends impetus for handshake
  Serial.flush(); //waits for outgoing stream to complete
  while (packet_loss < 3){
    //breaks from loop if 'sure' is found
    if (Serial.find(target)){
      break;
    }
    packet_loss = packet_loss + 1;
  }
  //3 errors occured
  if (packet_loss == 3){
    //OUTPUT to TRANSCIEVER ERROR
    error_sum = error_sum + 1;
  }
  
  //MPU check 
  // verify MPU connection
  if(!mpu.testConnection()){
    ////OUTPUT to TRANSCIEVER ERROR
    error_sum = error_sum + 1;
  }
  //Verify MPU acc and gryo readings
    
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void trans_interrupt(){
  //get transciver data
  STATE = sent_string
  goto loop_start;
}
/*
ARDUINO GOTO ---

label:

goto label; // sends program flow to the label

Example Code

for(byte r = 0; r < 255; r++){
    for(byte g = 255; g > 0; g--){
        for(byte b = 0; b < 255; b++){
            if (analogRead(0) > 250){ goto bailout;}
            // more statements ...
        }
    }
}

bailout:*/
