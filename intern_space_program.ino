#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define ADC_REF 5 //5v

// PINS
#define ROTARY_ANGLE_SENSOR A0 //Analog pin 0
#define BOOSTER_IIST        A6 //TODO update to reflect actual pin

// STATE THRESHOLD PARAMETERS //TODO tune all
#define LAUNCH_ACCEL     980.0 // m/s^2, the forward acceleration at which launch definitely happened
#define SEP_ACCEL       1200.0 // m/s^2, the forward acceleration at which separation definitely happened
#define MIN_SEP_ALTITUDE 152.4 // meters, 500 ft, minumum altitude at which separation would possibly happen

// OTHER
#define GROVE_VCC  5 //VCC of the grove interface is normally 5v
#define FULL_ANGLE 300 //full value of the rotary angle is 300 degrees
#define LOCK   1 //mutux lock
#define UNLOCK 0 //mutux unlock

const int colorR = 255;    //red
const int colorG = 0;      //green
const int colorB = 0;      //blue


Pixy pixy; // This is the main Pixy object

// IIST Sensors
bool boosterIIST = true;

// SERVOS

const int servoLeftPin = 9;      //digital pin 9
Servo servoLeft;          //create servo
int servoLeftAngle = 0;   // servo position in degrees

const int servoRightPin = 10;   //digital pin 10
Servo servoRight;          //create servo
int servoRightAngle = 0;    //servo position in degrees

const int servoBackPin = 11;   //digital pin 10
Servo servoBack;          //create servo
int servoBackAngle = 0;    //servo position in degrees

// MPU

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

    Serial.begin(115200); //baud rate needs to be agreed upon
    pi_go();
    initialize();
    diagnostic();
}

void loop() {
  // put your main code here, to run repeatedly:
  //collect_data();

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
    case 0: //0 SCRUB
      if (AUTOSTATE){
        // SCRUB never automatically changes to another state
        // the Diagnostics() function has authority to change state to GROUND_READY
      }
      break;
    case 1: //1 GROUND_READY
      if (AUTOSTATE){

        // Sense Launch Sensor Fusion
        if(false/*accelForward>LAUNCH_ACCEL && altitude>ALTIMETER_UNCERTAINTY*/){ //TODO
            STATE = 2; //BOOST

        }
      }
      break;
    case 2: //2 BOOST
      if (AUTOSTATE){
        // IIST sensor
        boosterIIST = digitalRead(BOOSTER_IIST);

        // Sense Ejection Sensor Fusion
        if(abs(accelForward)>SEP_ACCEL && altitude>MIN_SEP_ALTITUDE){
            if(boosterIIST)
            {
                STATE = 3; // SEPARATE
            }
        }

        // Timer Watchdog
        //TODO make this

      }
      break;
    case 3: //3 SEPARATE
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 4: //4 STABILIZE
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 5: //5 SEARCH
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 6: //6 ORBIT
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 7: //7 RECOVER_ABORT
      if (AUTOSTATE){
        //make state change
      }
      break;
    case 8: //8 LANDED
      if (AUTOSTATE){
        //make state change
      }
      break;
    default:

  }
}

void mutux(int lock){
  // Used to change whether or not to allow interrupts
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
        attachInterrupt(0, dmpDataReady, RISING);
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
  //PI to nano handshake

  // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //PI to Nano Handshake
  // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
}

void dmpDataReady() {
    mpuInterrupt = true;
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
