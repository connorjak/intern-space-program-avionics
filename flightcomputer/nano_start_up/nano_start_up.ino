#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "Pixy.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <RH_RF95.h>

// *** PINS ***
#define BOOSTER_IIST              A6 //Analog  pin 6  //TODO:800 update to reflect actual pin
#define I2C_SCL                   -1 //Digital pin #  //TODO:810 update to reflect actual pin
#define I2C_SDA                   -1 //Digital pin #  //TODO:820 update to reflect actual pin
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
  //pinMode(4, OUTPUT);
  pi_go();
  initialize();
  diagnostic();
}

void loop() {
  // put your main code here, to run repeatedly:
 //digitalWrite(4, HIGH);
 //delay(1000);
 //digitalWrite(4,LOW);
 //delay(1000);
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
  attachInterrupt(digitalPinToInterrupt(TRX_INT_PIN), transInterrupt, RISING);
  //TODO:390 communications setup

  //MPU Setup
    mpu.initialize();
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity //TODO:760 tune these?
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
        //TODO:140 REPORT ERROR HERE OVER TRANSCIVER
    }
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  //End MPU Setup

  //Servo Setup
  servoLeft.attach(SERVO_LEFT);
  servoRight.attach(SERVO_RIGHT);
  servoBack.attach(SERVO_BACK);

  //Pixy Initialization
  pixy.init();
}

bool diagnostic(){
  int error_sum = 0;
  // int error_sum is used as a bitset, where, when read as a binary number, it
  // tells you exactly where your errors were, no matter the combination.

  //Tranciever Handshake

  // *** PI to nano handshake
  // send "handshake" to pi and wait to recieve "sure"
  // if sure not recieved, wait 100 ms then send again
  // resend 3 times before throwing error
  int packet_loss = 0;
  for(packet_loss; packet_loss < 3; packet_loss += 1){
    Serial.println("handshake"); //sends impetus for handshake
    Serial.flush(); //waits for outgoing stream to complete
    while(!Serial.available());
    if (Serial.find("sure")){ //breaks from loop if 'sure' is found
      break;
    }
  }
  //3 errors occured
  if (packet_loss == 3){
    //OUTPUT to TRANSCIEVER ERROR
    error_sum = error_sum + 1; //1s place
  }

  //MPU check
  // verify MPU connection
  if(!mpu.testConnection()){
    ////OUTPUT to TRANSCIEVER ERROR
    error_sum = error_sum + 2; //2s place
  }
  //Verify MPU acc and gryo readings

  //Servo 'Dance'
  servo_sweep();
  if (servoRightAngle != servoRight.read()){ //TODO:350 can we actually read from these servos?
    //TODO:70 OUTPUT TO TRANSCIEVER: ERROR right servo: servoRight.read() angle off by (servoRight.read() - SERVO_CENTER)
    error_sum = error_sum + 4; //4s place
  }
  if (servoLeftAngle != servoLeft.read()){
    //TODO:50 OUTPUT TO TRANSCIEVER: ERROR left servo: servoLeft.read() angle off by (servoLeft.read() - SERVO_CENTER)
    error_sum = error_sum + 8; //8s place
  }
  if (servoBackAngle != servoBack.read()){
    //TODO:30 OUTPUT TO TRANSCIEVER: ERROR back servo: servoBack.read() angle off by (servoBack.read() - SERVO_CENTER)
    error_sum = error_sum + 16; //16s place
  }
  
  //PIXY verification
  
  
  //Final Error Output
  if (error_sum == 0){
    //TODO:110 OUTPUT to TRANSCIEVER: Packet Loss was (packet_loss). No errors, all clear. READY TO LAUNCH!
    return true;
  }
  else{
    //TODO:90 OUTPUT to TRANSCIEVER: Packet Loss was (packet_loss). Error code: (error_sum in binary). SCRUB!
    return false;
  }
  return false;
}

