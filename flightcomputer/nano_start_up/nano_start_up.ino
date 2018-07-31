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
#define TRX_POWER                 23 //output power (5-23 dBm)
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
#define RFM95_CS                  16 //UNKNOWN WHAT DIS
#define RFM95_RST                  5 //UNKNOWN WHAT DIS (DIGITAL DATA PIN?)
#define RFM95_INT                  4 //UNKNOWN WHAT DIS (INTERRUPT PIN?)
#define RF95_FREQ              915.0 //Transmit Frequency
#define TRX_POWER                 23 //output power (5-23 dBm)
#define CLIENT_ADDRESS             1
#define SERVER_ADDRESS             2
RH_RF95 rf95(RFM95_CS, RFM95_INT);    //Radio Driver Object
RHReliableDatagram manager(rf95, CLIENT_ADDRESS); //Radio Manager Object

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE); //baud rate needs to be agreed upon
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

void transmit_data(char* data){ // Send a message to rf95_server
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)
    if(manager.recvfromAckTimeout(buf, &len, 2000, &from)){
      //Sent successful, acknowledge recieved
      //Serial.print("got reply from : 0x"); Serial.print(from, HEX);Serial.print(": "); Serial.println((char*)buf);
    }
    else{
      //No acknowledge
     // Serial.println("No reply, is rf95_reliable_datagram_server running?");
    }
}

char recieve_data(){
  //TODO put recieve code here
}
void initialize(){
  
  //Transciever Setup
  //enable interrupt attach
  attachInterrupt(digitalPinToInterrupt(TRX_INT_PIN), transInterrupt, RISING);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  rf95.init(); //radio initialization
  rf95.setFrequency(RF95_FREQ) //set frequency
  rf95.setTxPower(TRX_POWER, false); //set output power

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
        transmit_data("ERROR: IMU Initialization Failed\n");
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
  //error_sum = (error_sum << 1) + 1; pushes 1 bit onto string of bits for error
  //error_sum = (error_sum << 1);  pushes 0 bit onto string of bits for no error
  //^^^ this method allows for easy addition of error checking protocols without hard coding the decimal values
  //and changing of order in code does not affect


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
    transmit_data("ERROR: PI to NANO Packet Loss\n");
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1); 
  }

  //MPU check
  // verify MPU connection
  if(!mpu.testConnection()){
    transmit_data("ERROR: IMU Connection Issues\n");
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1); 
  }
  //Verify MPU acc and gryo readings

  //Servo 'Dance'
  servo_sweep();
  if (servoRightAngle != servoRight.read()){ //TODO:350 can we actually read from these servos?
    transmit_data("ERROR: Right Servo Angle Error\n");
    //TODO:70 OUTPUT TO TRANSCIEVER: ERROR right servo: servoRight.read() angle off by (servoRight.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1); 
  }
  if (servoLeftAngle != servoLeft.read()){
    transmit_data("ERROR: Left Servo Angle Error\n");
    //TODO:50 OUTPUT TO TRANSCIEVER: ERROR left servo: servoLeft.read() angle off by (servoLeft.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1); 
  }
  if (servoBackAngle != servoBack.read()){
    transmit_data("ERROR: Back Servo Angle Error\n");
    //TODO:30 OUTPUT TO TRANSCIEVER: ERROR back servo: servoBack.read() angle off by (servoBack.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1); 
  }
  
  //TODO: PIXY verification
  
  
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

