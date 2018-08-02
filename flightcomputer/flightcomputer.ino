
 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>  
#include <Pixy.h>
#include<Wire.h>
#include <Servo.h>
 
 
 
//#include "MPU6050.h" // not necessary if using MotionApps include file
 
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
 
 
 
//int modecounter = 0;       //different modes display different data on LCD
 
 
Pixy pixy; // This is the main Pixy object
int pixyValue = 0;
float pixyResponse = 0;
float anglefrompixypixels = 0.0;
 
int servoLeftPin = 6;      //digital pin 6
Servo servoLeft;          //create servo
int servoLeftAngle = 0;   // servo position in degrees

int servoRightPin = 5;   //digital pin 5
Servo servoRight;          //create servo
int servoRightAngle = 0;    //servo position in degrees

int servoBackPin = 9;      //digital pin 9
Servo servoBack;          //create servo
int servoBackAngle = 0;    //servo position in degrees
 
 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
 
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

 
// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
 
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL
 
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 

// *** SENSOR UNCERTAINTIES ***
#define ALTIMETER_UNCERTAINTY    1.0 //meters //TODO tune

#define PITCH_UNCERTAINTY        0.5 //degrees
#define ROLL_UNCERTAINTY         0.5 //degrees
#define YAW_UNCERTAINTY          0.5 //degrees

#define THERM_UNCERTAINTY        1.0 //degrees F


// *** STATE-SPECIFIC PARAMETERS *** //TODO tune all
//0 SCRUB
#define CANARD_DEFLECT_0         0.0 //degrees //TODO check this
#define ELEVATOR_DEFLECT_0       0.0 //degrees
#define AILERON_DEFLECT_0        0.0 //degrees
//1 GROUND_READY
#define CANARD_DEFLECT_1         9.0 //degrees //TODO check this
#define ELEVATOR_DEFLECT_1       0.0 //degrees
#define AILERON_DEFLECT_1        0.0 //degrees
#define ALTIMETER_GROUNDSAFE    15.0 //meters, the altitude at which launch definitely happened
#define LAUNCH_ACCEL            90.0 // m/s^2, the forward acceleration at which launch definitely happened
#define MINTIME_1                0.0 //seconds
#define MAXTIME_1          9000000.0 //seconds
//2 BOOST
#define CANARD_DEFLECT_2         9.0 //degrees //TODO check this
#define ELEVATOR_DEFLECT_2       0.0 //degrees
#define AILERON_DEFLECT_2        0.0 //degrees
#define SEP_ACCEL             1200.0 // m/s^2, the forward acceleration at which separation definitely happened
#define MIN_SEP_ALTITUDE       167.6 //meters, 549.9 ft, minumum altitude at which separation would possibly happen
#define MINTIME_2                0.0 //seconds
#define MAXTIME_2          9000000.0 //seconds
//3 SEPARATE
#define ELEVATOR_DEFLECT_3      -1.0 //degrees
#define AILERON_DEFLECT_3        0.0 //degrees
#define MINTIME_3                0.0 //seconds
#define MAXTIME_3          9000000.0 //seconds
//4 STABILIZE
#define DEFAULT_PITCH_GOAL_4   -20.0 //degrees
#define DEFAULT_ROLL_GOAL_4      0.0 //degrees
#define DEFAULT_YAW_GOAL_4       0.0 //degrees (NORTH)
#define MINTIME_4                0.0 //seconds
#define MAXTIME_4          9000000.0 //seconds
//5 SEARCH
#define DEFAULT_PITCH_GOAL_5   -10.0 //degrees
#define DEFAULT_ROLL_GOAL_5    -30.0 //degrees
#define DEFAULT_YAW_GOAL_5       0.0 //degrees (NORTH)
#define YAW_RATE_5              20.0 //degrees per second
#define MINTIME_5                0.0 //seconds
#define MAXTIME_5          9000000.0 //seconds
//6 ORBIT
#define DEFAULT_PITCH_GOAL_6   -10.0 //degrees
#define DEFAULT_ROLL_GOAL_6    -30.0 //degrees
#define DEFAULT_YAW_GOAL_6       0.0 //degrees (NORTH)
#define YAW_RATE_MIN_6           2.0 //degrees per second
#define YAW_RATE_MAX_6          40.0 //degrees per second
#define MINTIME_6                0.0 //seconds
#define MAXTIME_6          9000000.0 //seconds
//7 RECOVER_ABORT
#define PARACHUTE_ALTITUDE     167.6 //meters, 549.9 ft, altitude threshold to deploy parachute at
#define DEFAULT_PITCH_GOAL_7    10.0 //degrees
#define DEFAULT_ROLL_GOAL_7    -30.0 //degrees
#define DEFAULT_YAW_GOAL_7       0.0 //degrees (NORTH)
#define MINTIME_7                0.0 //seconds
#define MAXTIME_7          9000000.0 //seconds
//8 LANDED
#define TIME_BETWEEN_BEACON     10.0 //seconds, time between signal pulses (if we do an RF locator to assist in recovery)

// *** OTHER ***
#define LOCK                       1 //mutux lock
#define UNLOCK                     0 //mutux unlock
#define SETUP_WAIT_TIME         1000 //milliseconds
#define BAUD_RATE             115200 //agreed baud rate
#define SERVO_CENTER              90 //degrees, Starting Angle
#define SERVO_MAX                130 //degrees, Artifical max for servos (mech. limits TBD)
#define SERVO_MIN                 50 //degrees, Artifical min for servos (mech. limits TBD)

#define SERVO_AILERON_MULT       0.5 // multiply servo actuation angle by this to get true aileron actuation
#define SERVO_ELEVATOR_MULT      0.5 // multiply servo actuation angle by this to get true elevator actuation //TODO test, crucial to launch stability




// *** VARIABLES ***************************************************************

//Pixy pixy; // This is the main Pixy object

// *** IIST Sensors *** //TODO may exclude from glider
bool boosterIIST = true;

// *** SERVOS ***
//Servo servoLeft;          //create servo
//int servoLeftAngle = SERVO_CENTER;   // servo position in degrees
//
//Servo servoRight;          //create servo
//int servoRightAngle = SERVO_CENTER;    //servo position in degrees
//
//Servo servoBack;          //create servo
//int servoBackAngle = SERVO_CENTER;    //servo position in degrees


// *** MPU ***

//MPU6050 mpu; //mpu object
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//volatile bool mpuInterruptFlag = false;  // indicates whether MPU interrupt pin has gone high


//// *** TRX ***
//
//volatile bool trxInterruptFlag =   false;  // indicates whether TRX interrupt pin has gone high
//#define RFM95_CS                  16 //TODO UNKNOWN WHAT DIS
//#define RFM95_RST                  5 //TODO UNKNOWN WHAT DIS (DIGITAL DATA PIN?)
//#define RFM95_INT                  4 //TODO UNKNOWN WHAT DIS (INTERRUPT PIN?)
//#define RF95_FREQ              915.0 //Transmit Frequency
//#define TRX_POWER                 23 //output power (5-23 dBm)
//#define CLIENT_ADDRESS             1 //TODO set from Pi handshake?
//#define SERVER_ADDRESS             2 //TODO set from Pi handshake?
//RH_RF95 rf95(RFM95_CS, RFM95_INT);    //Radio Driver Object
//RHReliableDatagram manager(rf95, CLIENT_ADDRESS); //Radio Manager Object

//*** COMMUNICATIONS ***
char output_string[250];

// *** META FLIGHT VARIABLES ***
// These values are more reliable and isolated from the volatile control flight variables.
// Use these values when TX to ground station.

float currentPitch =          0.0; // degrees, pitch above the horizon
float currentRoll =           0.0; // degrees, roll left wing above horizon //TODO:400 coordinate system document
float currentYaw =            0.0; // degrees, compass reading from forward of glider

float altitude =          -1000.0; // meters
float altitudeRate =          0.0; // m/s

float accelForward =          0.0; // acceleration in forward direction, no gravity
float accelGliderFrame[3];         // acceleration in glider frame, no gravity
float accelWorldFrame[3];          // acceleration in world frame (x+ East, y+ North, z+ Up), yes gravity

// *** CONTROL FLIGHT VARIABLES ***
// These values are meant to be used for RX from ground station and set by autopilot
// Be cautious to use these values when TX to ground station.

volatile long millMET =            -300000; //milliseconds, mission time counter
volatile long millisMETOffset =          0; //milliseconds, Offset = millis when MET starts countdown

volatile float pitchGoal =       DEFAULT_PITCH_GOAL_4; //commanded pitch goal, degrees, above the horizon
volatile float rollGoal =        DEFAULT_PITCH_GOAL_4; //commanded roll goal, degrees, left wing above horizon
volatile float yawGoal =         DEFAULT_PITCH_GOAL_4; //commanded yaw goal, degrees, compass reading for forward

float pitchDiff =                      0.0;
float rollDiff =                       0.0;
float yawDiff =                        0.0;

volatile float canardDeflect =   CANARD_DEFLECT_0;   //commanded aileron deflection in canard mode, positive is up from wing
volatile float elevatorDeflect = ELEVATOR_DEFLECT_0;   //commanded elevator deflection, positive is up from wing
volatile float aileronDeflect =  AILERON_DEFLECT_0;    //commanded aileron deflection, positive is up from right wing

volatile bool explosiveSafetyOn =    true; // must set to false before able to trigger parachute ejection charge
volatile char fireEjectionCharge[] = "no"; // must set to exactly "FIRE" to trigger parachute ejection charge


// *** FLIGHT STATES ***
volatile int STATE =                   0;     //start in SCRUB, then setup() switches to GROUND_READY if diagnostic() passes
volatile bool AUTOSTATE =              true;  //whether or not to allow automatic switching between flight state
volatile bool firstTimeThroughState =  true;  //does some logic inside a state if it's the first time through
volatile bool holdMET =                false; //whether or not to hold the elapsed time counter



 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//Interrupt Flag
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}
 
 
 
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 
void setup() {
 
    servoLeft.attach(servoLeftPin);
    servoRight.attach(servoRightPin);

    

    
 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
 
    // verify connection
    Serial.println(F("Testing device connections..."));

    ///////////////////////////////////
    //      Diagnose pixy
    //////////////////////////////////
    pixy.init();
    Serial.println(F("Pixy initialized"));
    
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //pi_go();
    STATE = diagnostic();
    //update_state();

    
    

    
 
}

void pi_go(){
  // wait for ready
    Serial.println(F("\nSend any character to begin"));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data from Pi serial
    while (Serial.available() && Serial.read()); // empty buffer again
}


 
bool diagnostic(){
    int error_sum = 0;
    // int error_sum is used as a bitset, where, when read as a binary number, it
    // tells you exactly where your errors were, no matter the combination.
    //error_sum = (error_sum << 1) + 1; pushes 1 bit onto string of bits for error
    //error_sum = (error_sum << 1);  pushes 0 bit onto string of bits for no error
    //^^^ this method allows for easy addition of error checking protocols without hard coding the decimal values
    //and changing of order in code does not affect
    // load and configure the DMP



    ///////////////////////////////////
    //      Diagnose MPU
    //////////////////////////////////
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //MPU check
    // verify MPU connection
    if(!mpu.testConnection()){
      //transmit_data("ERROR: IMU Connection Issues\n");
      error_sum = (error_sum << 1) + 1;
    }
    else{
      error_sum = (error_sum << 1);
    }
    
  ///////////////////////////////////
    //      Diagnose PI communication
    //////////////////////////////////
  // *** PI to nano handshake
  // send "handshake" to pi and wait to recieve "sure"
  // if sure not recieved, wait 100 ms then send again
  // resend 3 times before throwing error
  Serial.println(F("\nwaiting for handshake from pi "));
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
    //transmit_data("ERROR: PI to NANO Packet Loss\n");
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }

  ///////////////////////////////////
    //      Diagnose Servos
    //////////////////////////////////
  servo_sweep();
  if (servoRightAngle != servoRight.read()){ //TODO:350 can we actually read from these servos?
    //transmit_data("ERROR: Right Servo Angle Error\n");
    //TODO:70 OUTPUT TO TRANSCIEVER: ERROR right servo: servoRight.read() angle off by (servoRight.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }
  if (servoLeftAngle != servoLeft.read()){
    //transmit_data("ERROR: Left Servo Angle Error\n");
    //TODO:50 OUTPUT TO TRANSCIEVER: ERROR left servo: servoLeft.read() angle off by (servoLeft.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }
  if (servoBackAngle != servoBack.read()){
    //transmit_data("ERROR: Back Servo Angle Error\n");
    //TODO:30 OUTPUT TO TRANSCIEVER: ERROR back servo: servoBack.read() angle off by (servoBack.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }

  
  ///////////////////////////////////
    //      Diagnose Results
    //////////////////////////////////
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



void update_state(){
  //trxInterruptFlag = false;

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
    case 0: //0 SCRUB **********************************************************
      if(firstTimeThroughState){
        //TODO:730 things
      }

      //TODO:150 TX warning messages, etc

      //if (AUTOSTATE && !trxInterruptFlag){
      if(AUTOSTATE){
        // SCRUB never automatically changes to another state
        // the Diagnostics() function has authority to change state to GROUND_READY
      }
      break;

    case 1: //1 GROUND_READY ***************************************************
      if(firstTimeThroughState){
        aileronDeflect =  AILERON_DEFLECT_1;
        elevatorDeflect = ELEVATOR_DEFLECT_1;
        canardDeflect =   CANARD_DEFLECT_1;

        millMET = -300000; //T-300 seconds //TODO remember to change to #DEFINE
        millisMETOffset = millis();
      }
      else{
        updateMET();
      }

      //TODO:420 following
      // Do final calibration
      // Slow logging

      // Set control surfaces to launch position
      canardWrite(canardDeflect);
      //aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

//      if (AUTOSTATE && !trxInterruptFlag){
        if (AUTOSTATE){
        // Sense Launch Sensor Fusion
        if(accelForward>LAUNCH_ACCEL && altitude>ALTIMETER_UNCERTAINTY){ //TODO
            STATE = 2; //BOOST
            firstTimeThroughState = true;
        }
      }
      break;

    case 2: //2 BOOST **********************************************************
      if(firstTimeThroughState){
        aileronDeflect =  AILERON_DEFLECT_2;
        elevatorDeflect = ELEVATOR_DEFLECT_2;
      }

      updateMET();

      //TODO:430 following
      // Fast logging

      // Keep control surfaces at launch position
      aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

//      if (AUTOSTATE && !trxInterruptFlag){
        if (AUTOSTATE){
        // IIST sensor //TODO reenable if used
        //boosterIIST = digitalRead(BOOSTER_IIST);

        // Sense Ejection Sensor Fusion
        if(abs(accelForward)>SEP_ACCEL && altitude>MIN_SEP_ALTITUDE){
            //if(!boosterIIST) //if booster is not still there //TODO reenable if used
            //{
            //    STATE = 3; // SEPARATE
            //    firstTimeThroughState = true;
            //}
        }

        // Timer Watchdog
        //TODO:590 make this

      }
      break;

    case 3: //3 SEPARATE *******************************************************
      if(firstTimeThroughState){
        aileronDeflect =  AILERON_DEFLECT_3;
        elevatorDeflect = ELEVATOR_DEFLECT_3;
      }

      updateMET();

      //TODO:440 following
      // Fast logging

      // Set control surfaces to SEP position
      aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

      //if (AUTOSTATE && !trxInterruptFlag){
      if (AUTOSTATE){
        // Timer Watchdog
        //TODO:600 make this

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO:610 make this
      }
      break;

    case 4: //4 STABILIZE ******************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_4;
        rollGoal =  DEFAULT_ROLL_GOAL_4;
        yawGoal =   DEFAULT_YAW_GOAL_4;
      }

      updateMET();

      genericControl();

      //TODO:450 following
      // Autopilot for Stall/Spin recovery
      // Fast logging

//      if (AUTOSTATE && !trxInterruptFlag){
        if (AUTOSTATE){
        // Sense Stable Flight Sensor Fusion
        //TODO:520 logic behind this
        if(false){
          STATE = 5; // SEARCH
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO:620 make this

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO:630 make this
      }
      break;

    case 5: //5 SEARCH *********************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_5;
        rollGoal =  DEFAULT_ROLL_GOAL_5;
        yawGoal =   DEFAULT_YAW_GOAL_5;
      }

      updateMET();

      genericControl();

      //TODO:460 following
      // Autopilot for Search pattern
      // Fast logging

//      if (AUTOSTATE && !trxInterruptFlag){
      if (AUTOSTATE){
        // Accurate Solution of Relative Position Sensor Fusion
        //TODO:530 logic behind this
        if(false){
          STATE = 6; // ORBIT
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO:640 make this

        // Telemetry Out of Norm Sensor Fusion
        //TODO:540 logic behind this
        if(false){
          STATE = 4; // STABILIZE
          firstTimeThroughState = true;
        }

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO:650 make this
      }
      break;

    case 6: //6 ORBIT **********************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_6;
        rollGoal =  DEFAULT_ROLL_GOAL_6;
        yawGoal =   DEFAULT_YAW_GOAL_6;

      }

      updateMET();

      genericControl();

      //TODO:470 following
      // Autopilot for Orbit pattern
      // Fast logging

//      if (AUTOSTATE && !trxInterruptFlag){
        if (AUTOSTATE){
        // Lost Sight of Target Sensor Fusion
        //TODO:550 logic behind this
        if(false){
          STATE = 5; // SEARCH
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO:660 make this

        // Telemetry Out of Norm Sensor Fusion
        //TODO:560 logic behind this
        if(false){
          STATE = 4; // STABILIZE
          firstTimeThroughState = true;
        }

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO:670 make this
      }
      break;

    case 7: //7 RECOVER_ABORT **************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_7;
        rollGoal =  DEFAULT_ROLL_GOAL_7;
        yawGoal =   DEFAULT_YAW_GOAL_7;
      }

      updateMET();

      genericControl();

      //TODO:480 following
      // Autopilot for Abort
      // if (!explosiveSafetyOn && fireEjectionCharge == "FIRE"), fire it.
      // Fast logging

//      if (AUTOSTATE && !trxInterruptFlag){
        if (AUTOSTATE){
        // Landed? Sensor Fusion
        //TODO:570 logic behind this
        if(false){
          STATE = 8; // LANDED
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO:680 make this
      }
      break;

    case 8: //8 LANDED *********************************************************
      if(firstTimeThroughState){
        //TODO:740 things
      }

      updateMET();

      genericControl();

      //TODO:490 following
      // Some shutdown steps (especially for servos)
      // Recovery Beacon?
      // Slow logging

//      if (AUTOSTATE && !trxInterruptFlag){
      if (AUTOSTATE){
        //no auto state change from this state
      }
      break;
      default: // other than 0-8 *************************************************
      //TODO:10 Debug Error: what the hell? how did I get to a nonexistant state?

      updateMET();

//      if (AUTOSTATE && !trxInterruptFlag){
      if (AUTOSTATE){
        STATE = 4; //revert to Stabilize
        firstTimeThroughState = true;
      }
      break;
  }
}



void genericControl(){
  pitchDiff = currentPitch - pitchGoal;
  rollDiff =  currentRoll -  rollGoal;
  yawDiff =   currentYaw -   yawGoal;

  elevatorWrite(pitchDiff);
  aileronWrite(rollDiff);//TODO check direction
}


void aileronWrite(float aileronDeflection){
  servoLeftAngle =  constrain(SERVO_CENTER + (int)(SERVO_AILERON_MULT*aileronDeflection), SERVO_MIN, SERVO_MAX);
  servoRightAngle = constrain(SERVO_CENTER + (int)(SERVO_AILERON_MULT*aileronDeflection), SERVO_MIN, SERVO_MAX); //TODO:360 check direction
  servoLeft.write(servoLeftAngle);
  servoRight.write(servoRightAngle);
}


void elevatorWrite(float elevatorDeflection){
  servoBackAngle = constrain(SERVO_CENTER + (int)(SERVO_ELEVATOR_MULT*elevatorDeflection), SERVO_MIN, SERVO_MAX);//TODO:370 check direction
  servoBack.write(servoBackAngle);
}


void canardWrite(float canardDeflection){
  servoLeftAngle =  constrain(SERVO_CENTER + (int)(SERVO_AILERON_MULT*canardDeflection), SERVO_MIN, SERVO_MAX);//TODO:370 check direction
  servoRightAngle = constrain(SERVO_CENTER + (int)(SERVO_AILERON_MULT*canardDeflection), SERVO_MIN, SERVO_MAX);//TODO:370 check direction
  servoLeft.write(servoLeftAngle);
  servoRight.write(servoRightAngle);
}


void updateMET(){
  millMET = millis()-millisMETOffset;
}


void servo_sweep(){
  int numberofsweeps = 3;
  for(int x = 0; x < numberofsweeps; x++)
  {
    int pos = 90;
    for(pos; pos <= (180); pos += 1){//Sweep up to max range
      servoRight.write(pos);
      servoLeft.write(pos);
      servoBack.write(pos);
      Serial.print("wrote: ");
      Serial.print(pos);
      Serial.println(" to left servo");
      delay(25);
    }
    delay(500);
    for(pos; pos >= (0); pos -= 1){ //Sweep down to min range
      servoRight.write(pos);
      servoLeft.write(pos);
      servoBack.write(pos);
      Serial.print("wrote: ");
      Serial.print(pos);
      Serial.println(" to left servo");
      delay(25);
    }
  }
  

  
}


void writetoleftservo(int x){
  servoLeft.write(x);
}
 
void writetorightservo(int x){
  servoRight.write(x);
}

void writetobackservo(int x){
  servoBack.write(x);
}

float setBarometerZero(){
  //TODO:710 resets the barometer to zero and returns the pressure value
  return -1.0f;
}
 
int detectobjectwithcamera(){
   
  //Pixy is transmittingo on Analog 1
  pixyValue = analogRead(A1); 
  //less than 15 because of noise on cable
  if(pixyValue < 15)
  {
    anglefrompixypixels = 1000;
  }else
  {
  //normalize over pixels on screen 0-760, 7.78 = half of FOV in inches, 15.57 = length of field of view in inches
    anglefrompixypixels = ((pixyValue/760.0) * 15.56) - 7.78;
  }
  //print for debug
  //Serial.println(anglefrompixypixels);
  return anglefrompixypixels;
}






// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
 
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

        /////////////////////////
        //  print current state
        /////////////////////
         Serial.print("current state: ");
         Serial.println(STATE);
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
 
 
   
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        ////////////////////////////////
        //  Object Detection is here because
        //  because the interrupts have a greater possesion
        //  of the processor, so the cam updates on the interrupts too. 
        //////////////////////////////
        pixyResponse = detectobjectwithcamera();

         ////////////////////////////////
        //  update_state is here because
        //  because the interrupts have a greater possesion
        //  of the processor, so the states update on the interrupts too. 
        //////////////////////////////
        update_state();
       
       
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] );
 
            writetoleftservo( int( ypr[2]*45.0 + 90) ) ;
            writetorightservo( int( ypr[2]*45.0 + 90) ) ;
            writetobackservo( int( ypr[2]*45.0 + 90) ) ;
            Serial.print("Left Servo: ");
            Serial.println( int( ypr[2]*45.0 + 90) ) ;
        #endif
 
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif
 
  
 
    }
}
