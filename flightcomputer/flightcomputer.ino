/*flightcomputer.ino
* Autopilot, sensor fusion and TRX computer for the Intern Space Program:
* Flying Squirrel mission.
* Contributors:
* Connor Jakubik, Ronnie Ankner, olenakotaco, Gabe R, Bluthman2
*/
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "Pixy.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LoRa.h"

//using namespace std; //could


/* NOTE: Use "volatile" prefix for values directly modified inside an interrupt.
*  do not set variables inside an interrupt unless they have this volatile prefix.
*/


// *** CONSTANT DEFINITIONS ****************************************************

// *** ELECTRICAL ***
// define specific voltages and stuff here

// *** PINS ***
#define BOOSTER_IIST              A6 //Analog  pin 6  //TODO update to reflect actual pin
#define I2C_SCL                   -1 //Digital pin #  //TODO update to reflect actual pin
#define I2C_SDA                   -1 //Digital pin #  //TODO update to reflect actual pin
#define SERVO_LEFT                 9 //Digital pin 9
#define SERVO_RIGHT               10 //Digital pin 10
#define SERVO_BACK                11 //Digital pin 10
#define MPU_INT_PIN                2 //MPU interrupt pin
#define TRX_INT_PIN                3 //tranciever interrupt pin

// *** I2C ***
#define MPU_ADDR             0000000 //I2C Address of MPU //TODO update to reflect actual address

// *** STATE-SPECIFIC PARAMETERS *** //TODO tune all
//0 SCRUB
#define ELEVATOR_DEFLECT_0       0.0 //degrees
#define AILERON_DEFLECT_0        0.0 //degrees
//1 GROUND_READY
#define ELEVATOR_DEFLECT_1       0.0 //degrees
#define AILERON_DEFLECT_1        0.0 //degrees
#define LAUNCH_ACCEL           980.0 // m/s^2, the forward acceleration at which launch definitely happened
#define MINTIME_1                0.0 //seconds
#define MAXTIME_1          9000000.0 //seconds
//2 BOOST
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
#define DEFAULT_AZIMUTH_GOAL_4 270.0 //degrees (WEST)
#define MINTIME_4                0.0 //seconds
#define MAXTIME_4          9000000.0 //seconds
//5 SEARCH
#define DEFAULT_PITCH_GOAL_5   -10.0 //degrees
#define DEFAULT_ROLL_GOAL_5    -30.0 //degrees
#define DEFAULT_AZIMUTH_GOAL_5 270.0 //degrees (WEST)
#define MINTIME_5                0.0 //seconds
#define MAXTIME_5          9000000.0 //seconds
//6 ORBIT
#define DEFAULT_PITCH_GOAL_6   -10.0 //degrees
#define DEFAULT_ROLL_GOAL_6    -30.0 //degrees
#define DEFAULT_AZIMUTH_GOAL_6 270.0 //degrees (WEST)
#define MINTIME_6                0.0 //seconds
#define MAXTIME_6          9000000.0 //seconds
//7 RECOVER_ABORT
#define PARACHUTE_ALTITUDE     167.6 //meters, 549.9 ft, altitude threshold to deploy parachute at
#define DEFAULT_PITCH_GOAL_7    10.0 //degrees
#define DEFAULT_ROLL_GOAL_7    -30.0 //degrees
#define DEFAULT_AZIMUTH_GOAL_7 270.0 //degrees (WEST)
#define MINTIME_7                0.0 //seconds
#define MAXTIME_7          9000000.0 //seconds
//8 LANDED
#define TIME_BETWEEN_BEACON     10.0 //seconds, time between signal pulses (if we do an RF locator to assist in recovery)

// *** OTHER ***
#define LOCK                       1 //mutux lock
#define UNLOCK                     0 //mutux unlock
#define SETUP_WAIT_TIME         1000 //milliseconds
#define BAUD_RATE             115200 //agreed baud rate
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

// *** META FLIGHT VARIABLES ***
// These values are more reliable and isolated from the volatile control flight variables.
// Use these values when TX to ground station.

float currentPitch =   0.0; // degrees, pitch above the horizon
float currentRoll =    0.0; // degrees, roll left wing above horizon //TODO coordinate system document
float currentAzimuth = 0.0; // degrees, compass reading from forward of glider

float altitude =  -10000.0; // meters
float altitudeRate = 0.0; // m/s

float accelForward =   0.0; // acceleration in forward direction, no gravity
float accelGliderFrame[3]; // acceleration in glider frame, no gravity
float accelWorldFrame[3]; // acceleration in world frame (x+ East, y+ North, z+ Up), yes gravity

// *** CONTROL FLIGHT VARIABLES ***
// These values are meant to be used for RX from ground station and set by autopilot
// Be cautious to use these values when TX to ground station.

volatile float pitchGoal =       DEFAULT_PITCH_GOAL_4; //commanded pitch goal, degrees, above the horizon
volatile float rollGoal =        DEFAULT_PITCH_GOAL_4; //commanded roll goal, degrees, left wing above horizon
volatile float azimuthGoal =     DEFAULT_PITCH_GOAL_4; //commanded azimuth goal, degrees, compass reading for forward

volatile float elevatorDeflect = ELEVATOR_DEFLECT_0;   //commanded elevator deflection, positive is up from wing
volatile float aileronDeflect =  AILERON_DEFLECT_0;    //commanded aileron deflection, positive is up from right wing

volatile bool explosiveSafetyOn =    true; // must set to false before able to trigger parachute ejection charge
volatile char fireEjectionCharge[] = "no"; // must set to exactly "FIRE" to trigger parachute ejection charge


// *** FLIGHT STATES ***
int STATE =      0;    //start in SCRUB, then setup() switches to GROUND_READY if diagnostic() passes
bool AUTOSTATE = true; //whether or not to allow automatic switching between flight state
bool firstTimeThroughState = true; //does some logic inside a state if it's the first time through



// *** FUNCTIONS ***************************************************************

void setup() {
  //TODO should we lock the mutux in this function?
  //wait a little for any power fluctuations
  delay(SETUP_WAIT_TIME);

  bool diagnosticPass = false;

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
  //driver.init() is Transceiver setup
  pi_go();
  initialize();
  diagnosticPass = diagnostic();
  STATE = (int)diagnosticPass; // 0 for fail, 1 for success
}


void loop() {
  // put your main code here, to run repeatedly:
loop_start: collectData(); //TODO is this just a vestige of the GOTO stuff?
  trxInterrupt = false;

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
        //TODO things
      }

      //TODO TX warning messages, etc

      if (AUTOSTATE && !trxInterrupt){
        // SCRUB never automatically changes to another state
        // the Diagnostics() function has authority to change state to GROUND_READY
      }
      break;

    case 1: //1 GROUND_READY ***************************************************
      if(firstTimeThroughState){
        aileronDeflect = AILERON_DEFLECT_1;
        elevatorDeflect = ELEVATOR_DEFLECT_1;
      }

      //TODO following
      // Do final calibration
      // Slow logging

      // Set control surfaces to launch position
      aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

      if (AUTOSTATE && !trxInterrupt){
        // Sense Launch Sensor Fusion
        if(false/*accelForward>LAUNCH_ACCEL && altitude>ALTIMETER_UNCERTAINTY*/){ //TODO
            STATE = 2; //BOOST
            firstTimeThroughState = true;
        }
      }
      break;

    case 2: //2 BOOST **********************************************************
      if(firstTimeThroughState){
        aileronDeflect = AILERON_DEFLECT_2;
        elevatorDeflect = ELEVATOR_DEFLECT_2;
      }

      //TODO following
      // Fast logging

      // Keep control surfaces at launch position
      aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

      if (AUTOSTATE && !trxInterrupt){
        // IIST sensor
        boosterIIST = digitalRead(BOOSTER_IIST);

        // Sense Ejection Sensor Fusion
        if(abs(accelForward)>SEP_ACCEL && altitude>MIN_SEP_ALTITUDE){
            if(!boosterIIST) //if booster is not still there
            {
                STATE = 3; // SEPARATE
                firstTimeThroughState = true;
            }
        }

        // Timer Watchdog
        //TODO make this

      }
      break;

    case 3: //3 SEPARATE *******************************************************
      if(firstTimeThroughState){
        aileronDeflect = AILERON_DEFLECT_3;
        elevatorDeflect = ELEVATOR_DEFLECT_3;
      }

      //TODO following
      // Fast logging

      // Set control surfaces to SEP position
      aileronWrite(aileronDeflect);
      elevatorWrite(elevatorDeflect);

      if (AUTOSTATE && !trxInterrupt){
        // Timer Watchdog
        //TODO make this

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO make this
      }
      break;

    case 4: //4 STABILIZE ******************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_4;
        rollGoal = DEFAULT_ROLL_GOAL_4;
        azimuthGoal = DEFAULT_AZIMUTH_GOAL_4;

      }

      //TODO following
      // Autopilot for Stall/Spin recovery
      // Fast logging

      if (AUTOSTATE && !trxInterrupt){
        // Sense Stable Flight Sensor Fusion
        //TODO logic behind this
        if(false){
          STATE = 5; // SEARCH
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO make this

        // Less than parachute altitude
        if(altitude<PARACHUTE_ALTITUDE){
          STATE = 7; // RECOVER_ABORT
          firstTimeThroughState = true;
        }

        // Failure Mode Detected
        //TODO make this
      }
      break;

    case 5: //5 SEARCH *********************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_5;
        rollGoal = DEFAULT_ROLL_GOAL_5;
        azimuthGoal = DEFAULT_AZIMUTH_GOAL_5;

      }

      //TODO following
      // Autopilot for Search pattern
      // Fast logging

      if (AUTOSTATE && !trxInterrupt){
        // Accurate Solution of Relative Position Sensor Fusion
        //TODO logic behind this
        if(false){
          STATE = 6; // ORBIT
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO make this

        // Telemetry Out of Norm Sensor Fusion
        //TODO logic behind this
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
        //TODO make this
      }
      break;

    case 6: //6 ORBIT **********************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_6;
        rollGoal = DEFAULT_ROLL_GOAL_6;
        azimuthGoal = DEFAULT_AZIMUTH_GOAL_6;

      }

      //TODO following
      // Autopilot for Orbit pattern
      // Fast logging

      if (AUTOSTATE && !trxInterrupt){
        // Lost Sight of Target Sensor Fusion
        //TODO logic behind this
        if(false){
          STATE = 5; // SEARCH
          firstTimeThroughState = true;
        }

        // Timer Watchdog
        //TODO make this

        // Telemetry Out of Norm Sensor Fusion
        //TODO logic behind this
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
        //TODO make this
      }
      break;

    case 7: //7 RECOVER_ABORT **************************************************
      if(firstTimeThroughState){
        pitchGoal = DEFAULT_PITCH_GOAL_7;
        rollGoal = DEFAULT_ROLL_GOAL_7;
        azimuthGoal = DEFAULT_AZIMUTH_GOAL_7;

      }

      //TODO following
      // Autopilot for Abort
      // if (!explosiveSafetyOn && fireEjectionCharge == "FIRE"), fire it.
      // Fast logging

      if (AUTOSTATE && !trxInterrupt){
        // Landed? Sensor Fusion
        //TODO logic behind this
        if(false){
          STATE = 8; // LANDED
          firstTimeThroughState = true;
        } 

        // Timer Watchdog
        //TODO make this
      }
      break;

    case 8: //8 LANDED *********************************************************
      if(firstTimeThroughState){
        //TODO things
      }

      //TODO following
      // Some shutdown steps (especially for servos)
      // Recovery Beacon?
      // Slow logging

      if (AUTOSTATE && !trxInterrupt){
        //no auto state change from this state
      }
      break;

    default: // other than 0-8 *************************************************
      //TODO Debug Error: what the hell? how did I get to a nonexistant state?

      if (AUTOSTATE && !trxInterrupt){
        STATE = 4; //revert to Stabilize
        firstTimeThroughState = true;
      }
      break;
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
  //enable interrupt attach
  attachInterrupt(digitalPinToInterrupt(TRX_INT_PIN), transInterrupt, RISING);
  //TODO communications setup

  //MPU Setup
    mpu.initialize();
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity //TODO tune these?
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
        //TODO REPORT ERROR HERE OVER TRANSCIVER
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


void servo_sweep(){
  int pos = SERVO_CENTER;
  for(pos; pos <= (SERVO_MAX); pos += 1){//Sweep up to max range
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(25);
  }
  delay(500);
  for(pos; pos >= (SERVO_MIN); pos -= 1){ //Sweep down to min range
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(25);
  }
  delay(500);
  for(pos; pos <= (SERVO_MAX); pos += 1){//Sweep up to max range
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(25);
  }
  delay(500);
  for(pos; pos >= (SERVO_CENTER); pos -= 1){ //Sweep down to start
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(15);
  }
  delay(500);
  for(pos; pos >= (SERVO_MIN); pos -= 1){ //Sweep down to min range
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(15);
  }
  delay(500);
  for(pos; pos <= (SERVO_CENTER); pos += 1){//Sweep up to start
    servoRight.write(pos);
    servoLeft.write(pos);
    servoBack.write(pos);
    delay(15);
  }
  servoRightAngle = pos;
  servoLeftAngle = pos;
  servoBackAngle = pos;
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
  char target[4] = "sure";
  for(packet_loss; packet_loss < 3; packet_loss += 1){
    Serial.println("handshake"); //sends impetus for handshake
    Serial.flush(); //waits for outgoing stream to complete
    if (Serial.find(target)){ //breaks from loop if 'sure' is found
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
  if (servoRightAngle != servoRight.read()){ //TODO can we actually read from these servos?
    //TODO OUTPUT TO TRANSCIEVER: ERROR right servo: servoRight.read() angle off by (servoRight.read() - SERVO_CENTER)
    error_sum = error_sum + 4; //4s place
  }
  if (servoLeftAngle != servoLeft.read()){
    //TODO OUTPUT TO TRANSCIEVER: ERROR left servo: servoLeft.read() angle off by (servoLeft.read() - SERVO_CENTER)
    error_sum = error_sum + 8; //8s place
  }
  if (servoBackAngle != servoBack.read()){
    //TODO OUTPUT TO TRANSCIEVER: ERROR back servo: servoBack.read() angle off by (servoBack.read() - SERVO_CENTER)
    error_sum = error_sum + 16; //16s place
  }

  //Final Error Output
  if (error_sum == 0){
    //TODO OUTPUT to TRANSCIEVER: Packet Loss was (packet_loss). No errors, all clear. READY TO LAUNCH!
    return true;
  }
  else{
    //TODO OUTPUT to TRANSCIEVER: Packet Loss was (packet_loss). Error code: (error_sum in binary). SCRUB!
    return false;
  }
  return false;
}


void aileronWrite(float aileronDeflection){
  servoLeftAngle = SERVO_CENTER + (int)aileronDeflection;
  servoRightAngle = SERVO_CENTER + (int)aileronDeflection; //TODO check direction
  servoLeft.write(servoLeftAngle);
  servoRight.write(servoRightAngle);
}


void elevatorWrite(float elevatorDeflection){
  servoBackAngle = SERVO_CENTER + (int)elevatorDeflection;//TODO check direction
  servoBack.write(servoBackAngle);
}


void dmpDataReady() {
    mpuInterrupt = true;
}


void transInterrupt(){
  //TODO get transciver data
  //STATE = sent_string;
}


float setBarometerZero(){
  //TODO resets the barometer to zero and returns the pressure value
  return -1.0f;
}


void collectData(){
  //TODO Populate the META FLIGHT VARIABLES with values from the volatile sets
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
