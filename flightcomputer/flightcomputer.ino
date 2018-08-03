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
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

//using namespace std; //could


/* NOTE: Use "volatile" prefix for values directly modified inside an interrupt.
*  do not set variables inside an interrupt unless they have this volatile prefix.
*/


// *** CONSTANT DEFINITIONS ****************************************************

// *** PINS ***
//#define BOOSTER_IIST              A6 //Analog  pin 6  //TODO add if used
#define SERVO_LEFT                 5 //Digital pin 5
#define SERVO_RIGHT                6 //Digital pin 6
#define SERVO_BACK                 9 //Digital pin 9
#define MPU_INT_PIN                2 //MPU interrupt pin
#define TRX_INT_PIN                3 //tranciever interrupt pin
#define PIXY_CS_PIN                8
#define SPI_PIN_MOSI              11
#define SPI_PIN_MISO              12
#define SPI_PIN_SCK               13

#define PIXY_PIN                  A2
#define I2C_SCL                   A4//TODO correct order
#define I2C_SDA                   A5

// *** I2C ***
#define MPU_ADDR                0x68 //I2C Address of MPU 6050

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

Pixy pixy; // This is the main Pixy object

// *** IIST Sensors *** //TODO may exclude from glider
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
volatile bool mpuInterruptFlag = false;  // indicates whether MPU interrupt pin has gone high


// *** TRX ***

volatile bool trxInterruptFlag =   false;  // indicates whether TRX interrupt pin has gone high
#define RFM95_CS                  16 //TODO UNKNOWN WHAT DIS
#define RFM95_RST                  5 //TODO UNKNOWN WHAT DIS (DIGITAL DATA PIN?)
#define RFM95_INT                  4 //TODO UNKNOWN WHAT DIS (INTERRUPT PIN?)
#define RF95_FREQ              915.0 //Transmit Frequency
#define TRX_POWER                 23 //output power (5-23 dBm)
#define CLIENT_ADDRESS             1 //TODO set from Pi handshake?
#define SERVER_ADDRESS             2 //TODO set from Pi handshake?
RH_RF95 rf95(RFM95_CS, RFM95_INT);    //Radio Driver Object
RHReliableDatagram manager(rf95, CLIENT_ADDRESS); //Radio Manager Object

//*** COMMUNICATIONS ***
char output_string[250];
char temp_str[20];


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



// *** FUNCTIONS ***************************************************************

void setup() {
  //TODO:720 should we lock the mutux in this function?
  //wait a little for any power fluctuations
  //delay(SETUP_WAIT_TIME); <-- I believe the processor takes care of this

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
  trxInterruptFlag = false;
  collectData();
  make_string();
  transmit_pi(output_string);
  transmit_trx(output_string);

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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
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

      if (AUTOSTATE && !trxInterruptFlag){
        //no auto state change from this state
      }
      break;

    default: // other than 0-8 *************************************************
      //TODO:10 Debug Error: what the hell? how did I get to a nonexistant state?

      updateMET();

      if (AUTOSTATE && !trxInterruptFlag){
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
    while (!Serial.available());                 // wait for data from Pi serial
    while (Serial.available() && Serial.read()); // empty buffer again
}


void transmit_trx(char* data){ // Send a message to rf95_server
  //TODO: Adjust code to that from Gabe
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS);

  if(manager.recvfromAckTimeout(buf, &len, 2000, &from)){
    //Sent successful, acknowledge recieved
    //Serial.print("got reply from : 0x"); Serial.print(from, HEX);Serial.print(": "); Serial.println((char*)buf);
  }
  else{
    //No acknowledge
   // Serial.println("No reply, is rf95_reliable_datagram_server running?");
  }
}


char recieve_trx(){
  //TODO put recieve code here
}


void initialize(){

  //Transciever Setup
  //enable interrupt attach
  attachInterrupt(digitalPinToInterrupt(TRX_INT_PIN), trxInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuInterrupt, RISING);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  rf95.init(); //radio initialization
  rf95.setFrequency(RF95_FREQ); //set frequency
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
        transmit_trx("ERROR: IMU Initialization Failed\n");
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
  servoRightAngle = (pos-1);
  servoLeftAngle = (pos-1);
  servoBackAngle = (pos-1);
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
    transmit_trx("ERROR: PI to NANO Packet Loss\n");
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }

  //MPU check
  // verify MPU connection
  if(!mpu.testConnection()){
    transmit_trx("ERROR: IMU Connection Issues\n");
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }
  //Verify MPU acc and gryo readings

  //Servo 'Dance'
  servo_sweep();
  if (servoRightAngle != servoRight.read()){ //TODO:350 can we actually read from these servos?
    transmit_trx("ERROR: Right Servo Angle Error\n");
    //TODO:70 OUTPUT TO TRANSCIEVER: ERROR right servo: servoRight.read() angle off by (servoRight.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }
  if (servoLeftAngle != servoLeft.read()){
    transmit_trx("ERROR: Left Servo Angle Error\n");
    //TODO:50 OUTPUT TO TRANSCIEVER: ERROR left servo: servoLeft.read() angle off by (servoLeft.read() - SERVO_CENTER)
    error_sum = (error_sum << 1) + 1;
  }
  else{
    error_sum = (error_sum << 1);
  }
  if (servoBackAngle != servoBack.read()){
    transmit_trx("ERROR: Back Servo Angle Error\n");
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


void dmpDataReady() { //TODO what does/did this do?
    mpuInterruptFlag = true;
}


void mpuInterrupt(){

}


void trxInterrupt(){
  //TODO:500 get transciver data
  //STATE = sent_string;
}


float setBarometerZero(){
  //TODO:710 resets the barometer to zero and returns the pressure value
  return -1.0f;
}


void collectData(){
  //TODO:120 Populate the META FLIGHT VARIABLES with values from the volatile sets

}
void transmit_pi(char* str){
  Serial.println(str);
}
void make_string(){
  //TODO put in true global variables for 'data'
  //data         Precision          Max Char
  //----------------------------------------
  //STATE          1                    1
  //ACCX           0.1                  6
  //AXXY           0.1                  6
  //ACCZ           0.1                  6
  //YAW            0.1                  5
  //PITCH          0.1                  5
  //ROLL           0.1                  5
  //ALT            0.01                 6
  //TEMP           1                    3
  //M.E.T          0.01                 7
  //PIXY           1                    2
  //OFlg           1                    2
  // + #*del       1                   12
  //---------------------------------------
  //Total         n/a                  66

  char del[] = "\t";
  bool overflw_flg = 0;
  output_string[0] = '\0'; //clear string
  //STATE
  if (conv_to_str(STATE,0) > 2){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //ACCX
  if (conv_to_str(ACCX,1) > 6){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //ACCY
  if (conv_to_str(ACCY,1) > 6){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //ACCZ
  if (conv_to_str(ACCZ,1) > 6){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //YAW
  if (conv_to_str(YAW,1) > 5){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //PITCH
  if (conv_to_str(PITCH,1) > 5){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
   //ROLL
  if (conv_to_str(ROLL,1) > 5){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //ALT
  if (conv_to_str(ALT,2) > 6){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //TEMP
  if (conv_to_str(TEMP,0) > 3){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //M.E.T
  if (conv_to_str(MET,2) > 7){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //PIXY
  if (conv_to_str(PIXY,0) > 2){
    overflw_flg = 1;
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //OFlg
  if (overflw_flg){ //Overflow occurred
    conv_to_str(1,0)
  }
  if (!overflw_flg) //No overflow
    conv_to_str(0,0)
  }
  strcat(output_string, temp_str);
  strcat(output_string, del);
  //String Complete! Ready to Transmit!
  
}

int conv_to_str(float data, int prec){
int mag= 0;
int start = 0;
int lop, len;
  temp_str[0] = '\0'; //Clear temp string
  if (data < 0){ //Negative numbers
    data = -data;
    start = 1;
  }
  while (data > 10){ //Get power of 10 (above 0)
    data = data/10;
    mag = mag + 1;
  }
  len = mag+ prec + start + 2;
  if (start == 1){
    temp_str[0] = '-';
  }
  for(lop = start; lop < len; lop += 1){ //generate string
    if (lop == mag + start + 1){
      temp_str[lop] = '.';
    }
    else{
      temp_str[lop] = (int)data + '0';
      data = data - (int)data;
      data = data*10;
    }
  }
  temp_str[len] = '\0';
  return len;
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
