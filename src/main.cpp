
// @author Michael J Monda
// @author Jace Howhannesian

// class imports.
#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <PIDController.h>
#include <Rangefinder.h>
#include <RemoteConstants.h>
#include <IRdecoder.h>
#include <BlueMotor.h>
#include <Servo32u4.h>

// sensor port numbers.
static int leftSensor = 21;
static int rightSensor = 22;
// static int echoPin = 17;
// static int pingPin = 12;
static int irRemotePin = 14;

// establish robot states, for the state machine setup.
enum chassisState {FOLLOWINGLINE, FOLLOWTOHOUSE, FOLLOWFROMHOUSE, FOLLOWTODEPOT, 
                   CROSSDETECTION, RETURNCROSSDETECTION, HALT, ZERO, 
                   FORTYFIVE, TWENTYFIVE, ONEEIGHTZERO} currState, nextState; // driving
// enum armstrongState {ZERO, FORTYFIVE, TWENTYFIVE} currPosition, nextPosition; // arm actuation
// enum forkilftState {EXTENDED, RETRACTED} currGripState, nextGripState; // gripper control

// chassis, startup button, rangefinder and remote object creation.
Chassis chassis;
Romi32U4ButtonC buttonC;
Rangefinder rangefinder(17, 12);
IRDecoder decoder(irRemotePin);
BlueMotor armstrong;
Servo32U4 servo;

// variable declarations here.
int leftSensVal;
int rightSensVal;
int leftSpeed;
int rightSpeed;
int lineSensingThresh = 250; // < 250 == white, > 250 == black
// static double rangeThreshold = 12.7; // centimeters
int i; // counter for for() loop
int16_t leftEncoderValue;
static int houseEncoderCount = 1138;
static int depotEncoderCount = 1700;
static int fortyfivePosition = 100; // encoder count required to move the arm to the 45-degree position.
static int twentyfivePosition = 4452; // encoder count required to move the arm to the 25-degree position.
bool grabbed = false;
int servoExtend = 2100;
int servoRelease = 2100;

// static int divisor = 120;
static float defaultSpeed = 20.0; // default driving speed
static float constant = 0.01; // proportional gain for the controller function lineFollow()

// Deadband Correction
float angularSpeed;
float currPosition;
float pastPosition;
float currTime;
float pastTime;
float changeDirection;
float adjustedEffort;
float deadband;

// function declarations here.
int getLeftValue();
int getRightValue();
void beginning();
void lineFollow();
void crossDetected(bool);
void returnTurn(bool);
void handleInbound(int);
void deadBand();
void deadBandClocwise();
void deadBandAnticlockwise();
void closeFork();
void openFork();

// configure the robot setup.
void setup() {
    chassis.init();
    decoder.init();
    rangefinder.init();
    armstrong.setup();
    armstrong.reset();
    servo.setMinMaxMicroseconds(900, 2100);
    pinMode(irRemotePin, INPUT);
    Serial.begin(9600);
    currState = FOLLOWINGLINE; // establish initial driving state
    // currPosition = ZERO; // establish initial arm position
    // currGripState = EXTENDED; // establish initial fork position
    buttonC.waitForButton();
    //reset reflectance sensor
    getLeftValue();
    getRightValue();
    delay(1000);
}

void loop() {
    // survey for an inbout remote signal
    int inboundSignal = decoder.getKeyCode(); // when true, the key can be repeated if held down.
    if (inboundSignal != -1) handleInbound(inboundSignal); // inboundSignal == -1 only when unpressed.
    Serial.println(currState);
    Serial.println(analogRead(leftSensor));
    switch(currState) {      
        case FOLLOWINGLINE:
            lineFollow(); // I don't use chassis.setTwist() because it's cringe

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);
                nextState = CROSSDETECTION;
                currState = HALT;
                Serial.println("Checkpoint 1");
                chassis.getLeftEncoderCount(true);
                leftEncoderValue = chassis.getLeftEncoderCount();
            }
        break;

        case CROSSDETECTION:
            Serial.println("Check");
            crossDetected(true); // this function is essentially just a combination of state code from the example provided on Canvas.
            if (currState != CROSSDETECTION) {
                chassis.getLeftEncoderCount(true);
                chassis.getRightEncoderCount(true);
            }
        break;

        case FOLLOWTOHOUSE: // this is configured to use the ultrasonic right now, but can later be used with the encoders if we choose such.
            lineFollow();
            if (chassis.getLeftEncoderCount() >= houseEncoderCount || chassis.getRightEncoderCount() >= houseEncoderCount) {
            chassis.setWheelSpeeds(0, 0);
            currState = HALT;
            nextState = ONEEIGHTZERO;
            // nextState = TWENTYFIVE;
            Serial.println("Checkpoint 2");
            }
        break;
        
        // decided to wrap the EXTENDED and RETRACTED states into FORTYFIVE, TWENTYFIVE, and ZERO for simplicity
        case FORTYFIVE:
            armstrong.moveTo(fortyfivePosition);
            if (abs(armstrong.getPosition() - fortyfivePosition) > 3) {
                nextState = FOLLOWTOHOUSE;
                currState = HALT;
            }
        break;

        case TWENTYFIVE:
            if (armstrong.getPosition() != twentyfivePosition && grabbed == false) {
                armstrong.setEffort(25);
            } 
            if (armstrong.getPosition() == twentyfivePosition && grabbed == false) {
                armstrong.setEffort(0);
                servo.writeMicroseconds(2100);
                grabbed = true;
            }
            if (grabbed == true) {
                armstrong.moveTo(twentyfivePosition + 200);
                chassis.driveFor(-35, 35, true);
                nextState = ONEEIGHTZERO;
                currState = HALT;
                Serial.println("Checkpoint 3");
            }
        break;

        case ONEEIGHTZERO:
            chassis.setWheelSpeeds(-25, 25);
            if (getLeftValue() > lineSensingThresh) {
                    nextState = FOLLOWFROMHOUSE;
                    currState = HALT;
                    Serial.println("Checkpoint 4");
            }
        break;
        
        case FOLLOWFROMHOUSE:
            lineFollow(); // I don't use chassis.setTwist() because it's cringe

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);
                nextState = RETURNCROSSDETECTION;
                currState = HALT;
                Serial.println("Checkpoint 5");
                chassis.getLeftEncoderCount(true);
                leftEncoderValue = chassis.getLeftEncoderCount();
            }
        break;

        case RETURNCROSSDETECTION:
            Serial.println("Check");
            returnTurn(false);
            chassis.getLeftEncoderCount(true);
            chassis.getRightEncoderCount(true);
        break;

        case FOLLOWTODEPOT:
            lineFollow();
            if (chassis.getLeftEncoderCount() >= depotEncoderCount && chassis.getRightEncoderCount() >= depotEncoderCount) {
                chassis.setWheelSpeeds(0, 0);
                currState = HALT;
                nextState = ZERO; // prepare for plate deposit
                Serial.println("Checkpoint 6");
            }
        break;

        case HALT: // remain stopped until the remote is pressed
            chassis.idle();
            armstrong.setEffort(0);
            // Serial.println("Stopped");
        break;

        case ZERO:
            if (armstrong.getPosition() != 0 && grabbed == true) {
                armstrong.setEffort(-25);
            } 
            if (armstrong.getPosition() == 0 && grabbed == true) {
                armstrong.setEffort(0);
                servo.writeMicroseconds(-2100);
                grabbed = false;             
            }
            if (grabbed == false) {
                servo.writeMicroseconds(2100);
                nextState = ONEEIGHTZERO;
                currState = HALT;
            }
        break;
    }
}

// establish functions used to pull the values of left and right sensors.
// get the value of the left sensor, and return a value. 
int getLeftValue() {
    leftSensVal = analogRead(leftSensor);
    return leftSensVal;
}
// same as the previous, but for the right sensor.
int getRightValue() {
    rightSensVal = analogRead(rightSensor);
    return rightSensVal;
}

// this function performs the physical setup for the robot: orienting it in the desired direction.
void beginning() {
    chassis.turnFor(30, 40, true);
    delay(300);
    while (getRightValue() < lineSensingThresh) {
        chassis.setWheelSpeeds(-15, 15);
        if (getRightValue() >= lineSensingThresh) {
            break;
        }
    }
    chassis.idle();
}

// line following function, complete with a PID controller. This took an embarrasing amount of time
// for me to design.
void lineFollow() {
    float difference = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = defaultSpeed + difference;
    float rightSpeed = defaultSpeed - difference;
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

// detect the cross, at which the first turn is performed, and complete the maneuver.
void crossDetected(bool testing) {
    switch (testing) {
        case true:
            chassis.driveFor(7, 10, true);
            chassis.turnFor(-90, 100, true);
            nextState = FORTYFIVE;
            currState = HALT;
        break;

        case false:
            chassis.setWheelSpeeds(defaultSpeed/2, defaultSpeed/2);
            if (leftEncoderValue > 300) {
                chassis.setWheelSpeeds(0, 0);
                delay(100);
                chassis.setWheelSpeeds(25, -25);
                if (getLeftValue() > lineSensingThresh) {
                    nextState = FOLLOWTOHOUSE;
                    currState = HALT;
                    Serial.println("Checkpoint 2");
                }
            }
        break;
    }
}
// detect the cross again, and perform another maneuver.
void returnTurn(bool testing) {
    switch (testing) {
        case true:
            chassis.driveFor(7.33, 10, true);
            chassis.turnFor(90, 100, true);
        break;

        case false:
            chassis.setWheelSpeeds(defaultSpeed/2, defaultSpeed/2);
            if (leftEncoderValue > 300) {
                chassis.setWheelSpeeds(0, 0);
                delay(100);
                chassis.setWheelSpeeds(-25, 25);
                if (getRightValue() > lineSensingThresh) {
                    nextState = FOLLOWTODEPOT;
                    currState = HALT;
                    Serial.println("Checkpoint 7");
                }
            }
        break;
    }
}

void deadBand(bool clockwise, int deadband) {
    // deadband effort calculations
    for (int i = 0; i <= 400; i++) {
        armstrong.setEffortWithDB(i, clockwise, deadband);
    } 
}

void closeFork() {
    servo.writeMicroseconds(servoExtend);
}

void openFork() {
    servo.writeMicroseconds(servoRelease);
}

void handleInbound(int keyPress) { 
  if (keyPress == remotePlayPause)  //This is the emergency stop button
  {
    nextState = currState; //Save the current state so you can pick up where you left off
    currState = HALT;
    Serial.println("Emergency Stop");
  }

  if (keyPress == remoteUp) //This is the proceed button
  {
    currState = nextState;
    Serial.println("Onward");
  }
  if (keyPress == remoteDown) {
    currState = FOLLOWINGLINE;
  }
}