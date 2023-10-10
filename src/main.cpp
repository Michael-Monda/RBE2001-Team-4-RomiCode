
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
#include <math.h>

// sensor port numbers.
static int leftSensor = 21;
static int rightSensor = 22;
// static int echoPin = 17;
// static int pingPin = 12;
static int irRemotePin = 14;

// establish robot states, for the state machine setup.
enum chassisState {FOLLOWINGLINE, FOLLOWTOHOUSE, FOLLOWFROMHOUSE, FOLLOWTODEPOT, 
                   CROSSDETECTION, RETURNCROSSDETECTION, HALT, ZERO, 
                   FORTYFIVE, TWENTYFIVE, ONEEIGHTZERO, GRAB, DROP,
                   // the below states are established to make the robot pick up the panel from the depot
                   LOADPANEL, DROPOFF, END} currState, nextState; // driving
// enum armstrongState {ZERO, FORTYFIVE, TWENTYFIVE} currPosition, nextPosition; // arm actuation
// enum forkilftState {EXTENDED, RETRACTED} currGripState, nextGripState; // gripper control
bool side45 = false;
bool loading = false;

// chassis, startup button, rangefinder and remote object creation.
Chassis chassis;
Romi32U4ButtonC buttonC;
Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
Rangefinder rangefinder(17, 12);
IRDecoder decoder(irRemotePin);
BlueMotor armstrong;
Servo32U4 servo;

// variable declarations here.
int leftSensVal;
int rightSensVal;
int leftSpeed;
int rightSpeed;

long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;
int motorEffort = 400;

static const int lineSensingThresh = 250; // < 250 == white, > 250 == black
// static double rangeThreshold = 12.7; // centimeters
int i; // counter for for() loop
int16_t leftEncoderValue;
static int houseEncoderCount = 1138;    // formerly 1138
static int depotEncoderCount = 1756;    // formerly 1700
static int fortyfivePosition = -3300;   // encoder count required to move the arm to the 45-degree position. (2900)
static int twentyfivePosition = -3900;  // encoder count required to move the arm to the 25-degree position. (4000)
bool grabbed = false;
static const int servoMicroseconds = -500;
int angle;

// static int divisor = 120;
static float defaultSpeed = 15.0; // default driving speed
static const float constant = 0.01; // proportional gain for the controller function lineFollow()

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
void lineFollowToHouse();
void crossDetected();
void returnTurn(bool);
void handleInbound(int);
void closeFork();
void openFork();

// configure the robot setup.
void setup() {
    chassis.init(); // start chassis
    decoder.init(); // initialize ir decoder
    rangefinder.init(); // initialize rangefinder
    armstrong.setup();  // set up blue motor "armstrong"
    armstrong.reset();  // reset armstrong encoder
    servo.setMinMaxMicroseconds(1000, 2000);  // limit servo movement
    pinMode(irRemotePin, INPUT);    // create reciever pin
    Serial.begin(9600);
    currState = FOLLOWINGLINE;  // establish initial driving state
    // currState = GRAB;    // testing only
    // currPosition = ZERO; // establish initial arm position
    // currGripState = EXTENDED;    // establish initial fork position
    buttonC.waitForButton();    // wait until C is pressed to start the code.
    // reset reflectance sensor
    getLeftValue();
    getRightValue();
    beginning();    // initial turn
    delay(1000);
}

void loop() {
    // survey for an inbout remote signal
    int inboundSignal = decoder.getKeyCode();   // when true, the key can be repeated if held down.
    if (inboundSignal != -1) handleInbound(inboundSignal);  // inboundSignal == -1 only when unpressed.
    // Serial.println(currState)
    switch(currState) {      
        case FOLLOWINGLINE:
            lineFollow(); // I don't use chassis.setTwist() because it's inconsistent

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);
                nextState = CROSSDETECTION;
                currState = HALT;    

                Serial.println("Checkpoint 1");
                // chassis.getLeftEncoderCount(true);
                // leftEncoderValue = chassis.getLeftEncoderCount();
            }
        break;

        case CROSSDETECTION:
            Serial.println("Check");
            crossDetected(); // this function is essentially just a combination of state code from the example provided on Canvas.
            if (currState != CROSSDETECTION) {
                chassis.getLeftEncoderCount(true);
                chassis.getRightEncoderCount(true);
            }
        break;

        case FOLLOWTOHOUSE: // this is configured to use the ultrasonic right now, but can later be used with the encoders if we choose such.
            lineFollowToHouse();
            if (chassis.getLeftEncoderCount() >= houseEncoderCount || chassis.getRightEncoderCount() >= houseEncoderCount) {
                chassis.setWheelSpeeds(0, 0);
                if (loading == false) {
                    currState = HALT;
                    nextState = GRAB;
                    Serial.println("Checkpoint 4");
                } else if (loading == true) {
                    currState = HALT;
                    nextState = DROPOFF;
                    Serial.println("begin offload");
                }
                // nextState = TWENTYFIVE;
            }
        break;
        
        // decided to wrap the EXTENDED and RETRACTED states into FORTYFIVE, TWENTYFIVE, and ZERO for simplicity
        case FORTYFIVE:
            Serial.println("sisyphus and the boulder");
            armstrong.moveTo(fortyfivePosition);

            if (armstrong.getPosition() >= fortyfivePosition - 15) {
                nextState = FOLLOWTOHOUSE;
                currState = HALT;
                Serial.println("Checkpoint 3a");
            } 
        break;

        case TWENTYFIVE:
            Serial.println("arm stronging");
            armstrong.moveTo(twentyfivePosition);

            if (armstrong.getPosition() >= twentyfivePosition - 15) {
                nextState = FOLLOWTOHOUSE;
                currState = HALT;
                Serial.println("Checkpoint 3a");
            }
            
        break;

        case ONEEIGHTZERO:
            chassis.setWheelSpeeds(-25, -25);
            delay(215);
            chassis.turnFor(170, 25, true);
            chassis.driveFor(-6, 10, true);
            nextState = FOLLOWFROMHOUSE;
            currState = HALT;
            Serial.println("Spun");
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
            returnTurn(true);
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
            Serial.println("depositing");
            armstrong.moveTo(0);

            if (armstrong.getPosition() <= 15) {
                nextState = DROP;
                currState = HALT;
                Serial.println("Checkpoint 3a");
            }
        break;

        case GRAB:  // TODO: fix servo so that it knows when to close.
            servo.writeMicroseconds(1000);
            delay(800);
            servo.detach();

            chassis.driveFor(5.7, 15, true);
        
            servo.writeMicroseconds(2000);
            delay(700);
            servo.detach();
            delay(500);
            if (side45 == true) {   // if on this side, do this
                armstrong.moveTo(fortyfivePosition - 800);
                delay(100);
                chassis.driveFor(1.2, 10, true);
                delay(10);
                armstrong.moveTo(fortyfivePosition - 1200);
            } else {  // if not, do this
                armstrong.moveTo(twentyfivePosition - 800);
                delay(100);
                chassis.driveFor(1.2, 10, true);
                delay(10);
                armstrong.moveTo(twentyfivePosition - 1200);
            }
            nextState = ONEEIGHTZERO;
            currState = HALT;
        break;

        case DROP:
            servo.writeMicroseconds(1000);
            delay(700);
            servo.detach();

            delay(10);
            chassis.driveFor(-30, 10, true);
            
            loading = true;
            nextState = LOADPANEL;
            currState = HALT;
        break;

        case LOADPANEL:
            lineFollow();
            if (chassis.getLeftEncoderCount() >= depotEncoderCount + 100 && chassis.getRightEncoderCount() >= depotEncoderCount + 100) {
                chassis.setWheelSpeeds(0, 0); 
                armstrong.moveTo(200);               
                servo.writeMicroseconds(2000);
                delay (700);
                servo.detach();
                delay(20);

                if (side45 == true) armstrong.moveTo(fortyfivePosition);
                else armstrong.moveTo(twentyfivePosition);

                chassis.driveFor(-5, 12, true);
                chassis.turnFor(175, 20, true);

                currState = HALT;
                nextState = FOLLOWINGLINE;
            }

        break;
        
        case DROPOFF:
            if (side45 == true) {   // check that the arm is raising to correct positions out of load
                armstrong.moveTo(fortyfivePosition - 1200);
                delay(10);
                chassis.driveFor(7.7, 10, true);    // initial guess was 6.9 (nice!)
                delay(100);
                armstrong.moveTo(fortyfivePosition - 700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                currState = HALT;
                nextState = END;
            } else {
                armstrong.moveTo(twentyfivePosition - 1200);
                delay(10);
                chassis.driveFor(7.7, 10, true);
                delay(100);
                armstrong.moveTo(twentyfivePosition - 700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                currState = HALT;
                nextState = END;
            }
        break;

        case END:
            chassis.driveFor(-20, 10, false);
            delay(4000);
            armstrong.moveTo(0);
            servo.writeMicroseconds(2000);
            delay(700);
            currState = HALT;
            nextState = HALT;
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

void lineFollowToHouse() {
    float difference2 = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = 10 + difference2;
    float rightSpeed = 10 - difference2;
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

// detect the cross, at which the first turn is performed, and complete the maneuver.
void crossDetected() {
    if (side45 == true && loading == false) {
        angle = 85;
        currState = HALT;
        nextState = FORTYFIVE;
    } else if (side45 == false && loading == false) {
        angle = -85;
        currState = HALT;
        nextState = TWENTYFIVE;
    } else if (side45 == true && loading == true) {
        angle = 85;
        currState = HALT;
        nextState = FOLLOWTOHOUSE;
    } else if (side45 == false && loading == true){
        angle = -85;
        currState = HALT;
        nextState = FOLLOWTOHOUSE;
    }
    chassis.driveFor(7.33, 10, true);
    chassis.turnFor(angle, 100, true);
    Serial.println("directed");
} 

// detect the cross again, and perform another maneuver.
void returnTurn(bool testing) {
    switch (testing) {
        case true:
            chassis.driveFor(7.33, 10, true);
            chassis.turnFor(-angle, 100, true);
            nextState = FOLLOWTODEPOT;
            currState = HALT;

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

void closeFork() {
    servo.writeMicroseconds(-servoMicroseconds);
    delay(2100);
    servo.detach();
}

void openFork() {
    servo.writeMicroseconds(servoMicroseconds);
    delay(2100);
    servo.detach();
}

void handleInbound(int keyPress) { 
  if (keyPress == remotePlayPause)  //This is the emergency stop button
  {
    nextState = currState;  // save current state so you can pick up where you left off
    currState = HALT;
    Serial.println("Emergency Stop");
  }

  if (keyPress == remoteRight)  // the proceed button (changed from remoteUp)
  {
    currState = nextState;
    Serial.println("Onward");
  }

  if (keyPress == remoteDown) {
    currState = FOLLOWINGLINE;
  }
  
  if (keyPress == remote1) {
    currState = FORTYFIVE;
  }

}


// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass
// gotta get that fortnite battle pass