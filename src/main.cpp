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

// sensor port numbers, static so they don't get reassigned values on accident
static int leftSensor = 21;
static int rightSensor = 22;
static int echoPin = 17;
static int pingPin = 5; // this should be 12, but according to POLOLU pin 12 DNE
static int irRemotePin = 14;

// establish robot states, for the state machine setup.
// TODO: add a new state CROSSINGFIELD which makes the robot cross to the other depot and
// run that half of the code (switch side45 and loading from true to false or vice versa and enter LINEFOLLOW)
enum chassisState {FOLLOWINGLINE, FOLLOWTOHOUSE, FOLLOWFROMHOUSE, FOLLOWTODEPOT, 
                   CROSSDETECTION, RETURNCROSSDETECTION, HALT, ZERO, 
                   FORTYFIVE, TWENTYFIVE, ONEEIGHTZERO, GRAB, DROP,
                    // the next states are established to make the robot pick up the panel from the depot
                   LOADPANEL, DROPOFF, SWITCHPREP, 
                   // these states help the robot cross the field and get situated
                   STARTCROSS, CROSSINGFIELD} currState, nextState; // driving
bool side45 = true; // start on the side of the field with the 45 degree plate.
bool loading = false;  

// chassis, startup button, rangefinder and remote object creation.
Chassis chassis;
Romi32U4ButtonC buttonC;    // used to start the robot in the void setup()
Romi32U4ButtonB buttonB;    // unused as of 10/10/2023
Romi32U4ButtonA buttonA;    // unused as of 10/10/2023
Rangefinder rangefinder(echoPin, pingPin);    // rangefinder pin declarations
IRDecoder decoder(irRemotePin); // ir decoder pin declaration
BlueMotor armstrong;
Servo32U4 servo;

// variable declarations here.
int leftSensVal;    // used in line following
int rightSensVal;   // used in line following
int leftSpeed;      // used in line following -> setting speeds
int rightSpeed;     // used in line following -> setting speeds

// the following variables are used to create a state-machine where no blocking code is present.
// due to the present conditions, converting this code to that format has been ignored.
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
static int houseEncoderCount = 1138;    // previously 1138
static int depotEncoderCount = 1756;    // previously 1700
static int fortyfivePosition = -3300;   // encoder count required to move the arm to the 45-degree position. (2900)
static int twentyfivePosition = -3900;  // encoder count required to move the arm to the 25-degree position. (4000)
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
int getLeftValue();         // obtain the value of the left reflectance sensor
int getRightValue();        // obtain the value of the right reflectance sensor
void beginning();           // a function which turns the robot until it detects a line; called once only
void lineFollow();          // a function which follows a line utilizing PID control
void lineFollowToHouse();   // similar to the previous, but performed slower
void crossDetected(bool);   // a handler for if the reflectance sensors detect a crossroads
void returnTurn(bool);      // similar to the previous, with turn direction reversed
void handleInbound(int);    // a function which accepts an incoming signal from the infrared controller

// configure the robot
void setup() {
    chassis.init();         // start chassis
    decoder.init();         // initialize ir decoder
    rangefinder.init();     // initialize rangefinder
    armstrong.setup();      // set up blue motor "armstrong"
    armstrong.reset();      // reset armstrong encoder
    servo.setMinMaxMicroseconds(1000, 2000);  // limit servo movement
    pinMode(irRemotePin, INPUT);    // create reciever pin
    Serial.begin(9600);
    currState = FOLLOWINGLINE;  // establish initial driving state
    // currState = GRAB;        // testing only
    buttonC.waitForButton();    // wait until C is pressed to start the code.
    getLeftValue();     // reset left reflectance
    getRightValue();    // reset right reflectance
    beginning();        // perform initial maneuvers
    delay(1000);        
}

void loop() {
    // survey for an inbout remote signal
    int inboundSignal = decoder.getKeyCode();   // when true, the key can be repeated if held down
    if (inboundSignal != -1) handleInbound(inboundSignal);  // inboundSignal == -1 only when unpressed
    // Serial.println(currState)
    switch(currState) {      
        case FOLLOWINGLINE:
            lineFollow(); // I don't use chassis.setTwist() because it's inconsistent

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);
                nextState = CROSSDETECTION; // save this state for later, when we advance using the remote
                currState = HALT;           // set the current state to this, and wait for remote input

                Serial.println("Checkpoint 1");
            }
        break;

        case CROSSDETECTION:
            Serial.println("Check");
            crossDetected(true); // this function is essentially just a combination of state code from the example provided on Canvas.
            if (currState != CROSSDETECTION) {
                chassis.getLeftEncoderCount(true);  // reset encoder values before moving to approach the house
                chassis.getRightEncoderCount(true);
            }
        break;

        case FOLLOWTOHOUSE: // this can EASILY *knocks on wood* be adjusted to use rangefinder
            lineFollowToHouse();
            if (chassis.getLeftEncoderCount() >= houseEncoderCount || chassis.getRightEncoderCount() >= houseEncoderCount) {
                chassis.setWheelSpeeds(0, 0);
                if (loading == false) { // deteromine which state to switch to next via use of this boolean
                    currState = HALT;
                    nextState = GRAB;
                    Serial.println("Checkpoint 4");
                } else if (loading == true) {
                    currState = HALT;
                    nextState = DROPOFF;
                    Serial.println("begin offload");
                }
            }
        break;
        
        case FORTYFIVE: // this state lifts the arm, and is called before the robot approaches the house
            Serial.println("sisyphus and the boulder"); // haha funny
            armstrong.moveTo(fortyfivePosition);    // using the new moveTo() function we made, move the arm.

            if (armstrong.getPosition() >= fortyfivePosition - 15) {    // if position is within acceptable threshold
                nextState = FOLLOWTOHOUSE;                              // continue on in state machine
                currState = HALT;
                Serial.println("Checkpoint 3a");
            } 
        break;

        case TWENTYFIVE:    // identical to the above, but for the opposing side of the field
            armstrong.moveTo(twentyfivePosition);

            if (armstrong.getPosition() >= twentyfivePosition - 15) {
                nextState = FOLLOWTOHOUSE;
                currState = HALT;
                Serial.println("Checkpoint 3b");
            }
            
        break;

        case ONEEIGHTZERO:  // this state reverses the robot facing, so it can move away from the house
            chassis.setWheelSpeeds(-25, -25);
            delay(215);                     // wait to advance
            chassis.turnFor(170, 25, true); // turn around
            chassis.driveFor(-6, 10, true); // back up to avoid sitting on the crossroads
            nextState = FOLLOWFROMHOUSE;    // state change!
            currState = HALT;
            Serial.println("Spun");
        break;
        
        case FOLLOWFROMHOUSE:   // this state line follows from the house back to the lines' intersection
            lineFollow(); // I don't use chassis.setTwist() because it's cringe

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);       // stop the robot
                nextState = RETURNCROSSDETECTION;   // state change!
                currState = HALT;
                Serial.println("Checkpoint 5");
                chassis.getLeftEncoderCount(true);  // reset encoder, for use in turning (we don't use it lol)
                leftEncoderValue = chassis.getLeftEncoderCount();
            }
        break;

        case RETURNCROSSDETECTION:  // handle the detection of the cross when RTB
            Serial.println("Check");
            returnTurn(true);       // turn a certain direction according to the side of the field we're on
            chassis.getLeftEncoderCount(true);  // reset encoder values so we can travel to the depot
            chassis.getRightEncoderCount(true);
        break;

        case FOLLOWTODEPOT: // follow the line for a certain distance until we've reached the depot.
            lineFollow();   // follow the line (foolish samurai worrior)
            if (chassis.getLeftEncoderCount() >= depotEncoderCount && chassis.getRightEncoderCount() >= depotEncoderCount) {
                chassis.setWheelSpeeds(0, 0);
                currState = HALT;   // state change!
                nextState = ZERO;   // prepare for plate deposit
                Serial.println("Checkpoint 6");
            }
        break;

        case HALT: // remain stopped until the remote is pressed
            chassis.idle();         // idle the motors
            armstrong.setEffort(0); // cancel arm movement
            servo.detach();         // detach servo from input signal to prevent undesired movement
            Serial.println("Stopped");  // terminal confirmation
        break;

        case ZERO:  // lower are to the position where it will place plate at the depot
            Serial.println("depositing");
            armstrong.moveTo(0);    // move the arm to the desired position (blocking)

            if (armstrong.getPosition() <= 15) {    // if position within acceptable range
                nextState = DROP;                   // change states
                currState = HALT;                   // and stop all movement
                Serial.println("Checkpoint 3a");
            }
        break;

        case GRAB:  // TODO: fix servo so that it knows when to close.
            servo.writeMicroseconds(1000);      // 1000 = retract the servo
            delay(800);                         // wait for it to come back
            servo.detach();                     // cancel servo motion

            chassis.driveFor(5.7, 15, true);    // this is part of the gripping process
        
            servo.writeMicroseconds(2000);      // close servo
            delay(700);                         // for this amount of time
            servo.detach();                     // cancel motion
            delay(500);                         // this is not necessary, but helps distinguish movements when debugging

            if (side45 == true) {   // if on the side of the 45 degree roof, do this
                armstrong.moveTo(fortyfivePosition - 800);  // lift to this position to avoid roof contact
                delay(100);
                chassis.driveFor(1.9, 8, true);             // ensure plate is within griper constraints
                delay(10);
                armstrong.moveTo(fortyfivePosition - 1500); // lift away from roof pegs
                delay(100);
                chassis.driveFor(3, 8, true);               // i forgot what this does, and I'm too tired to check
                delay(10);
            } else {  // if on the side of the 25 degree roof, do this
                armstrong.moveTo(twentyfivePosition - 800); // same deal as before
                delay(100);
                chassis.driveFor(1.9, 8, true);
                delay(10);
                armstrong.moveTo(twentyfivePosition - 1500);
            }
            nextState = ONEEIGHTZERO;   // state change!
            currState = HALT;
        break;

        case DROP:  // this state will deposit the solar panel at the depot.
            servo.writeMicroseconds(1000);  // retract
            delay(700);                     // wait...
            servo.detach();                 // done!

            delay(10);
            chassis.driveFor(-30, 10, true);// prepare for restart
            
            loading = true;                 // IMPORTANT: this change will determine code path for loading panel
            nextState = LOADPANEL;          // state change, the first of loading panel
            currState = HALT;
        break;

        case LOADPANEL: // this state prepares the robot for loading a panel onto the roof
        // IMPORTANT: this design struggles to pick up the panel on its own. In order to have it work
        // in the video you guys have seen, I have to hold the plate and depot so the fork could get
        // between them without pushing them away.
            lineFollow();
            if (chassis.getLeftEncoderCount() >= depotEncoderCount + 100 && chassis.getRightEncoderCount() >= depotEncoderCount + 100) {
                chassis.setWheelSpeeds(0, 0);   // arrive at depot for loading
                servo.writeMicroseconds(2000);  // extend servo
                delay (700);
                servo.detach();                 // plate held
                delay(20);

                if (side45 == true) armstrong.moveTo(fortyfivePosition - 700);
                else armstrong.moveTo(twentyfivePosition - 700);

                chassis.driveFor(-5, 12, true); // back away from depot
                chassis.turnFor(175, 20, true); // and turn around

                nextState = FOLLOWINGLINE;      // state change!
                currState = HALT;
            }

        break;
        
        case DROPOFF:   // this state will deposit the panel onto the roof
            if (side45 == true) {   // check that the arm is raising to correct positions out of load
                armstrong.moveTo(fortyfivePosition - 2200); // begin unloading sequence for 45 degree roof
                delay(10);
                chassis.driveFor(7.7, 10, true);    // initial guess was 6.9 (nice!)
                delay(100);
                armstrong.moveTo(fortyfivePosition - 1700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                nextState = SWITCHPREP;    // state change!
                currState = HALT;
            } else {
                armstrong.moveTo(twentyfivePosition - 1200);// begin unloading sequence for 25 degree roof
                delay(10);
                chassis.driveFor(7.7, 10, true);
                delay(100);
                armstrong.moveTo(twentyfivePosition - 700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                nextState = SWITCHPREP;    // state change!
                currState = HALT;
            }
        break;

        case SWITCHPREP:    // this state prepares the robot for transfer between 
            chassis.setWheelSpeeds(-25, -25);
            delay(215);                         // wait to advance
            chassis.turnFor(170, 25, true);     // turn around
            chassis.driveFor(-6, 10, false);    // back up a bit
            delay (300);
            servo.writeMicroseconds(2000);      // reset arm and servo position
            delay(700);
            servo.detach();
            delay(20);
            armstrong.moveTo(0);
            nextState = STARTCROSS;
            currState = HALT;
        break;

        case STARTCROSS: // this state is where the robot starts to traverse the field (very creative nomenclature ik)
            lineFollow();
                if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                    chassis.turnFor(-angle, 15, true);
                    currState = CROSSINGFIELD;  // no button input needed
                }
        break;

        case CROSSINGFIELD: // woo yeah baby cross that shit
            lineFollow();
            if (chassis.getLeftEncoderCount() >= depotEncoderCount/2 && chassis.getRightEncoderCount() >= depotEncoderCount/2) {
                chassis.turnFor(-angle, 15, true);
                chassis.setWheelSpeeds(25, 25);
                if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) {
                    chassis.setWheelSpeeds(0, 0);
                    delay(100);
                    chassis.turnFor(-angle, 15, true);
                    if (side45 == true) side45 = false;     // if we were on the 45 previously, we're not now
                    else side45 = true;                     // if we weren't on the 45 previously, we are now
                    loading = false;                        // we will begin operating here by removing the panel from the roof
                    nextState = FOLLOWINGLINE;
                    currState = HALT;           // boom-bam, infinite state machine achieved
                    // field crossed. this code wil now repeat until failure
                }
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

void lineFollowToHouse() {
    float difference2 = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = 10 + difference2;
    float rightSpeed = 10 - difference2;
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

// detect the cross, at which the first turn is performed, and complete the maneuver.
void crossDetected(bool testing) {
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

    if (testing == true) {
        chassis.driveFor(7.33, 10, true);
        chassis.turnFor(angle, 100, true);
        Serial.println("directed");
    } else {
        chassis.setWheelSpeeds(defaultSpeed/2, defaultSpeed/2);
            if (leftEncoderValue > 300) {
                chassis.setWheelSpeeds(0, 0);
                delay(100);
                chassis.setWheelSpeeds(25, -25);
                if (getLeftValue() > lineSensingThresh) {
                    nextState = FOLLOWTODEPOT;
                    currState = HALT;
                    Serial.println("directed via sensor bus");
                }
            }
    }
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

void handleInbound(int keyPress) { 
  if (keyPress == remotePlayPause)  //This is the emergency stop button
  {
    nextState = currState;  // save current state so you can pick up where you left off
    currState = HALT;
    if (nextState == currState) Serial.print("stopped during halt. restart required.");
    Serial.println("Emergency Stop");
  }

  if (keyPress == remoteRight)  // the proceed button (changed from remoteUp)
  {
    currState = nextState;
    Serial.println("Forward");
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