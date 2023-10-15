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
static int leftSensor = 22;
static int rightSensor = 21;
static int echoPin = 17;
static int pingPin = 12; // this should be 12, but according to POLOLU pin 12 DNE
static int irRemotePin = 14;

// establish robot states, for the state machine setup.
// TODO: add a new state CROSSINGFIELD which makes the robot cross to the other depot and
// run that half of the code (switch side45 and loading from true to false or vice versa and enter LINEFOLLOW)
enum chassisState {LINEFOLLOWING, TOHOUSE, FOLLOWFROMHOUSE, FOLLOWTODEPOT, 
                   INTERSECT, RETURNINTERSECT, STAHP, ZERO, 
                   FORTYFIVE, TWENTYFIVE, ONEEIGHTZERO, GRAB, DROP,
                    // the next states are established to make the robot pick up the panel from the depot
                   LOADPANEL, DROPOFF, SWITCHPREP, PREPCONT,
                   // these states help the robot cross the field and get situated
                   STARTCROSS, CROSSINGFIELD, CONFIG} currState, nextState, autoState, nextAutomatic; // driving
bool side45 = true; // start on the side of the field with the 45 degree plate.
bool loading = false;
bool left = false; 

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
static double rangeThresh = 11; // centimeters
int i; // counter for for() loop
int16_t leftEncoderValue;
static int houseEncoderCount = 1000;    // previously 1138
static int depotEncoderCount = 1756;    // previously 1700
static int pickupEncoderCount = depotEncoderCount - 200;
static int spaceEncoderCount = depotEncoderCount + 200;
static int fortyfivePosition = -3050;   // encoder count required to move the arm to the 45-degree position. (2900)
static int twentyfivePosition = -3900;  // encoder count required to move the arm to the 25-degree position. (4000)
static const int servoMicroseconds = -500;
int angle;

// static int divisor = 120;
static float defaultSpeed = 15.0; // default driving speed
static float turnSpeed = 30.0;
float currRange;
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
void lineFollowSlow();      // a function which follows a line, slowly, using PID control
void lineFollowToHouse();   // similar to the previous, but performed slower
void turnUntilLine();
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
    currState = LINEFOLLOWING;  // establish initial driving state
    autoState = LINEFOLLOWING;
    //currState = LOADPANEL;        // testing only
    buttonB.waitForButton();    // wait until C is pressed to start the code.
    getLeftValue();     // reset left reflectance
    getRightValue();    // reset right reflectance
    rangefinder.getDistance();  // reset rangefinder reading
    beginning();        // perform initial maneuvers
    delay(1000);        
}

void loop() {
    // survey for an inbout remote signal
    int inboundSignal = decoder.getKeyCode();   // when true, the key can be repeated if held down
    if (inboundSignal != -1) handleInbound(inboundSignal);  // inboundSignal == -1 only when unpressed

    //Serial.println(inboundSignal);
    switch(currState) { // switching currState allows remote control, switching autoState does not.   
        case LINEFOLLOWING:
            lineFollow(); // I don't use chassis.setTwist() because it's inconsistent

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);
                nextState = INTERSECT; // save this state for later, when we advance using the remote
                currState = STAHP;           // set the current state to this, and wait for remote input
                autoState = INTERSECT;

                Serial.println("Checkpoint 1");
            }
        break;

        case INTERSECT:
            Serial.println("Check");
            crossDetected(true); // this function is essentially just a combination of state code from the example provided on Canvas.
            //turnUntilLine();
            if (currState != INTERSECT) {
                chassis.getLeftEncoderCount(true);  // reset encoder values before moving to approach the house
                chassis.getRightEncoderCount(true);
                Serial.println("leaving state INTERSECT");
            }
        break;

        case TOHOUSE: // this can EASILY *knocks on wood* be adjusted to use rangefinder
            lineFollowSlow();
            // currRange = rangefinder.getDistance();
            if (chassis.getLeftEncoderCount() >= houseEncoderCount && chassis.getRightEncoderCount() >= houseEncoderCount) {
                chassis.setWheelSpeeds(0, 0);
                if (loading == false) { // deteromine which state to switch to next via use of this boolean
                    currState = STAHP;
                    nextState = GRAB;
                    autoState = GRAB;
                    Serial.println("Checkpoint 4");
                } else {
                    currState = STAHP;
                    nextState = DROPOFF;
                    autoState = DROPOFF;
                    Serial.println("begin offload");
                }
            }
        break;
        
        case FORTYFIVE: // this state lifts the arm, and is called before the robot approaches the house
            Serial.println("sisyphus and the boulder"); // haha funny
            armstrong.moveTo(fortyfivePosition);    // using the new moveTo() function we made, move the arm.

            if (armstrong.getPosition() <= fortyfivePosition + 15) {    // if position is within acceptable threshold
                nextState = TOHOUSE;                              // continue on in state machine
                currState = STAHP;
                autoState = TOHOUSE;
                Serial.println("Checkpoint 3a");
            } 
        break;

        case TWENTYFIVE:    // identical to the above, but for the opposing side of the field
            armstrong.moveTo(twentyfivePosition);

            if (armstrong.getPosition() <= twentyfivePosition + 15) {
                nextState = TOHOUSE;
                currState = STAHP;
                autoState = TOHOUSE;
                Serial.println("Checkpoint 3b");
            }
            
        break;

        case ONEEIGHTZERO:  // this state reverses the robot facing, so it can move away from the house
            Serial.println("to ONEEIGHTZERO");
            chassis.setWheelSpeeds(-25, -25);
            delay(215);                     // wait to advance
            chassis.turnFor(178, 30, true); // turn around
            chassis.driveFor(-6, 10, true); // back up to avoid sitting on the crossroads
            nextState = FOLLOWFROMHOUSE;    // state change!
            currState = STAHP;
            autoState = FOLLOWFROMHOUSE;
            Serial.println("Spun");
        break;
        
        case FOLLOWFROMHOUSE:   // this state line follows from the house back to the lines' intersection
            lineFollow(); // I don't use chassis.setTwist() because it's cringe

            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                chassis.setWheelSpeeds(0, 0);       // stop the robot
                nextState = RETURNINTERSECT;   // state change!
                currState = STAHP;
                autoState = RETURNINTERSECT;

                Serial.println("Checkpoint 5");
                chassis.getLeftEncoderCount(true);  // reset encoder, for use in turning (we don't use it lol)
                leftEncoderValue = chassis.getLeftEncoderCount();
            }
        break;

        case RETURNINTERSECT:  // handle the detection of the cross when RTB
            Serial.println("Check");
            returnTurn(true);       // turn a certain direction according to the side of the field we're on
            // turnUntilLine();
            chassis.getLeftEncoderCount(true);  // reset encoder values so we can travel to the depot
            chassis.getRightEncoderCount(true);
        break;

        case FOLLOWTODEPOT: // follow the line for a certain distance until we've reached the depot.
            lineFollow();   // follow the line (foolish samurai worrior)
            if (chassis.getLeftEncoderCount() >= depotEncoderCount && chassis.getRightEncoderCount() >= depotEncoderCount) {   // 4 cm
                // chassis.getLeftEncoderCount() >= depotEncoderCount && chassis.getRightEncoderCount() >= depotEncoderCount
                // ragefinder.getDistance() <= 2

                chassis.setWheelSpeeds(0, 0);
                currState = STAHP;   // state change!
                nextState = ZERO;   // prepare for plate deposit
                autoState = ZERO;
                Serial.println("Checkpoint 6");
            }
        break;

        case STAHP: // remain stopped until the remote is pressed
            chassis.idle();         // idle the motors
            armstrong.setEffort(0); // cancel arm movement
            // Serial.println("Stopped");  // terminal confirmation
        break;

        case ZERO:  // lower are to the position where it will place plate at the depot
            Serial.println("depositing");
            armstrong.moveTo(0);    // move the arm to the desired position (blocking)

            if (armstrong.getPosition() >= -5) {    // if position within acceptable range
                nextState = DROP;                   // change states
                currState = STAHP;                   // and stop all movement
                autoState = DROP;
                Serial.println("Checkpoint 3a");
            }
        break;

        case GRAB:  // TODO: fix servo so that it knows when to close.
            servo.writeMicroseconds(1000);      // 1000 = retract the servo
            delay(800);                         // wait for it to come back
            servo.detach();                     // cancel servo motion

            chassis.driveFor(7.1, 15, true);    // this is part of the gripping process
        
            servo.writeMicroseconds(1750);      // close servo
            delay(1400);                         // for this amount of time
            servo.detach();                     // cancel motion
            delay(500);                         // this is not necessary, but helps distinguish movements when debugging

            if (side45 == true) {   // if on the side of the 45 degree roof, do this
                armstrong.moveTo(fortyfivePosition - 800);  // lift to this position to avoid roof contact
                delay(100);
                chassis.driveFor(3.7, 8, true);             // ensure plate is within griper constraints
                delay(10);
                armstrong.moveTo(fortyfivePosition - 1800); // lift away from roof pegs
                delay(100);
                chassis.driveFor(1.2, 8, true);               // i forgot what this does, and I'm too tired to check
                nextState = ONEEIGHTZERO;   // state change!
                currState = STAHP;
                autoState = ONEEIGHTZERO;
            } else {  // if on the side of the 25 degree roof, do this
                armstrong.moveTo(twentyfivePosition - 800); // same deal as before
                delay(100);
                Serial.print("1 ");
                chassis.driveFor(2.4, 8, true);
                delay(10);
                Serial.println("2");
                armstrong.moveTo(twentyfivePosition - 1500);
                nextState = ONEEIGHTZERO;   // state change!
                currState = STAHP;
                autoState = ONEEIGHTZERO;
                Serial.print("State change: GRAB ");
            }
        break;

        case DROP:  // this state will deposit the solar panel at the depot.
            servo.writeMicroseconds(1000);  // retract
            delay(700);                     // wait...
            servo.detach();                 // done!

            delay(10);
            chassis.driveFor(-30, 10, true);// prepare for restart
            armstrong.moveTo(100);

            servo.writeMicroseconds(2000);
            delay(700);
            servo.detach();
            
            loading = true;                 // IMPORTANT: this change will determine code path for loading panel
            nextState = LOADPANEL;          // state change, the first of loading panel
            currState = STAHP;
            autoState = LOADPANEL;
        break;

        case LOADPANEL: // this state prepares the robot for loading a panel onto the roof
        // IMPORTANT: this design struggles to pick up the panel on its own. In order to have it work
        // in the video you guys have seen, I have to hold the plate and depot so the fork could get
        // between them without pushing them away.
            lineFollowSlow();
            if (chassis.getLeftEncoderCount() >= pickupEncoderCount && chassis.getRightEncoderCount() >= pickupEncoderCount) {
                chassis.setWheelSpeeds(0, 0);   // arrive at depot for loading
                delay(20);

                if (side45 == true) armstrong.moveTo(fortyfivePosition - 700);
                else armstrong.moveTo(twentyfivePosition - 700);

                chassis.driveFor(-5, 12, true); // back away from depot
                chassis.turnFor(175, turnSpeed, true); // and turn around

                nextState = LINEFOLLOWING;      // state change!
                currState = STAHP;
                autoState = LINEFOLLOWING;
            }

        break;
        
        case DROPOFF:   // this state will deposit the panel onto the roof
            if (side45 == true) {   // check that the arm is raising to correct positions out of load
                armstrong.moveTo(fortyfivePosition - 2200); // begin unloading sequence for 45 degree roof
                delay(10);
                chassis.driveFor(7.9, 10, true);    // initial guess was 6.9 (nice!)
                delay(100);
                armstrong.moveTo(fortyfivePosition - 1700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                armstrong.moveTo (fortyfivePosition - 1100);
                nextState = SWITCHPREP;    // state change!
                currState = STAHP;
                autoState = SWITCHPREP;
            } else {
                armstrong.moveTo(twentyfivePosition - 1200);// begin unloading sequence for 25 degree roof
                delay(10);
                chassis.driveFor(10.2, 10, true);
                delay(100);
                armstrong.moveTo(twentyfivePosition - 700);
                delay(500);
                servo.writeMicroseconds(1000);
                delay(700);
                servo.detach();
                nextState = SWITCHPREP;    // state change!
                currState = STAHP;
                autoState = SWITCHPREP;
            }
        break;

        case SWITCHPREP:    // this state prepares the robot for transfer between 
            armstrong.moveTo(fortyfivePosition-950);
            chassis.setWheelSpeeds(-10, -10);
            delay(1200);                         // wait to advance
            chassis.turnFor(178, turnSpeed, true);     // turn around
            chassis.driveFor(-6, 10, false);    // back up a bit
            delay (300);
            servo.writeMicroseconds(2000);      // reset arm and servo position
            delay(700);
            servo.detach();
            delay(20);
            armstrong.moveTo(0);
            nextState = STARTCROSS;
            currState = STAHP;
            autoState = STARTCROSS;
        break;

        case STARTCROSS: // this state is where the robot starts to traverse the field (very creative nomenclature ik)
            lineFollow();
                if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) { // this statement is true only when Romi detects the crossroads
                    chassis.driveFor(7.33, 10, true);
                    chassis.turnFor(-angle, turnSpeed, true);

                    currState = STAHP;
                    nextState = PREPCONT;
                    autoState = PREPCONT;
                }
        break;

        case PREPCONT:
            lineFollow();
                if (chassis.getLeftEncoderCount() >= (spaceEncoderCount) && chassis.getRightEncoderCount() >= (spaceEncoderCount)) {
                    chassis.turnFor(-angle, turnSpeed, true);
                    currState = STAHP;
                    nextState = CROSSINGFIELD;
                    autoState = CROSSINGFIELD;
                }
        break;
        case CROSSINGFIELD: // woo yeah baby cross that shit
            chassis.setWheelSpeeds(defaultSpeed, defaultSpeed);
            if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) {
                chassis.setWheelSpeeds(0, 0);
                delay(100);
                chassis.driveFor(7.75, 25, true);
                chassis.turnFor(angle, turnSpeed, true);
                if (side45 == true) side45 = false;     // if we were on the 45 previously, we're not now
                else side45 = true;                     // if we weren't on the 45 previously, we are now
                loading = false;                            // we will begin operating here by removing the panel from the roof
                nextState = CONFIG;
                currState = STAHP;           // boom-bam, infinite state machine achieved
                autoState = CONFIG;
                // field crossed. this code wil now repeat until failure
            } 
        break;

        case CONFIG:
            if (side45 == false) {
                chassis.setWheelSpeeds(-25, 25);
                if (getRightValue() > lineSensingThresh) {
                    chassis.setWheelSpeeds(0, 0);
                    
                    currState = STAHP;
                    nextState = LINEFOLLOWING;
                    autoState = LINEFOLLOWING;
                    delay(2000);
                    beginning();
                }

            } else if (side45 == true) {
                chassis.setWheelSpeeds(25, 25);
                if (getLeftValue() > lineSensingThresh) {
                    chassis.setWheelSpeeds(0, 0);

                    currState = STAHP;
                    nextState = LINEFOLLOWING;
                    autoState = LINEFOLLOWING;
                    delay(2000);
                    beginning();
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
    chassis.turnFor(30, turnSpeed, true);
    delay(300);
    while (getRightValue() < lineSensingThresh) {
        chassis.setWheelSpeeds(-15, 15);
        if (getRightValue() >= lineSensingThresh) break;
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

void lineFollowSlow() {
    float difference2 = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = 10 + difference2;
    float rightSpeed = 10 - difference2;
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

void lineFollowToHouse() {
    float difference3 = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = 10  + difference3 - (30 / rangefinder.getDistance());
    float rightSpeed = 10 - difference3 - (30 / rangefinder.getDistance());
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

void turnUntilLine() {
    if (left == true) {
        while (getRightValue() < lineSensingThresh) {
        chassis.setWheelSpeeds(-20, 20);
            if (getRightValue() >= lineSensingThresh) break;
        }
    } else if (left == false) {
        while (getLeftValue() < lineSensingThresh) {
            chassis.setWheelSpeeds(20, 20);
            if (getLeftValue() >= lineSensingThresh) break;
        }
    }
    
    chassis.idle();
}

// detect the cross, at which the first turn is performed, and complete the maneuver.
void crossDetected(bool testing) {
    if (side45 == true && loading == false) {
        angle = 85;
        left = true;

        currState = STAHP;
        nextState = FORTYFIVE;
        autoState = FORTYFIVE;
    } else if (side45 == false && loading == false) {
        angle = -85;
        left = false;

        currState = STAHP;
        nextState = TWENTYFIVE;
        autoState = TWENTYFIVE;
    } else if (side45 == true && loading == true) {
        angle = 85;
        left = true;

        currState = STAHP;
        nextState = TOHOUSE;
        autoState = TOHOUSE;
    } else if (side45 == false && loading == true){
        angle = -85;
        left = false;

        currState = STAHP;
        nextState = TOHOUSE;
        autoState = TOHOUSE;
    }

    if (testing == true) {
        chassis.driveFor(7.75, 10, true);
        chassis.turnFor(angle, turnSpeed, true);
        Serial.println("directed");
    }
} 

// detect the cross again, and perform another maneuver.
void returnTurn(bool testing) {
    switch (testing) {
        case true:
            chassis.driveFor(7.75, 10, true);
            chassis.turnFor(-angle, turnSpeed, true);
            nextState = FOLLOWTODEPOT;
            currState = STAHP;
            autoState = FOLLOWTODEPOT;

        break;

        case false:
        //  do nothing
        break;
    }
}

void handleInbound(int keyPress) { 
  if (keyPress == remoteLeft) //This is the emergency stop button
  {
    nextState = currState;  // save current state so you can pick up where you left off
    nextAutomatic = autoState;
    currState = STAHP;
    autoState = STAHP;
    if (nextState == currState) Serial.print("stopped during STAHP. restart required.");
    Serial.println("Emergency Stop");
  }

  if (keyPress == remote5)  // the proceed button (changed from remoteUp)
  {
    currState = nextState;
    autoState = nextAutomatic;
    Serial.print("(state change) ");
  }

  if (keyPress == remoteDown) {
    currState = LINEFOLLOWING;
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