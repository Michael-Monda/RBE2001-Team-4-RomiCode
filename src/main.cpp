// @author Michael J Monda.

// class imports.
#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <PIDController.h>
#include <Rangefinder.h>

// Chassis, button, and rangefinder object creation.
Chassis chassis;
Romi32U4ButtonB buttonB;
Rangefinder rangefinder(17, 12);
// sensor port numbers.
int leftSensor = 22;
int rightSensor = 21;
int echoPin = 17;
int pingPin = 12;
// variable declarations here.
int leftSensVal;
int rightSensVal;
int leftSpeed;
int rightSpeed;
<<<<<<< Updated upstream
int divisor = 120;
int lineSensingThresh = 250; // < 250 == white; > 250 == black.
int rangeThreshold = 12.7; // centimeters
int i;
=======
int lineSensingThresh = 250; // < 250 == white, > 250 == black
// static double rangeThreshold = 12.7; // centimeters
int i; // counter for for() loop
int16_t leftEncoderValue;
static int houseEncoderCount = 1138;
static int depotEncoderCount = 1700;
static int fortyfivePosition = 2676; // encoder count required to move the arm to the 45-degree position.
static int twentyfivePosition = 4452; // encoder count required to move the arm to the 25-degree position.
bool grabbed = false;
int servoExtend = 2100;
int servoRelease = -2100;

// static int divisor = 120;
static float defaultSpeed = 20.0; // default driving speed
static float constant = 0.01; // proportional gain for the controller function lineFollow()
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
void crossDetected();
void returnTurn();

=======
void crossDetected(bool);
void returnTurn(bool);
void handleInbound(int);
void deadBand();
void deadBandClocwise();
void deadBandAnticlockwise();
void closeFork();
void openFork();
>>>>>>> Stashed changes

// configure the robot setup.
void setup() {
    Serial.begin(9600);
    chassis.init();
    buttonB.waitForButton();
    rangefinder.init();
    delay(1000);

}

// main code loop.
void loop() {
    // reset the light sensor.
    getLeftValue();
    getRightValue();
    delay(100);
    beginning();

    // when programming this assignment, I found it much more digestible to create "blocks" of code,
    // in which each block is a while loop with an exit condition blocking the following loop. This
    // first block will follow the line until both sensors read black, upon which it will perform a
    // required maneuver before continuing on to the next block.
    while (getLeftValue() < lineSensingThresh || getRightValue() < lineSensingThresh) {
        lineFollow();
        if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) {
            delay(100);
            crossDetected();
            break;
        }
    }

    // wait a moment before continuing. 
    delay(100);
    rangefinder.getDistance();
    delay(50);
    // enter the second block of code, in which the robot will travel forward until the sonar
    // object returns a specific distance. This is very simply achieved by just setting the
    // condition for the while loop and its associated if statement for when the value < 5".
    while (rangefinder.getDistance() > rangeThreshold) {
        lineFollow();
        if (rangefinder.getDistance() <= rangeThreshold) {
            break;
        }
    }
    // stop the robot.
    chassis.idle();
    delay(200);
    // do a turnaround and prep to go back.
    chassis.turnFor(170, -100, true);

    // enter the third block, which is identical to the first save for a turn in the opposite direction.
    while (getLeftValue() < lineSensingThresh || getRightValue() < lineSensingThresh){
        lineFollow();
        if (getRightValue() > lineSensingThresh && getLeftValue() > lineSensingThresh) {
            returnTurn();
            break;
        }
    }

    // reset encoders again, so we can control where we stop with the robot. The goal is to be in front of
    // the "staging area"
    delay(100);
    chassis.getLeftEncoderCount(true);
    chassis.getRightEncoderCount(true);
    while (chassis.getLeftEncoderCount() < 1700 || chassis.getRightEncoderCount() < 1700) {
        lineFollow();
        if (chassis.getLeftEncoderCount() >= 1700 && chassis.getRightEncoderCount() >= 1700) {
            break;
        }
    }
    // stop the robot, and remain standing.
    while(true) {
        chassis.idle();
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
    float defaultSpeed = 35.0;
    float constant = 0.01;
    float difference = (getRightValue() - getLeftValue()) * constant;
    float leftSpeed = defaultSpeed + difference;
    float rightSpeed = defaultSpeed - difference;
    chassis.setWheelSpeeds(leftSpeed, rightSpeed);
}

// detect the cross, at which the first turn is performed, and complete the maneuver.
void crossDetected() {
    chassis.driveFor(7, 10, true);
    chassis.turnFor(-90, 100, true);
}
// detect the cross again, and perform another maneuver.
<<<<<<< Updated upstream
void returnTurn() {
    chassis.driveFor(7.33, 10, true);
    chassis.turnFor(90, 100, true);
=======
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
>>>>>>> Stashed changes
}