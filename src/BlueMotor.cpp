#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
long count;
int DBPOS = 251;
int DBNEG = -218;
long time;
float angSpeed = 0.0;
long target;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;
    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isrA()
{
    if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count--;
  }
  else {
    count++;
  }
}

void BlueMotor::isrB()
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count++;
  }
  else {
    count--;
  }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified limit
{     
    int16_t currPos = getPosition(); // initialize the current position
    float direction = (target - currPos) / abs(target - currPos); // specifies direction motor needs to move in.

    while (abs(target-currPos) > 6) { // while motor position is outside the desired position (thresh)
        float error = target -  currPos; // get error
        float effort = (error*kp) + (direction*defaultSpeed); // establish effort via PID
        setEffortWithoutDB(effort); // set the established effort

        currPos = getPosition(); // update position
    }

    setEffort(0); // stop so arm is in correct position for grabbing
}

long oldCount;
long oldTime;

void BlueMotor::gradualEffort(int effort, int adjEffort){
    count = 0;
    effort = 0;
    for(int effort = 0; effort <= 400; effort++){
        setEffort(effort);
        delay(10);
    }
    Serial.println(count);
    Serial.print(effort);

}


void BlueMotor::setEffortWithoutDB(int effort){
    oldCount = 0;

    if(effort > 0){ //if effort is positive
        int adjEffort = (effort * 0.373) + DBPOS;
        for(int temp = 0; temp <= effort; temp++){ //gradually increase effort over non-deadband
            adjEffort = (temp * 0.373) + DBPOS; //calculate adjusted effort each loop
            setEffort(adjEffort);
            oldTime = time;
            time = millis();
            float dcount = count - oldCount;
            float dtime = time- oldTime;
            angSpeed = ((dcount /540) *360) / dtime; 

            Serial.println(count);
            // Serial.print(time);
            // Serial.print("   ");
            // Serial.print(temp);
            // Serial.print("   ");
            // Serial.print(adjEffort);
            // Serial.print("   ");
            // Serial.println(angSpeed,3);

            oldCount = count;
            delay(100);

        }

    }else if (effort < 0){ //if effort is negative
        int adjEffort = (effort * 0.455) + DBNEG; 
        for(int temp = 0; temp >= effort; temp--){ //gradually increase effort over non-deadband
            adjEffort = (temp * 0.455) + DBNEG; //calculate adjusted effort each loop
            setEffort(adjEffort);
            oldTime = time;
            time = millis();
            float dcount = count - oldCount;
            float dtime = time- oldTime;
            angSpeed = ((dcount /540) *360) / dtime;

            Serial.println(count);
            // Serial.print(time);
            // Serial.print("   ");
            // Serial.print(temp);
            // Serial.print("   ");
            // Serial.print(adjEffort);
            // Serial.print("   ");
            // Serial.println(angSpeed, 3);
            
            oldCount = count;
            delay(100);
        }

    }

}

void BlueMotor::setEffortWithDB(int effort, bool clockwise, float deadband) {
    float b = deadband;

    // account for directional change
    if (clockwise == true) {
        changeDirection = 1.00;
    } else {
        changeDirection = -1.00;
    }

    currPosition = getPosition();
    currTime = millis() / 1000.00;
    adjustedEffort = ((400*changeDirection - deadband) / (400*changeDirection) * (effort*changeDirection)) + b; // deadband slope
    angularSpeed = (currPosition - pastPosition) / (currTime - pastTime); // angular velocity in radians per second
    setEffort(effort*changeDirection);
    delay(15);

    // print data in a "graph" for the deadband portion of the lab assignment
    Serial.print(currTime);
    Serial.print(", ");
    Serial.print(effort*changeDirection);
    Serial.print(", ");
    Serial.print(adjustedEffort);
    Serial.print(", ");
    Serial.print(angularSpeed);
    Serial.print(", ");
    Serial.println(getPosition());
    pastPosition = currPosition;
    pastTime = currTime;
}
