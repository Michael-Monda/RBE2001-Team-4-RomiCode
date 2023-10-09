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
    long diff;
    long kp = 10;
    count = getPosition();

    while(count > target + tolerance){ //move in negative direction
        diff = target - count;
        count = getPosition();
        setEffortWithoutDB(kp * diff);
        Serial.println(count);

    }while(count < target - tolerance){ //move in positive direction
        diff = target - count;
        count = getPosition();
        setEffortWithoutDB(kp * diff);
        Serial.println(count);
    }
    setEffort(0);
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
