#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void moveTo(long position);
    long getPosition();
    void reset();
    void setup();
    void setEffortWithoutDB(int effort);
    void gradualEffort(int effort, int adjEffort);
    long getCount();
    void setEffortWithDB(int effort, bool clockwise, float deadband);

    // deadband variables
    float angularSpeed;
    float currPosition;
    float pastPosition;
    float currTime;
    float pastTime;
    float changeDirection;
    float adjustedEffort;
    float deadband;

    float defaultSpeed = 275;
    float kp = 2; // proportional gain for motor PID
    const int limitPin = 6;
    int bottomCCW;
    int bottomCW;

private:
    static void isrA();
    static void isrB();
    void setEffort(int effort, bool clockwise);
    static void isr();
    const long tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    static const int ENCA = 0;
    static const int ENCB = 1;
};