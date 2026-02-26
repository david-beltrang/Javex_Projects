#pragma once
#include "vex.h"
#include <vector>
#include <functional>

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

const double WHEEL_DIAMETER = 3.65;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

// Declaración de dispositivos
extern brain Brain;
extern controller Controller;

extern inertial inertialSensor;

extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;
extern motor LeftMotor4;
extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;
extern motor RightMotor4;
extern motor Recolector1;
extern motor Recolector2;
extern motor Recolector3;
extern motor Recolector4;
extern motor Recolector5;


extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor_group Recoleccion;

struct PID
{
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    double error = 0;
    double prevError = 0;

    double integral = 0;
    double derivative = 0;  // filtered derivative

    double integralLimit = 50;  // anti-windup clamp
};