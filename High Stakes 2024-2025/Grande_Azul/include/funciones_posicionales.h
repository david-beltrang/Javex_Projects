#include "vex.h"
#include <vector>
#include <functional>
#include <iostream>

#pragma once // Para evitar errores de definición múltiple

#include "configuration.h" // Incluimos la configuración de variables

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

using std::cout;
using std::endl;

//---------------- FUNCIONES DE MOVIMIENTO ----------------

void resetEncoders() {
    MotorL1.setPosition(0, rotationUnits::deg);
    MotorL2.setPosition(0, rotationUnits::deg);
    MotorL3.setPosition(0, rotationUnits::deg);
    MotorL4.setPosition(0, rotationUnits::deg);
    MotorR1.setPosition(0, rotationUnits::deg);
    MotorR2.setPosition(0, rotationUnits::deg);
    MotorR3.setPosition(0, rotationUnits::deg);
    MotorR4.setPosition(0, rotationUnits::deg);
}

void setLeftMotors(double speed) {
    MotorL1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void setRightMotors(double speed) {
    MotorR1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors() {
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorL4.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
    MotorR4.stop();
    Recolector.stop();
    Rampa.stop();
    //Garra.stop();
}

void moveDistance(double distanceInInches, double speed) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveDistanceV(double distanceInInches, double speed, double velocidad) {
    resetEncoders();

    Recolector.spin(reverse, velocidad, percent);
    Rampa.spin(reverse, velocidad, percent);

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolic(double distanceInInches, double speedleft, double speedRight) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolicV(double distanceInInches, double speedleft, double speedRight, double speed) {
    resetEncoders();

    Recolector.spin(reverse, speed, percent);
    Rampa.spin(reverse, speed, percent);

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolicN(double distanceInInches, double speedleft, double speedRight, double speed) {
    resetEncoders();

    Recolector.spin(reverse, speed, percent);
    Rampa.spin(reverse, speed, percent);
    RecolectorNeumatica.open();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;
    
    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolicNC(double distanceInInches, double speedleft, double speedRight, double speed) {
    resetEncoders();

    Recolector.spin(reverse, speed, percent);
    Rampa.spin(reverse, speed, percent);

    RecolectorNeumatica.close();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    
    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void rotateOnAxis(double angleInDegrees, double speed) {
    resetEncoders();
    double targetRotations = (angleInDegrees / 360) * (TRACK_WIDTH * M_PI / WHEEL_DIAMETER) * 360;

    setLeftMotors((angleInDegrees > 0) ? speed : -speed);
    setRightMotors((angleInDegrees > 0) ? -speed : speed);

    while ( (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations) && (fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) ) {
        task::sleep(10);
    }
    stopAllMotors();
}

void rotateOnAxisB(double angleInDegrees, double speed) {
    resetEncoders();

    double targetRotations = (angleInDegrees / 360) * (TRACK_WIDTH * M_PI / WHEEL_DIAMETER) * 360;

    setLeftMotors((angleInDegrees > 0) ? speed : -speed);
    setRightMotors((angleInDegrees > 0) ? -speed : speed);
    
    brazo.open();

    while ( (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations) && (fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) ) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveDistanceN(double distanceInInches, double speed) {
    resetEncoders();

    Recolector.spin(reverse, speed, percent);
    Rampa.spin(reverse, speed, percent);

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveDistanceB(double distanceInInches, double speed) {
    resetEncoders();

    brazo.open();
    
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
    RecolectorNeumatica.open();
}

void recoleccion(int speed,double duration) {
  Recolector.spin(reverse, speed, percent);
  Rampa.spin(reverse, speed, percent);
  wait(duration, seconds);
  stopAllMotors();
}