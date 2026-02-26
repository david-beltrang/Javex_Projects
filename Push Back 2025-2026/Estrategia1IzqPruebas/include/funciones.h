#include "vex.h"
#include <vector>
#include <functional>
#include <iostream>

#pragma once // Para evitar errores de definición múltiple

#include "configuration.h" // Incluimos la configuración de variables
#include <algorithm>

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

using std::cout;
using std::endl;

//---------------- FUNCIONES DE MOVIMIENTO ----------------

void resetEncoders()
{
    LeftMotor1.setPosition(0, rotationUnits::deg);
    LeftMotor2.setPosition(0, rotationUnits::deg);
    LeftMotor3.setPosition(0, rotationUnits::deg);
    LeftMotor4.setPosition(0, rotationUnits::deg);
    RightMotor1.setPosition(0, rotationUnits::deg);
    RightMotor2.setPosition(0, rotationUnits::deg);
    RightMotor3.setPosition(0, rotationUnits::deg);
    RightMotor4.setPosition(0, rotationUnits::deg);
}

void setLeftMotors(double speed)
{
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::dps);
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::dps);
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::dps);
    LeftMotor4.spin(directionType::fwd, speed, velocityUnits::dps);
}

void setRightMotors(double speed)
{
    RightMotor1.spin(directionType::fwd, speed, velocityUnits::dps);
    RightMotor2.spin(directionType::fwd, speed, velocityUnits::dps);
    RightMotor3.spin(directionType::fwd, speed, velocityUnits::dps);
    RightMotor4.spin(directionType::fwd, speed, velocityUnits::dps);
}

void stopAllMotors()
{
    LeftMotor1.stop(brake);
    LeftMotor2.stop(brake);
    LeftMotor3.stop(brake);
    LeftMotor4.stop(brake);
    RightMotor1.stop(brake);
    RightMotor2.stop(brake);
    RightMotor3.stop(brake);
    RightMotor4.stop(brake);
}

void stopRecoleccion()
{
    Recoleccion.stop(brake);
}

void recoleccionSubir(int speed, double duration)
{
    Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionSubirStart(int speed)
{
    Recoleccion.spin(directionType::rev, speed, velocityUnits::pct);
}

void recoleccionBajar(int speed, double duration)
{
    Recoleccion.spin(directionType::fwd, speed, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionAlmacenar(int speed, double duration)
{
    Recolector1.spin(directionType::rev, 100, velocityUnits::pct);
    Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
    Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionBajito(int speed, double duration)
{
    Recolector1.spin(directionType::fwd, 100, velocityUnits::pct);
    Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
    Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
    Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

double normalizeAngle(double angle)
{
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

void moveDistance(double distanceInInches, double maxSpeed)
{
    resetEncoders();

    double targetDegrees = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360.0;

    double initialHeading = inertialSensor.rotation(degrees);

    double kHeading = 2.0;   // heading correction gain
    double maxAccel = 30;     // slew rate
    double lastSpeed = 0;

    while (true)
    {
        double leftPos = (
            LeftMotor1.position(deg) +
            LeftMotor2.position(deg) +
            LeftMotor3.position(deg) +
            LeftMotor4.position(deg)
        ) / 4.0;

        double rightPos = (
            RightMotor1.position(deg) +
            RightMotor2.position(deg) +
            RightMotor3.position(deg) +
            RightMotor4.position(deg)
        ) / 4.0;

        double avgPos = (leftPos + rightPos) / 2.0;

        double error = targetDegrees - avgPos;
        double remaining = fabs(error);

        if (remaining < 5)
            break;

        // Physics-based deceleration
        double velocityLimit = sqrt(2 * 600 * remaining);  // 200 = tune
        double targetSpeed = std::min(maxSpeed, velocityLimit);

        if (error < 0)
            targetSpeed = -targetSpeed;

        // Slew rate limit
        double delta = targetSpeed - lastSpeed;

        if (delta > maxAccel) delta = maxAccel;
        if (delta < -maxAccel) delta = -maxAccel;

        targetSpeed = lastSpeed + delta;
        lastSpeed = targetSpeed;

        // Heading correction
        double headingError = normalizeAngle(initialHeading - inertialSensor.rotation(degrees));
        double correction = headingError * kHeading;

        double leftSpeed = targetSpeed + correction;
        double rightSpeed = targetSpeed - correction;

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        wait(10, msec);
    }

    stopAllMotors();
}

/*
void moveDistance(double distanceInInches, double maxSpeed)
{
    resetEncoders();

    double targetDegrees = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360.0;
    double targetAbs = fabs(targetDegrees);
    double sign = (targetDegrees >= 0) ? 1.0 : -1.0;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Target deg: %.2f", targetDegrees);

    // Fracción para comenzar a frenar (18% del recorrido)
    double decelFraction = 0.18;
    double decelStart = targetAbs * decelFraction;

    // Mínimo para distancias muy pequeñas
    if (decelStart < 40)
        decelStart = 40;

    while (true)
    {
        double leftPos = (
            LeftMotor1.position(deg) +
            LeftMotor2.position(deg) +
            LeftMotor3.position(deg) +
            LeftMotor4.position(deg)
        ) / 4.0;

        double rightPos = (
            RightMotor1.position(deg) +
            RightMotor2.position(deg) +
            RightMotor3.position(deg) +
            RightMotor4.position(deg)
        ) / 4.0;

        double avgPos = (leftPos + rightPos) / 2.0; // signed
        double error = targetDegrees - avgPos;
        double remaining = fabs(error);

        if (remaining <= 2.0)  // tolerancia pequeña
            break;

        double currentSpeed = maxSpeed;

        // Dsesaceleración proporcional al error restante
        if (remaining < decelStart)
        {
            double scale = remaining / decelStart;
            currentSpeed = maxSpeed * scale;

            if (currentSpeed < 8)  // velocidad mínima baja
                currentSpeed = 8;
        }

        currentSpeed *= sign;

        setLeftMotors(currentSpeed);
        setRightMotors(currentSpeed);

        task::sleep(10);
    }

    stopAllMotors();

    double finalLeft = (
        LeftMotor1.position(deg) +
        LeftMotor2.position(deg) +
        LeftMotor3.position(deg) +
        LeftMotor4.position(deg)
    ) / 4.0;

    double finalRight = (
        RightMotor1.position(deg) +
        RightMotor2.position(deg) +
        RightMotor3.position(deg) +
        RightMotor4.position(deg)
    ) / 4.0;

    double finalAvg = (finalLeft + finalRight) / 2.0;
    double finalError = targetDegrees - finalAvg;

    Brain.Screen.newLine();
    Brain.Screen.print("Final deg: %.2f", finalAvg);

    Brain.Screen.newLine();
    Brain.Screen.print("Error deg: %.2f", finalError);
}
*/

void moveDistanceConRecoleccion(double distanceInInches, double maxSpeed, int recoSpeed)
{
    resetEncoders();

    // Activar recoleccion antes de empezar a moverse
    recoleccionSubirStart(recoSpeed);

    resetEncoders();

    double targetDegrees = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360.0;
    double targetAbs = fabs(targetDegrees);
    double sign = (targetDegrees >= 0) ? 1.0 : -1.0;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Target deg: %.2f", targetDegrees);

    // Fracción para comenzar a frenar (18% del recorrido)
    double decelFraction = 0.18;
    double decelStart = targetAbs * decelFraction;

    // Mínimo para distancias muy pequeñas
    if (decelStart < 40)
        decelStart = 40;

    while (true)
    {
        double leftPos = (
            LeftMotor1.position(deg) +
            LeftMotor2.position(deg) +
            LeftMotor3.position(deg) +
            LeftMotor4.position(deg)
        ) / 4.0;

        double rightPos = (
            RightMotor1.position(deg) +
            RightMotor2.position(deg) +
            RightMotor3.position(deg) +
            RightMotor4.position(deg)
        ) / 4.0;

        double avgPos = (leftPos + rightPos) / 2.0; // signed
        double error = targetDegrees - avgPos;
        double remaining = fabs(error);

        if (remaining <= 2.0)  // tolerancia pequeña
            break;

        double currentSpeed = maxSpeed;

        // Dsesaceleración proporcional al error restante
        if (remaining < decelStart)
        {
            double scale = remaining / decelStart;
            currentSpeed = maxSpeed * scale;

            if (currentSpeed < 8)  // velocidad mínima baja
                currentSpeed = 8;
        }

        currentSpeed *= sign;

        setLeftMotors(currentSpeed);
        setRightMotors(currentSpeed);

        task::sleep(10);
    }

    stopAllMotors();

    // Apagar recolección al terminar
    stopRecoleccion();
}

double computePID(PID &pid, double setpoint, double current, double dt)
{
    if (dt < 0.005) dt = 0.005;  // protect against division spikes

    pid.error = setpoint - current;

    // ----- Integral with anti-windup -----
    pid.integral += pid.error * dt;

    if (pid.integral > pid.integralLimit)
        pid.integral = pid.integralLimit;
    if (pid.integral < -pid.integralLimit)
        pid.integral = -pid.integralLimit;

    // ----- Derivative (filtered) -----
    double rawDerivative = (pid.error - pid.prevError) / dt;

    double alpha = 0.6;  // smoothing factor (0.5–0.8 works well)
    pid.derivative = alpha * pid.derivative + (1 - alpha) * rawDerivative;

    // ----- Output -----
    double output =
        pid.kp * pid.error +
        pid.ki * pid.integral +
        pid.kd * pid.derivative;

    pid.prevError = pid.error;

    return output;
}

void resetPID(PID &pid)
{
    pid.error = 0;
    pid.prevError = 0;
    pid.integral = 0;
    pid.derivative = 0;
}

void calibrateInertial()
{
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()){
        wait(100, msec);
    }
}

void rotateOnAxis(double targetAngle, double maxSpeed, PID &pid)
{
    resetPID(pid);

    double lastTime = Brain.timer(msec);
    double lastOutput = 0;
    int settleCount = 0;

    while (true)
    {
        double currentAngle = inertialSensor.rotation(degrees);

        // Normalize to shortest path
        double error = normalizeAngle(targetAngle - currentAngle);

        double angularVelocity = inertialSensor.gyroRate(dps);

        if (fabs(error) < 1 && fabs(angularVelocity) < 5)
            settleCount++;
        else
            settleCount = 0;

        if (settleCount > 10)
            break;

        double currentTime = Brain.timer(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        if (dt < 0.005)
            dt = 0.005;

        double output = computePID(pid, targetAngle, currentAngle, dt);

        // Clamp output
        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;

        // Slew rate limiting
        double accelLimit = 15;
        double brakeLimit = 30;

        double delta = output - lastOutput;

        if (delta > accelLimit) delta = accelLimit;
        if (delta < -brakeLimit) delta = -brakeLimit;

        output = lastOutput + delta;
        lastOutput = output;

        setLeftMotors(output);
        setRightMotors(-output);

        wait(10, msec);
    }

    stopAllMotors();
}

void rotateOnAxisRecoleccion(double targetAngle, double maxSpeed, PID &pid_t)
{
    resetPID(pid_t);

    // Activar recolección mientras se mueve
    recoleccionSubirStart(80);

    double lastTime = Brain.timer(msec);

    while (true)
    {
        double currentAngle = inertialSensor.rotation(degrees);

        // Error normalizado (camino más corto)
        double error = normalizeAngle(targetAngle - currentAngle);

        if (fabs(error) < 2) break;

        double currentTime = Brain.timer(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        double power = pid_t.kp * error
                     + pid_t.kd * ((error - pid_t.prevError) / dt);

        pid_t.prevError = error;

        if (power > maxSpeed)
            power = maxSpeed;
        if (power < -maxSpeed)
            power = -maxSpeed;

        setLeftMotors(power);
        setRightMotors(-power);

        task::sleep(10);
    }

    stopAllMotors();
    wait(100, msec);
}