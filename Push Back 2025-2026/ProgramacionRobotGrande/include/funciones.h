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
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void setRightMotors(double speed)
{
    RightMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors()
{
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    LeftMotor4.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();
    RightMotor4.stop();
}

void stopRecoleccion()
{
    Recoleccion.stop();
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

void moveDistanceConRecoleccion(double distanceInInches, double maxSpeed, int recoSpeed)
{
    resetEncoders();

    // Activar recolección mientras se mueve
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

double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

void resetPID(PID &pid)
{
    pid.error = 0;
    pid.prevError = 0;
    pid.Integral = 0;
}

void calibrateInertial()
{
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating())
    {
        wait(100, msec);
    }
}

double normalizeAngle(double angle)
{
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void rotateOnAxis(double targetAngle, double maxSpeed, PID &pid_t)
{
    resetPID(pid_t);

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

        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        setLeftMotors(power);
        setRightMotors(-power);

        task::sleep(10);
    }

    stopAllMotors();
    wait(100, msec);
}
void rotateOnAxisConRecoleccion(double targetAngle, double maxSpeed, PID &pid_t)
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

        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        setLeftMotors(power);
        setRightMotors(-power);

        task::sleep(10);
    }

    stopAllMotors();

    // Detenemos recolección al terminar
    stopRecoleccion();

    wait(100, msec);
}