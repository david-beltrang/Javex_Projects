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

void setLeftMotorsVolt(double volt)
{
    LeftMotor1.spin(fwd, volt, voltageUnits::volt);
    LeftMotor2.spin(fwd, volt, voltageUnits::volt);
    LeftMotor3.spin(fwd, volt, voltageUnits::volt);
    LeftMotor4.spin(fwd, volt, voltageUnits::volt);
}

void setRightMotorsVolt(double volt)
{
    RightMotor1.spin(fwd, volt, voltageUnits::volt);
    RightMotor2.spin(fwd, volt, voltageUnits::volt);
    RightMotor3.spin(fwd, volt, voltageUnits::volt);
    RightMotor4.spin(fwd, volt, voltageUnits::volt);
}

void stopAllMotors()
{
    LeftMotor1.stop(brakeType::brake);
    LeftMotor2.stop(brakeType::brake);
    LeftMotor3.stop(brakeType::brake);
    LeftMotor4.stop(brakeType::brake);
    RightMotor1.stop(brakeType::brake);
    RightMotor2.stop(brakeType::brake);
    RightMotor3.stop(brakeType::brake);
    RightMotor4.stop(brakeType::brake);
}

void stopRecoleccion()
{
    Recoleccion.stop(brakeType::brake);
}

void recoleccionSubir(int speed, double duration)
{
    Recoleccion.spin(directionType::rev, speed, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionSubirSinTirar(int speed, double duration)
{
    Recolector1.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector2.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionSubirStart(int speed)
{
    Recoleccion.spin(directionType::rev, speed, velocityUnits::pct);
}

void recoleccionSubirSinTirarStart(int speed)
{
    Recolector1.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector2.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
}

void recoleccionSubirSacarAdelanteStart(int speed)
{
    Recolector1.spin(directionType::fwd, speed, velocityUnits::pct);
    Recolector2.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
}

void recoleccionBajar(int speed, double duration)
{
    Recoleccion.spin(directionType::fwd, speed, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionAlmacenar(int speed, double duration)
{
    Recolector1.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector2.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector5.spin(directionType::fwd, speed, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void recoleccionBajito(int speed, double duration)
{
    Recolector1.spin(directionType::fwd, speed, velocityUnits::pct);
    Recolector2.spin(directionType::rev, speed, velocityUnits::pct);
    Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
    Recolector5.spin(directionType::fwd, speed, velocityUnits::pct);
    wait(duration, seconds);
    stopRecoleccion();
}

void moveDistance(double distanceInInches, double maxVolt)
{
    resetEncoders();
    inertialSensor.resetRotation();

    double targetDeg = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360.0;
    double startAngle = inertialSensor.rotation();

    double kP_dist = 0.015;   // PID distancia
    double kP_angle = 0.04;   // corrección de rumbo

    while (true)
    {
        double leftPos = LeftMotor1.position(deg);
        double rightPos = RightMotor1.position(deg);
        double avgPos = (leftPos + rightPos) / 2.0;

        double distError = targetDeg - avgPos;

        if (fabs(distError) < 5) break;

        // --- Control de distancia ---
        double forwardVolt = kP_dist * distError;

        // Limitar voltaje máximo
        if (forwardVolt > maxVolt) forwardVolt = maxVolt;
        if (forwardVolt < -maxVolt) forwardVolt = -maxVolt;

        // --- Corrección de rumbo ---
        double currentAngle = inertialSensor.rotation();
        double angleError = currentAngle - startAngle;
        double turnCorrection = kP_angle * angleError;

        double leftVolt = forwardVolt - turnCorrection;
        double rightVolt = forwardVolt + turnCorrection;

        // Clamp final
        leftVolt = std::clamp(leftVolt, -12.0, 12.0);
        rightVolt = std::clamp(rightVolt, -12.0, 12.0);

        setLeftMotorsVolt(leftVolt);
        setRightMotorsVolt(rightVolt);

        task::sleep(10);
    }

    stopAllMotors();
}

/*
void moveDistance(double distanceInInches, double speed)
{
    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    //double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;
    
    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations && fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
}
*/

void moveDistanceConRecoleccion(double distanceInInches, double speed, int recoSpeed)
{
    resetEncoders();

    // Activar recoleccion antes de empezar a moverse
    recoleccionSubirStart(recoSpeed);

    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    //double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;
    
    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations && fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
}

void moveDistanceConRecoleccionSinTirar(double distanceInInches, double speed, int recoSpeed)
{
    resetEncoders();

    // Activar recoleccion antes de empezar a moverse
    recoleccionSubirSinTirarStart(recoSpeed);

    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;
    
    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations && fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
}


void moveDistanceConRecoleccionSacarAdelante(double distanceInInches, double speed, int recoSpeed)
{
    resetEncoders();

    // Activar recoleccion antes de empezar a moverse
    recoleccionSubirSacarAdelanteStart(recoSpeed);

    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;
    
    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations && fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
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
    while (inertialSensor.isCalibrating()){
        wait(100, msec);
    }
}

double normalizeAngle(double angle)
{
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
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