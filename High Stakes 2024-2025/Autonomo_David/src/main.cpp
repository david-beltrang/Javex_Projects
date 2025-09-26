/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      09/01/2025, 2:23:48 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>

using namespace vex;

// Definiciones y constantes
#define M_PI 3.14159265358979323846 // Valor de pi
const double WHEEL_DIAMETER = 4.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

brain Brain;
controller Controller1;
inertial Inercial(PORT11); 

// Motores del lado izquierdo (puertos 1-4)
motor MotorL1(PORT14, true); 
motor MotorL2(PORT18, false);
motor MotorL3(PORT19, false);
motor MotorL4(PORT20, true);

// Motores del lado derecho (puertos 7-10)
motor MotorR1(PORT7, false);
motor MotorR2(PORT8, true);
motor MotorR3(PORT9, true);
motor MotorR4(PORT10, false);

// Motor para el sistema de recolección
motor Recolector(PORT1, true);
motor Rampa(PORT12, true);
motor Garra(PORT15, true);

vex::pneumatics Pinza(Brain.ThreeWirePort.A);
vex::pneumatics RecolectorNeumatica(Brain.ThreeWirePort.B);
vex::pneumatics Brazo(Brain.ThreeWirePort.C);

// Funciones auxiliares
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
    Garra.stop();
}

// Función para mover el robot en línea recta con corrección de desvíos
void moveWithInertialSensor(double distanceInInches, double speed) {
    // Calibración del sensor de inercia
    Inercial.calibrate();
    while (Inercial.isCalibrating()) {
        wait(100, msec);
    }

    // Ajustar la distancia de acuerdo con el error relativo
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    // Calcular el número de rotaciones de rueda necesarias para alcanzar la distancia deseada
    double targetDistance = distanceInInches; // Aquí simplemente usamos la distancia en pulgadas como objetivo

    // Iniciar los motores con la velocidad indicada
    MotorL1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL4.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR4.spin(directionType::fwd, speed, velocityUnits::pct);

    // Variables para el seguimiento de la distancia recorrida y la orientación
    double distanceTravelled = 0;
    double initialYaw = Inercial.rotation(degrees);

    // Continuar moviendo hasta que la distancia recorrida sea suficiente
    while (distanceTravelled < targetDistance) {
        // Leer los valores del sensor de inercia
        double currentYaw = Inercial.rotation(degrees);
        double currentPitch = Inercial.pitch();
        double currentRoll = Inercial.roll();

        // Calcular el error angular en el eje Z (yaw)
        double yawError = currentYaw - initialYaw;

        // Aplicar un control proporcional para corregir el desvío
        double correction = -yawError * 0.5; // Factor de corrección proporcional

        // Ajustar la velocidad de los motores para corregir el rumbo
        MotorL1.setVelocity(speed + correction, velocityUnits::pct);
        MotorL2.setVelocity(speed + correction, velocityUnits::pct);
        MotorL3.setVelocity(speed + correction, velocityUnits::pct);
        MotorL4.setVelocity(speed + correction, velocityUnits::pct);
        MotorR1.setVelocity(speed - correction, velocityUnits::pct);
        MotorR2.setVelocity(speed - correction, velocityUnits::pct);
        MotorR3.setVelocity(speed - correction, velocityUnits::pct);
        MotorR4.setVelocity(speed - correction, velocityUnits::pct);

        // Leer la aceleración en el eje Y (lateral)
        double lateralAcceleration = Inercial.acceleration(vex::axisType::yaxis);

        // Si la aceleración lateral es negativa, significa que el robot se está desviando hacia la derecha
        if (lateralAcceleration < 0) {
            // Reducir la velocidad de los motores derechos para corregir el desvío
            MotorR1.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR2.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR3.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR4.setVelocity(speed - correction - 10, velocityUnits::pct);
        } else {
            // Aumentar la velocidad de los motores derechos para corregir el desvío
            MotorR1.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR2.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR3.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR4.setVelocity(speed - correction + 10, velocityUnits::pct);
        }

        // Leer la aceleración en el eje X (longitudinal)
        double longitudinalAcceleration = Inercial.acceleration(vex::axisType::xaxis);

        // Si la aceleración longitudinal es negativa, significa que el robot se está desviando hacia atrás
        if (longitudinalAcceleration < 0) {
            // Reducir la velocidad de los motores para corregir el desvío
            MotorL1.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorL2.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorL3.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorL4.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR1.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR2.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR3.setVelocity(speed - correction - 10, velocityUnits::pct);
            MotorR4.setVelocity(speed - correction - 10, velocityUnits::pct);
        } else {
            // Aumentar la velocidad de los motores para corregir el desvío
            MotorL1.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorL2.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorL3.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorL4.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR1.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR2.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR3.setVelocity(speed - correction + 10, velocityUnits::pct);
            MotorR4.setVelocity(speed - correction + 10, velocityUnits::pct);
        }

        // Calcular la distancia recorrida utilizando la aceleración longitudinal
        distanceTravelled += longitudinalAcceleration * 0.1; // Asumiendo que la lectura es en metros por segundo cuadrado y el ciclo es de 100 ms

        // Esperar un corto tiempo para permitir que los motores avancen
        task::sleep(10);
    }

    // Detener todos los motores al finalizar el movimiento
    stopAllMotors();
}


void recoleccion(int speed,double duration) {
  Recolector.spin(reverse, speed, percent);
  Rampa.spin(reverse, speed, percent);
  wait(duration, seconds);
  stopAllMotors();
}
void garrita(int speed,double duration) {
  Garra.spin(reverse, speed, percent);
  wait(duration, seconds);
  stopAllMotors();
}

int main() {
    // Calibración del sensor de inercia
    Inercial.calibrate();
    while (Inercial.isCalibrating()) {
        wait(100, msec);
    }

    // Mover hacia adelante 12 pulgadas a 70% de velocidad
    moveWithInertialSensor(12, 70);

    RecolectorNeumatica.open();
    moveWithInertialSensor(12, 100, 100, 0);
    moveWithInertialSensor(35, 20, 100, 0);
    moveWithInertialSensor(35, 100, 40, 100);
    moveWithInertialSensor(18, 100, 100, 100);

    rotateOnAxis(20, -100);

    moveWithInertialSensor(33, -45, -70, 0);
    moveWithInertialSensor(35, -90, -50, 0);

    rotateOnAxis(7, 100);

    Pinza.open();

    moveWithInertialSensor(60, -60, -60, 0);

    Pinza.close();

    recoleccion(100,2);

    rotateOnAxis(5, -100);

    moveWithInertialSensor(25, 50, 50, 50);

    recoleccion(100,1);

    rotateOnAxis(1.2, -100);

    moveWithInertialSensor(6.75, -50, -50, 0);

    garrita(-20,0.5);
    garrita(-100,0.5);

    rotateOnAxis(10, 100);

    moveWithInertialSensor(45, 100, 100, 100);

    rotateOnAxis(4, -100);

    moveWithInertialSensor(20, 100, 100, 100);

    rotateOnAxis(48, -100);

    moveWithInertialSensor(40, 100, 100, 100);*/

    return 0;
}