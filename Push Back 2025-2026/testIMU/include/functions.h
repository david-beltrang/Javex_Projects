#include "vex.h"
#include <vector>
#include <functional>
#include <iostream>

#pragma once 

#include "configuration.h"
#include <algorithm>
#include <cmath>

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

using std::cout;
using std::endl;

void resetEncoders()
{
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
}

void setLeftMotors(double speed)
{
    LeftDrive.spin(directionType::fwd, speed, velocityUnits::pct); 
}

void setRightMotors(double speed)
{
    RightDrive.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors()
{
    LeftDrive.stop(brakeType::hold);
    RightDrive.stop(brakeType::hold);
}

void stopRecoleccion()
{
    Recoleccion.stop(brakeType::hold);
}

void activarRecoleccion(int speed, double duration)
{
    Recoleccion.spin(reverse, speed, percent);
    wait(duration, seconds);
    stopRecoleccion();
}

void calibrateInertial()
{
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()){
        wait(100, msec);
    }
}

//---------------- FUNCIONES CON PID ----------------
/*
    Gira el robot a un ángulo absoluto usando el IMU y PID.
    Basado en el ángulo deseado en grados (0-360).
*/
 void rotateOnAxis(double target_angle) {
    // Configurar el setpoint del PID y reiniciar su estado actual
    turnPID.set_setpoint(target_angle);
    turnPID.reset_state();

    while (true) {
        // Obtener el ángulo actual del robot desde el sensor inercial
        double current_angle = inertialSensor.rotation(degrees);
        
        // Manejar el error circular (shortest path)
        turnPID.adjust_circular_setpoint(current_angle);
        
        // 1. Calcular la salida (voltaje)
        double output_voltage = turnPID.calculate(current_angle);

        // 2. Limitar la salida
        output_voltage = std::max((double)-MAX_VOLTAGE, std::min(output_voltage, (double)MAX_VOLTAGE));

        // 3. Aplicar la potencia (Giro: Izquierda + / Derecha -) en Voltaje
        LeftDrive.spin(forward, output_voltage, voltageUnits::mV);
        RightDrive.spin(forward, -output_voltage, voltageUnits::mV); 

        // 4. Criterio de Salida: cerca del target Y velocidad baja
        if (turnPID.is_at_target() && std::abs(LeftDrive.velocity(rpm)) < 1.0) {
            break; 
        }

        wait(DELTA_T_MS, msec); 
    }

    stopAllMotors();
}


/*
    Conduce el robot una distancia recta usando Control Dual PID (Distancia + Rumbo).
    Basando en la distancia deseada en pulgadas y teniendo una velocidad para la recolección.
*/
void moveDistance(double target_inches, double speed_recoleccion = 0) {
    // Configurar los setpoints de los PID y reiniciar sus estados actuales
    drivePID.set_setpoint(target_inches);
    drivePID.reset_state();
    correctionPID.reset_state();

    // Asegurar que los encoders están en 0
    resetEncoders();

    // Configurar el setpoint inicial del PID de corrección de rumbo
    double initial_heading = inertialSensor.rotation(degrees);
    // Establecer el setpoint del PID de corrección al rumbo inicial
    correctionPID.set_setpoint(initial_heading);

    // Iniciar la recolección si se especifica velocidad
    if (speed_recoleccion!= 0) {
        Recoleccion.spin(reverse, speed_recoleccion, percent);
    }

    // Circunferencia / 360 grados de rotación del motor.
    const double DEGREES_TO_INCHES = WHEEL_CIRCUMFERENCE / 360.0;
    
    while (true) {
        // A. Variables de Proceso (PV)

        // Obtener la posición promedio de los encoders (en pulgadas)
        double avg_encoder_degrees = (LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2.0;
        double current_distance = avg_encoder_degrees * DEGREES_TO_INCHES; 
        
        // Obtener el rumbo actual del robot desde el sensor inercial
        double current_heading = inertialSensor.rotation(degrees);

        // B. Cálculo de Salidas (Voltaje)
        double distance_output = drivePID.calculate(current_distance);
        double correction_output = correctionPID.calculate(current_heading); 
        
        // C. Combinación de Salidas
        double left_power_raw = distance_output + correction_output;
        double right_power_raw = distance_output - correction_output;

        // D. Manejo de Saturación (Clamping/Escalado)
        double max_raw_power = std::max(std::abs(left_power_raw), std::abs(right_power_raw));
        double scale_factor = 1.0;

        // Si la potencia máxima excede el voltaje máximo, escalar ambas potencias
        if (max_raw_power > MAX_VOLTAGE) {
            scale_factor = MAX_VOLTAGE / max_raw_power;
        }

        // Potencias finales después del escalado para mantener la proporción
        double left_voltage = left_power_raw * scale_factor;
        double right_voltage = right_power_raw * scale_factor;

        // E. Aplicar Potencia (Control por Voltaje)
        LeftDrive.spin(forward, left_voltage, voltageUnits::mV);
        RightDrive.spin(forward, right_voltage, voltageUnits::mV);

        // F. Criterio de Salida
        if (drivePID.is_at_target() && std::abs(LeftDrive.velocity(rpm)) < 1.0) {
            break;
        }

        wait(DELTA_T_MS, msec); 
    }

    stopAllMotors();
    if (speed_recoleccion!= 0) {
        stopRecoleccion();
    }
}