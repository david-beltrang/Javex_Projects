#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "vex.h"
#include <cmath> // Para usar std::abs()
#include <algorithm> // Para usar std::max

using namespace vex;

// Definición de la clase genérica PIDController
class PIDController {
private:
    // Ganancias PID
    double kP, kI, kD;
    
    // Variables de Estado
    double setpoint;
    double integral_sum = 0.0;
    double prev_error = 0.0;

    // Límites y Umbrales
    const double INTEGRAL_THRESHOLD = 5.0; // Anti-Windup: solo acumular I si el error es pequeño [1]
    const double MAX_INTEGRAL = 50.0;      
    
    // Tasa de Bucle: 50Hz (20ms), crucial para el cálculo Derivativo [2]
    const double DELTA_T = 0.020; 

public:
    // Umbral de parada, se puede ajustar por tipo de PID (giro vs distancia)
    double DEAD_BAND = 1.0; 

    // Constructor de la clase
    PIDController(double p, double i, double d, double deadband) : kP(p), kI(i), kD(d), DEAD_BAND(deadband) {}

    // Establecer el punto de destino (target)
    void set_setpoint(double target) {
        setpoint = target;
    }

    // Reiniciar variables de estado para una nueva rutina
    void reset_state() {
        integral_sum = 0.0;
        prev_error = 0.0;
    }

    // La función principal que calcula la salida de control (Voltaje)
    double calculate(double current_value) {
        double error = setpoint - current_value;
        double p_out = kP * error;

        // Implementación Anti-Windup
        if (std::abs(error) < INTEGRAL_THRESHOLD && error!= 0.0) {
            integral_sum += error * DELTA_T;
        } else {
            integral_sum = 0.0; 
        }
        
        // Limitar la suma integral
        integral_sum = std::max(-MAX_INTEGRAL, std::min(integral_sum, MAX_INTEGRAL));

        double i_out = kI * integral_sum;

        // Cálculo Derivativo
        double derivative = (error - prev_error) / DELTA_T;
        double d_out = kD * derivative;

        double output = p_out + i_out + d_out;

        prev_error = error;

        return output;
    }

    // Verifica si el error es menor que la banda muerta
    bool is_at_target() const {
        return std::abs(setpoint - prev_error) < DEAD_BAND;
    }
    
    // Función de ayuda para la lógica de giros (shortest path)
    void adjust_circular_setpoint(double current_value) {
        double error = setpoint - current_value;
        // Ajustar el error para que siempre sea el camino más corto [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        // Re-establecer el setpoint para que la clase PID use el camino corto
        setpoint = current_value + error; 
    }
};

#endif // PID_CONTROLLER_H