#pragma once
#include "vex.h"
#include <vector>
#include <functional>
#include "PIDController.h"

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

// CONSTANTES FÍSICAS (Mantener las del usuario)
const double WHEEL_DIAMETER = 4.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

// CONSTANTES DE CONTROL
const int MAX_VOLTAGE = 12000; // Voltaje máximo de salida
const double DELTA_T_MS = 20; // Frecuencia del bucle: 20 ms

// Declaración de dispositivos
extern brain Brain;
extern controller Controller;
extern competition Competition;

extern inertial inertialSensor;

extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;
extern motor LeftMotor4;
extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;
extern motor RightMotor4;
extern motor Recoleccion1;
extern motor Recoleccion2;
extern motor Recoleccion3;
extern motor Recoleccion4;
extern motor Recoleccion5;

extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor_group Recoleccion;

// INSTANCIAS DE CONTROLADORES PID
extern PIDController turnPID;      // Controlador para giros
extern PIDController drivePID;     // Controlador para distancia (PID primario)
extern PIDController correctionPID; // Controlador para corrección de rumbo (PD secundario)