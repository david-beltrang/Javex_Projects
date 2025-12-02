/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       abelt                                                     */
/*    Created:      12/1/2025, 10:23:14 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "configuration.h"
#include "functions.h"
#include "driver.h"


using namespace vex;

// A global instance of competition
brain Brain;
controller Controller;
competition Competition;

// Instanciar motores y sensores inerciales
inertial inertialSensor = inertial(PORT21);

// Motores de la base lado Izquierdo
motor LeftMotor4 = motor(PORT5, gearSetting::ratio18_1, true);
motor LeftMotor1 = motor(PORT8, gearSetting::ratio18_1, false);
motor LeftMotor2 = motor(PORT9, gearSetting::ratio18_1, true);
motor LeftMotor3 = motor(PORT10, gearSetting::ratio18_1, false);
// Motores de la base lado Derecho
motor RightMotor1 = motor(PORT1, gearSetting::ratio18_1, true);
motor RightMotor2 = motor(PORT2, gearSetting::ratio18_1, false);
motor RightMotor3 = motor(PORT3, gearSetting::ratio18_1, true);
motor RightMotor4 = motor(PORT4, gearSetting::ratio18_1, false);

// Motores de Recolección
motor Recoleccion1(PORT11, false);
motor Recoleccion2(PORT15, true);
motor Recoleccion3(PORT18, true);
motor Recoleccion4(PORT20, false);
motor Recoleccion5(PORT19, false);

// Grupos de Motores
motor_group LeftDrive = motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
motor_group RightDrive = motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);
motor_group Recoleccion = motor_group(Recoleccion1, Recoleccion2, Recoleccion3, Recoleccion4, Recoleccion5);

// 2. INSTANCIACIÓN DE CONTROLADORES PID
// PID Rotacional (Giro) - Tiende a ser más P y D (Kp=0.06, Ki=0.0, Kd=0.1)
PIDController turnPID(0.06, 0.0, 0.1, 2.0); 

// PID Translacional (Distancia) - Necesita I para el error de estado estacionario (Kp=0.2, Ki=0.0005, Kd=0.05)
PIDController drivePID(0.2, 0.0005, 0.05, 0.5); 

// PD Corrección de Rumbo - Generalmente solo P y D para una corrección rápida (Kp=0.5, Ki=0.0, Kd=0.2)
PIDController correctionPID(0.5, 0.0, 0.2, 0.0);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    // Calibración del IMU (Mantenemos la lógica de espera del usuario)
    Brain.Screen.printAt(10, 50, "IMU Calibrando...");
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) {
        vex::task::sleep(100); 
    }
    Brain.Screen.printAt(10, 50, "IMU Calibrado!     ");
    
    // Configuración inicial de posición
    inertialSensor.setRotation(0.0, degrees);
    resetEncoders();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Reiniciar ganancias a los valores deseados al inicio del autónomo
  turnPID.reset_state();
  drivePID.reset_state();
  correctionPID.reset_state();

  // PASO 1: Movimiento recto, girar y avanzar hacia el Goal del medio
    moveDistance(65.0, 0); 
    rotateOnAxis(-45.0); 
    moveDistance(14.0, 0);
    activarRecoleccion(90, 3.0);

    //PASO 2: Dirigirse al Loader y recolectar bloques
    moveDistance(-25.0, 0);
    rotateOnAxis(170.0);
    moveDistance(20.0, 0);
    rotateOnAxis(45.0);
    moveDistance(5.0, 0);
    activarRecoleccion(-80, 2.0);

    // //PASO 3: Rotar y dirigirse al Goal
    moveDistance(-8.0);
    rotateOnAxis(180.0);
    moveDistance(8.5);
    activarRecoleccion(-80, 5.0);

    // //PASO 4: Rotar y dirigirse al Loader
    moveDistance(-7.0);
    rotateOnAxis(-180.0);
    moveDistance(8.0);
    activarRecoleccion(-80, 3.0);

    // //PASO 5: Rotar y dirigirse al Goal
    moveDistance(-8.0);
    rotateOnAxis(180.0);
    moveDistance(8.5);
    activarRecoleccion(-80, 5.0);

    // //PASO 6: Rotar e ir a la Parking Zone
    moveDistance(-5.0);
    rotateOnAxis(-100.0);
    moveDistance(40.0);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // El control de usuario se mantiene sin cambios, usando su lógica original
    // en driver.h (twoJoysticksControl / joystickNewControl) que usan % de velocidad.
    while (true) {
        //... (Lógica de control de usuario original)
        if (Controller.ButtonY.pressing()) {
             switchControlMode();
             while (Controller.ButtonY.pressing()) {
                 task::sleep(10);
             }
         }
     
         if (controlMode == 0) {
             twoJoysticksControl();
         }else if(controlMode == 1){
             joystickNewControl(); 
         }
     
         // Control de la recoleccion L1(Sube) y L2(Baja)
         if (Controller.ButtonL1.pressing()) {
             Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
         } else if (Controller.ButtonL2.pressing()) {
             Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
         } else {
             Recoleccion.stop(brakeType::hold);
         }
         
         wait(20, msec);
    }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
