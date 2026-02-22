/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       PC                                                        */
/*    Created:      2/6/2026, 5:09:59 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "configuration.h"
#include "driver.h"
#include "funciones.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller;
inertial inertialSensor = inertial(PORT4);

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT5, false);
motor LeftMotor2(PORT6, true);
motor LeftMotor3(PORT7, false);
motor LeftMotor4(PORT8, true);
motor_group LeftDrive(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT14, true);
motor RightMotor2(PORT15, false);
motor RightMotor3(PORT11, true);
motor RightMotor4(PORT16, false);
motor_group RightDrive(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

motor Recolector1(PORT3, true);
motor Recolector2(PORT18, true);
motor Recolector3(PORT2, false);
motor Recolector4(PORT13, true);
motor Recolector5(PORT12, false);
motor_group Recoleccion(Recolector1, Recolector2, Recolector3, Recolector4, Recolector5);
vex::pneumatics BarraLoader(Brain.ThreeWirePort.A);

bool pistonAbierto = false;

// Var PID
PID pid;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) {
    vex::task::sleep(100); // Espera hasta que la calibración esté completa
    }
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

void autonomous(void)
{
    // ..........................................................................
    calibrateInertial();
    pid.kp = 0.6;
    pid.kd = 0.2;
    pid.ki = 0.001;

    // PASO 1: Movimiento recto, girar y avanzar hacia el Loader
    moveDistance(32, 80); // Mover hacia adelante 14 pulgadas a velocidad 80%
    moveDistanceConRecoleccion(6, 50, 100); // Subir recolección durante 3 segundos a velocidad 100%
    rotateOnAxisConRecoleccion(65, 80, pid); // Girar 90 grados a la derecha a velocidad 80%
    moveDistanceConRecoleccion(4, 40, 100); // Subir recolección durante 3 segundos a velocidad 100%
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

void usercontrol(void)
{
    // User control code here, inside the loop
    while (1)
    {
        twoJoysticksControl();
        // Control del motor recolector y rampa usando L1 y L2
        // Mover recoleccion completa para recolectar
        if (Controller.ButtonL1.pressing())
        {
            Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
            // Mover recoleccion completa para devolver
        }
        else if (Controller.ButtonL2.pressing())
        {
            Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        // Mover recoleccion para almacenar
        else if (Controller.ButtonR1.pressing())
        {
            Recolector1.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        // Mover recoleccion para Goal bajito
        else if (Controller.ButtonR2.pressing())
        {
            Recolector1.spin(directionType::fwd, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        else
        {
            Recoleccion.stop(brakeType::hold);
        }

        if (Controller.ButtonA.pressing())
        {
            // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
            while (Controller.ButtonA.pressing())
            {
                // Espera a que el botón se suelte
            }

            // Cambiamos el estado del pistón
            pistonAbierto = !pistonAbierto;

            // Ejecutamos la acción correspondiente
            if (pistonAbierto)
            {
                BarraLoader.open();
            }
            else
            {
                BarraLoader.close();
            }
        }

        // Espera para evitar saturar el CPU
        task::sleep(20);
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
