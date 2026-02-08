/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       PC                                                        */
/*    Created:      2/6/2026, 5:09:59 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller;

// define your global instances of motors and other devices here
//Motor para el sistema de recolección
/*motor Recolector(PORT6, true);
motor Rampa(PORT3, false);
motor Recolector1(PORT14, false);
motor Recolector2(PORT15, false);
motor Escalada1(PORT17, true);
motor Escalada2(PORT19, false);
motor Escalada3(PORT16, true);
motor Escalada4(PORT13, false);*/

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT5, false); 
motor LeftMotor2(PORT6, true);
motor LeftMotor3(PORT7, false);
motor LeftMotor4(PORT8, true);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT14, true);
motor RightMotor2(PORT15, false);
motor RightMotor3(PORT11, true);
motor RightMotor4(PORT16, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

motor Recolector1(PORT3, true);
motor Recolector2(PORT18, true);
motor Recolector3(PORT19, false);
motor Recolector4(PORT13, true);
motor Recolector5(PORT12, false);
motor_group Recoleccion(Recolector1, Recolector2, Recolector3, Recolector4, Recolector5);

/* Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT6, true); 
motor LeftMotor2(PORT8, false);
motor LeftMotor3(PORT9, true);
motor LeftMotor4(PORT10, false);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT4, false);
motor RightMotor2(PORT3, true);
motor RightMotor3(PORT2, false);
motor RightMotor4(PORT1, true);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

motor Recolector1(PORT20, true);
motor Recolector2(PORT19, true);*/


/*vex::pneumatics Pinza(Brain.ThreeWirePort.A);
vex::pneumatics RecolectorNeumatica(Brain.ThreeWirePort.B);
vex::pneumatics Brazo(Brain.ThreeWirePort.C);*/

/*vex::pneumatics Pinza(Brain.ThreeWirePort.A);
vex::pneumatics RecolectorNeumatica(Brain.ThreeWirePort.B);
vex::pneumatics Brazo(Brain.ThreeWirePort.C);*/


bool pistonAbierto = false;


// Función para controlar con dos joysticks
void twoJoysticksControl() {
    int leftSpeed = Controller.Axis3.position() + (Controller.Axis1.position() * 1);
    int rightSpeed = Controller.Axis3.position() - (Controller.Axis1.position() * 1);

    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
}

//Función para controlar con un joystick
void singleJoystickControl() {
    int forwardSpeed = Controller.Axis3.position();
    int turnSpeed = Controller.Axis4.position();
    Left.spin(forward, forwardSpeed + turnSpeed, percent);
    Right.spin(forward, forwardSpeed - turnSpeed, percent);
}

// Función para controlar con las flechas
void arrowControl() {
    if (Controller.ButtonUp.pressing()) {
        Left.spin(forward, 100, percent);
        Right.spin(forward, 100, percent);
    } else if (Controller.ButtonDown.pressing()) {
        Left.spin(reverse, 100, percent);
        Right.spin(reverse, 100, percent);
    } else if (Controller.ButtonLeft.pressing()) {
        Left.spin(reverse, 100, percent);
        Right.spin(forward, 100, percent);
    } else if (Controller.ButtonRight.pressing()) {
        Left.spin(forward, 100, percent);
        Right.spin(reverse, 100, percent);
    } else {
        Left.stop();
        Right.stop();
    }
}
void joystickNewControl(){
  int leftSpeed = Controller.Axis3.position(); 
  int rightSpeed = Controller.Axis2.position(); 
  Left.spin(forward, leftSpeed, percent);
  Right.spin(forward, rightSpeed, percent); 
}

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
  // User control code here, inside the loop
  while (1) {
    twoJoysticksControl();
        // Control del motor recolector y rampa usando L1 y L2
        // Mover recoleccion completa para recolectar
        if (Controller.ButtonL1.pressing()) {
            Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
        // Mover recoleccion completa para devolver
        } else if (Controller.ButtonL2.pressing()) {
            Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
        } 
          // Mover recoleccion para almacenar
          else if (Controller.ButtonR1.pressing()) {
            Recolector1.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        }
          // Mover recoleccion para Goal bajito
          else if (Controller.ButtonR2.pressing()) {
            Recolector1.spin(directionType::fwd, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        } else {
            Recoleccion.stop(brakeType::hold);
        }

        // Espera para evitar saturar el CPU
        task::sleep(20);
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
