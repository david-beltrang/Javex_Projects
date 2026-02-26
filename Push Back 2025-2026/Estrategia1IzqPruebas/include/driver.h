#include "vex.h"
using namespace vex;

// Librerías propias
#include "configuration.h"
#include "funciones.h"

// Modo de control
int controlMode = 0;

// Función para cambiar el modo de control
void switchControlMode() {
    controlMode = (controlMode + 1) % 2;
    Brain.Screen.clearScreen();
    Brain.Screen.print("Modo de control: %d", controlMode);
}

// Función para controlar con dos joysticks
void twoJoysticksControl() {
    int leftSpeed = Controller.Axis3.position() + (Controller.Axis1.position() * 1);
    int rightSpeed = Controller.Axis3.position() - (Controller.Axis1.position() * 1);

    LeftDrive.spin(forward, leftSpeed, percent);
    RightDrive.spin(forward, rightSpeed, percent);
}

//Función para controlar con un joystick
void singleJoystickControl() {
    int forwardSpeed = Controller.Axis3.position();
    int turnSpeed = Controller.Axis4.position();
    LeftDrive.spin(forward, forwardSpeed + turnSpeed, percent);
    RightDrive.spin(forward, forwardSpeed - turnSpeed, percent);
}

// Función para controlar con las flechas
void arrowControl() {
    if (Controller.ButtonUp.pressing()) {
        LeftDrive.spin(forward, 100, percent);
        RightDrive.spin(forward, 100, percent);
    } else if (Controller.ButtonDown.pressing()) {
        LeftDrive.spin(reverse, 100, percent);
        RightDrive.spin(reverse, 100, percent);
    } else if (Controller.ButtonLeft.pressing()) {
        LeftDrive.spin(reverse, 100, percent);
        RightDrive.spin(forward, 100, percent);
    } else if (Controller.ButtonRight.pressing()) {
        LeftDrive.spin(forward, 100, percent);
        RightDrive.spin(reverse, 100, percent);
    } else {
        LeftDrive.stop();
        RightDrive.stop();
    }
}
void joystickNewControl(){
    int leftSpeed = Controller.Axis3.position(); 
    int rightSpeed = Controller.Axis2.position(); 
    LeftDrive.spin(forward, leftSpeed, percent);
    RightDrive.spin(forward, rightSpeed, percent); 
}