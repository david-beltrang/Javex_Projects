#include "vex.h"
using namespace vex;

// Librerías propias
#include "configuration.h"
#include "funciones_posicionales.h"

// Modo de control
int controlMode = 0;
bool pistonAbierto = false;
bool piston2Abierto = false; 

// Función para cambiar el modo de control
void switchControlMode() {
    controlMode = (controlMode + 1) % 2;
    Brain.Screen.clearScreen();
    Brain.Screen.print("Modo de control: %d", controlMode);
}

void joystickNewControl(){
  int leftSpeed=Controller1.Axis3.position(); 
  int rightSpeed = Controller1.Axis2.position(); 
  Left.spin(forward, leftSpeed, percent);
  Right.spin(forward, rightSpeed, percent); 
}


// Función para controlar con dos joysticks
void twoJoysticksControl() {
    int leftSpeed = Controller1.Axis3.position()+Controller1.Axis1.position();
    int rightSpeed = Controller1.Axis3.position()- Controller1.Axis1.position();
    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
}