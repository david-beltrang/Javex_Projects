/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      3/7/2025, 6:31:15 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */


#include "vex.h"
#include "configuration.h"
#include "driver.h"
#include "funciones_posicionales.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */


void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */


void autonomous(void) {
  // ..........................................................................
  //PASO 1
  // 1. Recolecta el primer ring y lo mantiene con el de recarga
  moveParabolicV(81, 80, 67, 69);
  // 2. Vuelve a la estaca del color y pone ambos rings
  moveParabolic(19, -95.5, -98);
  // 3. Pone los dos rings en la estaca de color
  recoleccion(100, 2);

  //PASO 2
  // 4. Se acomoda para ir por la estaca
  moveDistance(10, 50);
  rotateOnAxis(37, 87);
  //5. Recoge la estaca 
  moveDistance(39, -87);
  Pinza.open();
  //6. Se acomoda para ir por el ring de arriba
  rotateOnAxis(71, 95);

  //PASO 3
  //7. Va por los otros dos rings
  RecolectorNeumatica.open();
  moveDistance(38, 95);
  // 8. Recolecta la dona azul de arriba 
  RecolectorNeumatica.close();
  recoleccion(75, 3);

  //PASO 4
  //9. Va por la dona de abajo 
  rotateOnAxis(30, -98);
  moveParabolicV(43, 90, 78, 95);
  recoleccion(75, 3);
  moveDistance(4, -80);

  //PASO 5
  //10. Se acomoda para ir por los rings de la esquina
  rotateOnAxis(16, 85);

  //11. Va y tumba las donas de la esquina
  brazo.open();
  moveDistance(23, 90);
  rotateOnAxis(65, 48);


  //12. Deja la estaca en la esquina positiva 
  moveDistance(6, -85);
  Pinza.close();
  rotateOnAxis(7, 85);
  moveParabolicV(20, 85, 90, 95);


  //PASO 6
  //13. Se dirige a la escalera
  brazo.close();
  moveDistance(43, 60);

  // ..........................................................................
}


/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */


void usercontrol(void) {
  // User control code here, inside the loop
    // User control code here, inside the loop
    while (true) {
      // Cambiar el modo de control con el botón A
      if (Controller1.ButtonY.pressing()) {
          switchControlMode();
          while (Controller1.ButtonA.pressing()) {
              // Esperar a que se suelte el botón A
              task::sleep(10);
          }
      }
  
      // Control del robot basado en el modo seleccionado
      if (controlMode == 0) {
          twoJoysticksControl();
      }else if(controlMode == 1){
          joystickNewControl(); 
      }
  
      // Control del motor recolector y rampa usando L1 y L2
      if (Controller1.ButtonL1.pressing()) {
          Recolector.spin(directionType::fwd, 100, velocityUnits::pct);
          Rampa.spin(directionType::fwd, 100, velocityUnits::pct);
      } else if (Controller1.ButtonL2.pressing()) {
          Recolector.spin(directionType::rev, 100, velocityUnits::pct);
          Rampa.spin(directionType::rev, 100, velocityUnits::pct);
      } else {
          Recolector.stop(brakeType::hold);
          Rampa.stop(brakeType::hold);
      }
  
      // Control del motor recolector y rampa usando L1 y L2
      if (Controller1.ButtonX.pressing()) {
          Garra.spin(directionType::fwd, 100, velocityUnits::pct);
      } else if (Controller1.ButtonA.pressing()) {
          Garra.spin(directionType::rev, 100, velocityUnits::pct);
      } else {
          Garra.stop(brakeType::hold);
      }
  
      if(Controller1.ButtonR2.pressing())
      {
          // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
          while(Controller1.ButtonR2.pressing()) {
              // Espera a que el botón se suelte
          }
  
          // Cambiamos el estado del pistón
          pistonAbierto = !pistonAbierto;
          
          // Ejecutamos la acción correspondiente
          if(pistonAbierto) {
              Pinza.open();
          } else {
              Pinza.close();
          }
      }
  
      if(Controller1.ButtonR1.pressing())
      {
          RecolectorNeumatica.open();
      }else{RecolectorNeumatica.close();}
  
      if(Controller1.ButtonB.pressing())
      {
          // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
          while(Controller1.ButtonB.pressing()) {
              // Espera a que el botón se suelte
          }
  
          // Cambiamos el estado del pistón
          pistonAbierto = !pistonAbierto;
          
          // Ejecutamos la acción correspondiente
          if(pistonAbierto) {
              brazo.open();
          } else {
              brazo.close();
          }
      }
      }   
  
  
      // Espera para evitar saturar el CPU
      wait(20, msec);
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