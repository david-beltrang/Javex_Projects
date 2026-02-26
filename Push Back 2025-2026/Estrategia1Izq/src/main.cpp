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

vex::pneumatics BarraSacar(Brain.ThreeWirePort.A);
vex::pneumatics BarraLoader(Brain.ThreeWirePort.B);

bool pistonLoader = false;
bool pistonSacar = true;

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

    
    // PASO 1: Movimiento recto, girar y avanzar hacia el Goal
    moveDistance(24, 85); // Mover hacia adelante 14 pulgadas a velocidad 80%
    rotateOnAxis(-92, 85, pid); // Girar 90 grados a la derecha velocidad 80%
    moveDistance(18, -85); // Mover hacia atras 17 pulgadas a velocidad 80%
    // PASO 2: Poner el bloque y volver al Loader
    recoleccionSubir(100, 1.5); // Activar recolección durante 3 segundos a velocidad 100%
    rotateOnAxis(-0.5, 50, pid); // Girar 90 grados a la izquierda a velocidad 60%
    moveDistance(23, 70); // Mover hacia adelante 17 pulgadas a velocidad
    moveDistance(5, 60); // Mover hacia adelante 3 pulgadas a velocidad 80%
    // PASO 3: Recolectar los 3 bloques del Loader
    moveDistanceConRecoleccionSinTirar(7, 95, 80); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSinTirar(0.5, -95, 80); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSinTirar(1, 95, 80); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSacarAdelante(0.5, -95, 60); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSacarAdelante(1, 95, 60); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    recoleccionSubirSinTirar(30, 3); // Activar recolección durante 3 segundos a velocidad 100%
    // PASO 4: Volver al Goal y poner los bloques
    moveDistance(7, -10); // Mover hacia atras 4 pulgadas a velocidad 30%
    rotateOnAxis(-7.5, 65, pid); // Girar 90 grados a la izquierda a velocidad 80%
    moveDistance(26, -80); // Mover hacia atras 14 pulgadas a velocidad 80%
    moveDistance(2, -60); // Mover hacia adelante 3 pulgadas a velocidad 80%
    recoleccionSubir(100, 4); // Activar recolección durante 2 segundos a velocidad 100%

    /*
    // PASO 1: Movimiento recto, girar y avanzar hacia el Loader
    BarraLoader.open(); // Abrir el pistón para cargar
    moveDistance(9.2, 17.5); // Mover hacia adelante 9.2 pulgadas a velocidad 80%
    moveDistanceConRecoleccionSinTirar(9.2, 27, 90); // Mover hacia adelante 9.2 pulgadas a velocidad 27% con recolección activa
    moveDistanceConRecoleccionSinTirar(3, 95, 90); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSinTirar(0.5, -95, 90); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSinTirar(1, 95, 90); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSacarAdelante(0.5, -95, 60); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    moveDistanceConRecoleccionSacarAdelante(1, 95, 60); // Mover hacia adelante 10 pulgadas a velocidad 80% con recolección activa
    recoleccionSubirSinTirar(30, 3); // Activar recolección durante 3 segundos a velocidad 100%
    recoleccionSubirSinTirar(10, 2); // Activar recolección durante 3 segundos a velocidad 100%
    moveDistance(4, -10); // Mover hacia atras 4 pulgadas a velocidad 30%
    moveDistance(28, -80); // Mover hacia atras 14 pulgadas a velocidad 80%
    rotateOnAxis(-9, 55, pid); // Girar 90 grados a la izquierda a velocidad 80%
    moveDistance(5, -80); // Mover hacia adelante 3 pulgadas a velocidad 80%
    recoleccionSubir(100, 4); // Activar recolección durante 2 segundos a velocidad 100%
*/
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
        if (Controller.ButtonL2.pressing())
        {
            Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
            // Mover recoleccion completa para devolver
        }
        else if (Controller.ButtonL1.pressing())
        {
            Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        // Mover recoleccion para Goal bajito
        else if (Controller.ButtonR1.pressing())
        {
            Recolector1.spin(directionType::fwd, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        // Mover recoleccion para almacenar
        else if (Controller.ButtonR2.pressing())
        {
            Recolector1.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector2.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector3.spin(directionType::rev, 100, velocityUnits::pct);
            Recolector5.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        else
        {
            Recoleccion.stop(brakeType::hold);
        }

        if (Controller.ButtonB.pressing())
        {
            // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
            while (Controller.ButtonB.pressing()){
                // Espera a que el botón se suelte
            }

            // Cambiamos el estado del pistón
            pistonLoader = !pistonLoader;

            // Ejecutamos la acción correspondiente Abrir o Cerrar segun en que este
            if (pistonLoader){
                BarraLoader.open();
            }
            else{
                BarraLoader.close();
            }
        }

        if (Controller.ButtonDown.pressing())
        {
            // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
            while (Controller.ButtonDown.pressing()){
                // Espera a que el botón se suelte
            }

            // Cambiamos el estado del pistón
            pistonSacar = !pistonSacar;

            // Ejecutamos la acción correspondiente Abrir o Cerrar segun en que este
            if (pistonSacar){
                BarraSacar.open();
            }
            else{
                BarraSacar.close();
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
