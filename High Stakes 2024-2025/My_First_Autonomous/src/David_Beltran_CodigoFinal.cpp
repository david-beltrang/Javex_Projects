/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      18/10/2024, 2:23:48 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
vex::controller Control;

    //MOTORES PARA MOVER LA BASE
    motor MotorIzquierdo1(PORT14, gearSetting::ratio18_1, true);
    motor MotorIzquierdo2(PORT18, gearSetting::ratio18_1, false);
    motor MotorIzquierdo3(PORT19, gearSetting::ratio18_1, false);
    motor MotorIzquierdo4(PORT20, gearSetting::ratio18_1, true);

    motor MotorDerecho1(PORT7, gearSetting::ratio18_1, false);
    motor MotorDerecho2(PORT8, gearSetting::ratio18_1, true);
    motor MotorDerecho3(PORT9, gearSetting::ratio18_1, true);
    motor MotorDerecho4(PORT10, gearSetting::ratio18_1, false);

    motor_group BaseIzquierda(MotorIzquierdo1, MotorIzquierdo2, MotorIzquierdo3, MotorIzquierdo4);
    motor_group BaseDerecha(MotorDerecho1, MotorDerecho2, MotorDerecho3, MotorDerecho4);


    //MOTORES PARA LA RAMPA Y RECOLECTOR
    motor Rampa1(PORT4,gearSetting::ratio18_1, true);
    motor Rampa2(PORT5,gearSetting::ratio18_1, false);
    motor Recolector(PORT6,gearSetting::ratio18_1, true);
    motor_group RampaCompleta(Rampa1, Rampa2, Recolector);

    //MOTOR PARA LA GARRA
    motor Garra1(PORT11,gearSetting::ratio18_1, true);
    motor Garra2(PORT12,gearSetting::ratio18_1, true);
    motor_group GarraCompleta(Garra1, Garra2);


    //FUNCIONES MOVIMIENTO
    //Funcion para mover el robot hacia Adelante en el Autonomo
    void moverAdelante(int tiempo){
        BaseIzquierda.spin(forward,90,rpm);
        BaseDerecha.spin(forward,90,rpm);
        wait(tiempo, seconds);
        BaseIzquierda.stop();
        BaseDerecha.stop();
    }

    //Funcion para mover el robot hacia Atras en el Autonomo
    void moverAtras(int tiempo){
        BaseIzquierda.spin(reverse,90,rpm);
        BaseDerecha.spin(reverse,90,rpm);
        wait(tiempo, seconds);
        BaseIzquierda.stop();
        BaseDerecha.stop();
    }
    
    //Funcion para hacer un giro hacia la Izquierda en el Autonomo
    void giroIzquierda(int tiempo){
        BaseIzquierda.spin(reverse,90,rpm);
        BaseDerecha.spin(forward,90,rpm);
        wait(tiempo, seconds);
        BaseIzquierda.stop();
        BaseDerecha.stop();
    }
    
    //Funcion para hacer un giro hacia la Derecha en el Autonomo
    void giroDerecha(int tiempo){
        BaseIzquierda.spin(forward,90,rpm);
        BaseDerecha.spin(reverse,90,rpm);
        wait(tiempo, seconds);
        BaseIzquierda.stop();
        BaseDerecha.stop();
    }


int main() {
    
    while (true) {
        //DRIVER
        //Modo normalito con los 2 Joysticks
        BaseIzquierda.spin(forward, Control.Axis3.position(), percent);
        BaseDerecha.spin(forward, Control.Axis2.position(), percent);

        //Modo para moverse unicamente con el Joystick Izquierdo
        BaseIzquierda.spin(forward, Control.Axis3.position() + Control.Axis4.position(), percent);
        BaseDerecha.spin(forward, Control.Axis3.position() - Control.Axis4.position(), percent);

        //Al presionar L1 la Rampa sube
        if(Control.ButtonL1.pressing()){
            RampaCompleta.spin(forward, 60, velocityUnits::pct);
        }
        //Al presionar L2 la Rampa baja
        else if (Control.ButtonL2.pressing()){
            RampaCompleta.spin(reverse, 60, velocityUnits::pct);
        }
        else{
            RampaCompleta.stop();
        }
        
        //Al presionar R1 la Garra sube
        if(Control.ButtonR1.pressing()){
            GarraCompleta.spin(forward, 60, velocityUnits::pct);
        }
        //Al presionar R2 la Garra baja
        else if (Control.ButtonR2.pressing()){
            GarraCompleta.spin(reverse, 60, velocityUnits::pct);
        }
        else{
            GarraCompleta.stop();
        }

       
        //AUTONOMO
        //El robot inicia hacia atras para agarrar la estaca y no perder tiempo dando el giro
        moverAtras(3);
        //agarrarEstaca(); No entendi como usar la Neumatica :(
        giroIzquierda(0.8);
        RampaCompleta.spin(forward, 60, velocityUnits::pct);
        moverAdelante(1);
        giroDerecha(1);
        moverAdelante(3);
        giroDerecha(0.8);
        moverAdelante(1);
        //soltarEstaca();
        giroIzquierda(1.5);
        moverAtras(1);
        //agarrarEstaca();
        giroDerecha(0.6);
        moverAdelante(3);
        GarraCompleta.spin(forward, 60, velocityUnits::pct);
        GarraCompleta.stop();
        moverAdelante(0.2);
        GarraCompleta.spin(reverse, 60, velocityUnits::pct);
        GarraCompleta.stop();
        giroIzquierda(0.6);
        moverAdelante(1);
        giroIzquierda(1);
        //soltarEstaca();
        moverAdelante(4);
        GarraCompleta.spin(forward, 60, velocityUnits::pct);
        GarraCompleta.stop();
        moverAdelante(0.2);
        GarraCompleta.spin(reverse, 60, velocityUnits::pct);
        GarraCompleta.stop();
        RampaCompleta.stop();
        
    }
    while(1) {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}