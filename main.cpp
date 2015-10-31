/* Copyright
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

/* Coautores
   Marco Ramirez
   Juan Carlos Aguilera Perez
   Aurelio Puebla
   Fernando Lopez
*/

/* Aspectos de revision
 * Confirmar se mantega dentro del carril
 * Revisar Caidas
 * Agregar que se detenga a los 3 mins
 * Crear rutina de apagado y encendido
*/

/* Reglas
 * Distancia total 5m
 * Franja inicio 18 cm
 * Franja final 30 cm y de color rojo
 * Max tiempo fuera de carril = 3 segs
 * Se considera fuera de carril con 1 pie fuera de la linea
 * Si algo sucede el robot tiene que poder responder autonomamente (levantar)
*/

#include <iostream>
#include "NaoVision.h"

using namespace std;
using namespace AL;
using namespace cv;

AL::ALValue walk();

int main(int argc, char *argv[]) {
    const int port = 9559;
    string ip = argv[1];        // NAO ip
    cout << "IP: " << ip << endl;

    AL::ALRobotPostureProxy posture(ip, port);  // Posture Proxy
    AL::ALMotionProxy motion(ip, port);         // Motion Proxy
    Mat src;

    bool DEBUG = true;          // Bandera para mostrar mensajes
    bool LOCAL = false;         // Bandera para el tipo de ejecucion (local o remota)
    bool NAO = true;
    char key = 'x';
    double orientation;         // Angulo de la línea detectada
    string clientName = "test";

    posture.goToPosture("Crouch", 0.5);
    posture.goToPosture("StandInit", 0.5);
    cout << "Stand" << endl;

    NaoVision naoVision(ip, port, clientName, LOCAL);

    VideoCapture cap(1);        // Class for video capturing from video files or cameras.

    while(key != 27) {
        if (NAO) {
            naoVision.getImage();
            src = naoVision.getSourceMat();
        }
        else {
            cap >> src;
        }

        orientation = naoVision.calculateAngleToALine();

        if(DEBUG){
            //cout << "VelLin: " << linearVelocity(orientation) << endl;
            //cout << "VelAng: " << angularVelocity(orientation) << endl;
            cout << "Theta: " << orientation << endl;
            cout << "--------------------------------" << endl;
        }

        key = waitKey(10);

        //motion.move(linearVelocity(orientation), 0, angularVelocity(orientation),walk());
        for(int i = 0; i < 250000; i++);
    }

    naoVision.unsubscribe();
    motion.stopMove();
    cout << "Stop" << endl;
    posture.goToPosture("Crouch", 0.5);
    motion.setStiffnesses("Body", 0);

    return 0;
}

AL::ALValue walk() {
   return  AL::ALValue::array(AL::ALValue::array("MaxStepX",0.08),AL::ALValue::array("MaxStepY",0.14),
                              AL::ALValue::array("MaxStepTheta",0.4),AL::ALValue::array("MaxStepFrequency",0.5), //Frec 0.5
                              AL::ALValue::array("StepHeight",0.04),AL::ALValue::array("TorsoWx",0.0),
                              AL::ALValue::array("TorsoWy",0));
}
