#include <iostream>
#include <cmath>
#include "NaoMovement.h"

NaoMovement::NaoMovement(const string ip, const int port, bool local): posture(ip, port), motion(ip, port) {
    this->ip = ip;
    this->port = port;
    this->local = local;
}

// Establish the position in Crouch and then in StandInit.
void NaoMovement::initialPosition() {
    posture.goToPosture("Crouch", 0.5);
    posture.goToPosture("StandInit", 0.5);

    if (!local)
        cout << "Stand" << endl;
}

// Given an angle in degrees, move the NAO in straight mode.
void NaoMovement::moveInIndividualRace(double angleInDegrees) {
    //motion.move(linearVelocity(orientation), 0, angularVelocity(orientation),walkParameters());

    if (!local){
        cout << "VelLin: " << linearVelocity(angleInDegrees) << endl;
        cout << "VelAng: " << angularVelocity(angleInDegrees) << endl;
        cout << "Theta: " << angleInDegrees << endl;
        cout << "--------------------------------" << endl;
    }
}

// Establish the position in Crouch and set Stiffnesses to body.
void NaoMovement::stop() {
    motion.stopMove();
    posture.goToPosture("Crouch", 0.5);
    motion.setStiffnesses("Body", 0);

    if (!local)
        cout << "Stop" << endl;
}

// Helper methods.

// v = vmax * e^(-k*abs(theta - 90))
double NaoMovement::linearVelocity(double theta){
    const double vMax = 0.85;
    const double k1 = 1.0 / 40;
    const double k2 = 1.0 / 15;
    return vMax * exp(-(theta > 90 ? k2 : k1) * abs(theta - 90));
}

// w = wmax * ( 1 - e^(-k*abs(theta - 90)))*N if (theta > 90) (N = -1) else (N = 1)
double NaoMovement::angularVelocity(double theta){
    const double wMax = 0.25;
    const double k1 = 1.0 / 20;     // k1 right to left correction  // 1/50
    const double k2 = 1.0 / 20;     // k2 left to right correction
    return pow(-1, theta > 90) * (wMax * (1 - exp(-(theta > 90 ? k2 : k1) * abs(theta - 90))));
}

AL::ALValue NaoMovement::walkParameters() {
   return  AL::ALValue::array(AL::ALValue::array("MaxStepX",0.08),AL::ALValue::array("MaxStepY",0.14),
                              AL::ALValue::array("MaxStepTheta",0.4),AL::ALValue::array("MaxStepFrequency",0.5), //Frec 0.5
                              AL::ALValue::array("StepHeight",0.04),AL::ALValue::array("TorsoWx",0.0),
                              AL::ALValue::array("TorsoWy",0));
}
