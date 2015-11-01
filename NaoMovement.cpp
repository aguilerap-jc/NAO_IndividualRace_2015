#include <iostream>
#include "NaoMovement.h"

NaoMovement::NaoMovement(const string ip, const int port): posture(ip, port), motion(ip, port) {
    this->ip = ip;
    this->port = port;
}

// Establish the position in Crouch and then in StandInit.
void NaoMovement::initialPosition() {
    posture.goToPosture("Crouch", 0.5);
    posture.goToPosture("StandInit", 0.5);
    cout << "Stand" << endl;
}
