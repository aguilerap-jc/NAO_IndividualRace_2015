#include <iostream>
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

// Establish the position in Crouch and set Stiffnesses to body.
void NaoMovement::stop() {
    motion.stopMove();
    posture.goToPosture("Crouch", 0.5);
    motion.setStiffnesses("Body", 0);

    if (!local)
        cout << "Stop" << endl;
}
