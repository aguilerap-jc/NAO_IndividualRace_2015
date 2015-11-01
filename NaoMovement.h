#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>

using namespace std;
using namespace AL;

class NaoMovement {
public:
    NaoMovement(const string ip, const int port, bool local);
    void initialPosition();
    void stop();

private:
    AL::ALRobotPostureProxy posture;  // Posture Proxy
    AL::ALMotionProxy motion;         // Motion Proxy

    bool local;             // Flag for the execution type (local or remote).
    int port;
    string ip;
};
