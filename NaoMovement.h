#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>

using namespace std;
using namespace AL;

class NaoMovement {
public:
    NaoMovement(const string ip, const int port);
    void initialPosition();

private:
    AL::ALRobotPostureProxy posture;  // Posture Proxy
    AL::ALMotionProxy motion;         // Motion Proxy

    int port;
    string ip;
};
