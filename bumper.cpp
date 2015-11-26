/**
 * Copyright (c) 2011 Aldebaran Robotics
 */

#include "bumper.h"
#include <time.h>
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include <althread/alcriticalsection.h>

Bumper::Bumper(boost::shared_ptr<AL::ALBroker> broker, const std::string& name): AL::ALModule(broker, name),fCallbackMutex(AL::ALMutex::createALMutex()),naoVision(true), naoMovement(true)
{
  start = true;
  setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");
  functionName("onRightBumperPressed", getName(), "Method called when the right bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Bumper::onRightBumperPressed);

  functionName("onLeftBumperPressed", getName(), "Method called when the right bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Bumper::onLeftBumperPressed);
}

Bumper::~Bumper() {
  fMemoryProxy.unsubscribeToEvent("onRightBumperPressed", "Bumper");
  fMemoryProxy2.unsubscribeToEvent("onLeftBumperPressed","Bumper");
}

void Bumper::init() {
  try {
    /** Create a proxy to ALMemory.
    */
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy2 = AL::ALMemoryProxy(getParentBroker());

    fState = fMemoryProxy.getData("RightBumperPressed");
    fState2 = fMemoryProxy2.getData("LeftBumperPressed");
    /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Bumper","onRightBumperPressed");
    fMemoryProxy2.subscribeToEvent("LeftBumperPressed", "Bumper","onLeftBumperPressed");
  }
  catch (const AL::ALError& e) {
    qiLogError("module.example") << e.what() << std::endl;
  }
}

void Bumper::onRightBumperPressed() {
  qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;
  /**
  * As long as this is defined, the code is thread-safe.
  */
  AL::ALCriticalSection section(fCallbackMutex);

  /**
  * Check that the bumper is pressed.
  */
  fState =  fMemoryProxy.getData("RightBumperPressed");
  if (fState  > 0.5f) {
    return;
  }
  try {
    fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
    //fTtsProxy.say("Right bumper pressed");
    if (start) 
      walkWithBumper();
  }
  catch (const AL::ALError& e) {
    qiLogError("module.example") << e.what() << std::endl;
  }
}


void Bumper::onLeftBumperPressed() {
  qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;
  /**
  * As long as this is defined, the code is thread-safe.
  */
  AL::ALCriticalSection section(fCallbackMutex);

  /**
  * Check that the bumper is pressed.
  */
  fState2 =  fMemoryProxy2.getData("LeftBumperPressed");
  if (fState2  > 0.5f) {
    return;
  }
  try {
    fTtsProxy2 = AL::ALTextToSpeechProxy(getParentBroker());
    //fTtsProxy2.say("Left bumper pressed");
    if (!start)
      emergencyStop();
  }
  catch (const AL::ALError& e) {
    qiLogError("module.example") << e.what() << std::endl;
  }
}

void Bumper::walkWithBumper() {
    start = false;
    bool LOCAL = true;         // Flap for the kind of execution (local or remote).
    bool NAO = true;
    bool finish = false;
    char key = 'x';
    double angleToBlackLine;    // Angle of the detected line.

    Mat bottomImage;
    VideoCapture cap(1);        // Class for video capturing from video files or cameras.

    naoMovement.initialPositionIndividualRace();

    while (!finish) {
        if (NAO) {
            bottomImage = naoVision.getImageFrom(NaoVision::BOTTOM_CAMERA);
        } else {
            cap >> bottomImage;
            naoVision.setSourceMat(bottomImage);
        }

        if (naoVision.naoIsNearTheGoal(bottomImage)) {
            naoMovement.naoOnGoal();
            finish = true;
        } else {
            angleToBlackLine = naoVision.calculateAngleToBlackLine();
            naoMovement.moveInIndividualRace(angleToBlackLine);
        }

        //key = waitKey(10);

        for (int i = 0; i < 250000; i++);   // Delay.
    }

    naoVision.unsubscribe();
    naoMovement.stop();
    start = true;
}

void Bumper::emergencyStop() {
  naoVision.unsubscribe();
  naoMovement.stop();
  start = true;
}