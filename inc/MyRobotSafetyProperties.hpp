#ifndef MyRobotSAFETYPROPERTIES_HPP_
#define MyRobotSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class MyRobotSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    MyRobotSafetyProperties(ControlSystem &cs, double dt);

    // Define all possible events ()
    eeros::safety::SafetyEvent doSystemOn;
    eeros::safety::SafetyEvent abort;
    eeros::safety::SafetyEvent systemStarted;
    eeros::safety::SafetyEvent emergency;
    eeros::safety::SafetyEvent resetEmergency;
    eeros::safety::SafetyEvent powerOn;
    eeros::safety::SafetyEvent startMoving;
    eeros::safety::SafetyEvent shutdown;
    eeros::safety::SafetyEvent motorsHalted;
    eeros::safety::SafetyEvent powerOff;
    eeros::safety::SafetyEvent stopMoving;

    // Define all possible levels
    eeros::safety::SafetyLevel slSystemOff;         // Level 0
    eeros::safety::SafetyLevel slShuttingDown;      // Level 1
    eeros::safety::SafetyLevel slBraking;           // Level 2
    eeros::safety::SafetyLevel slStartingUp;        // Level 3
    eeros::safety::SafetyLevel slEmergency;         // Level 4
    eeros::safety::SafetyLevel slEmergencyBraking;  // Level 5
    eeros::safety::SafetyLevel slSystemOn;          // Level 6
    eeros::safety::SafetyLevel slMotorPowerOn;      // Level 7
    eeros::safety::SafetyLevel slSystemMoving;      // Level 8

private:
    // Define all critical outputs (We can only use them if they are defined here)
    eeros::hal::Output<bool>* redLED;
    eeros::hal::Output<bool>* greenLED;

    // Define all critical inputs (We can only use them if they are defined here)
    eeros::hal::Input<bool>* buttonMode;
    eeros::hal::Input<bool>* buttonPause;

    ControlSystem &cs;
};

#endif // MyRobotSAFETYPROPERTIES_HPP_
