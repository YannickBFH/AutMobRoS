#ifndef MyRobotSAFETYPROPERTIES_HPP_
#define MyRobotSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class MyRobotSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    MyRobotSafetyProperties(ControlSystem &cs, double dt);

    // Here we define the

    // Define all possible events ()
    //eeros::safety::SafetyEvent doSystemOff; 

    // Defina all possible levels
    //eeros::safety::SafetyLevel slSystemOff;         // Level 0

private:
    // Define all critical outputs (We can only use them if they are defined here)
    // eeros::hal::Output<bool>* ...;

    // Define all critical inputs (We can only use them if they are defined here)
    // eeros::hal::Input<bool>* ...;

    ControlSystem &cs;
};

#endif // MyRobotSAFETYPROPERTIES_HPP_
