#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include "customBlocks/Controller.hpp"
#include <eeros/control/PeripheralOutput.hpp>


using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> sensor;   // IMU-Senso
    Controller<> controller;    // Controler
    PeripheralOutput<> servo;   // Servo                  

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP