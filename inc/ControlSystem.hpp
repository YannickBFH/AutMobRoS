#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/PeripheralOutput.hpp>


using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem
    PeripheralInput<> encoder2;         // Encoder-Inputsignal
    Gain<> cont;                        // scale the value 
    Saturation<> Qmax;                  // check if value is a limit
    Gain<> iInv;                        // inverse gear-ratio
    Gain<> kMInv;                       // inverse motor-constant
    Gain<> R;                           // Resistance
    PeripheralOutput<> motor1;          // Motor-Outputsignal                  

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP