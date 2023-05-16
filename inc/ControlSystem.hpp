#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include <eeros/control/Constant.hpp>
#include "customBlocks/InvKin.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem
    PeripheralInput<> enc1, enc2;           // Encoder-Inputsignal (1 and 2)
    Mux<2> E;                               // Merge the two signals into a vector
    D<eeros::math::Vector2> Ed;             // Derive the signal
    FwKinOdom<> fwKinOdom;                  // forward Kinematics (Defined as a subsystem)

    Constant<> RvRx, omegaR;                // Constants for testing
    InvKin<> invKin;                        // Inverse Kinematics

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP