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

#include "customBlocks/PI_Controller.hpp"
#include "customBlocks/invMotMod.hpp"
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem

    // Forward Kinematic
    PeripheralInput<> enc1, enc2;           // Encoder-Inputsignal (1 and 2)
    Mux<2> E;                               // Merge the two signals into a vector
    D<eeros::math::Vector2> Ed;             // Derive the signal
    FwKinOdom<> fwKinOdom;                  // forward Kinematics (Defined as a subsystem)

    // Inverse Kinematic
    Constant<> RvRx, omegaR;                // Constants for testing
    InvKin<> invKin;                        // Inverse Kinematics

    // Controller
    D<> d;
    Gain<> g;
    PI_Controller<> controller; 
    invMotMod<> invMot;
    PeripheralOutput<> M1;


    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP