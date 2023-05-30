#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include "customBlocks/TCPVecPosCont.hpp"
#include "customBlocks/InvKin.hpp"
#include "customBlocks/PI_Controller.hpp"
#include "customBlocks/invMotMod.hpp"
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem
    PeripheralInput<> enc1, enc2;                       // Encoder-Inputsignal (1 and 2)
    Mux<2> mux;                                         // Merge the two signals into a vector
    D<eeros::math::Vector2> Ed;                         // Derive the signal

    FwKinOdom<> fwKinOdom;                              // forward Kinematics (Defined as a subsystem)

    TCPVecPosCont<> tcpVecPosCont;

    InvKin invKin;                                    // Inverse Kinematics

    PI_Controller<eeros::math::Vector2> controller;     // Controller

    invMotMod<eeros::math::Vector2> invMot;             // InvMotMod


    DeMux<2> demux;                                     // Separation of Vector-Signal 
    PeripheralOutput<> M1, M2;                          // Motor-Outputsignal (1 and 2)


    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP