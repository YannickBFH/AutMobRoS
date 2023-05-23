#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem
    PeripheralInput<> enc1, enc2;           // Encoder-Inputsignal (1 and 2)
    Sum<> e;
    Gain<> Kp;
    D<> ed;
    Gain<> Kd;
    Sum<> qdd_c;
    Gain<> M;
    Saturation<> MQmax;
    Gain<> iInv;
    Gain<> KmInv;
    Gain<> R;

    D<> qd;
    Saturation<> qdmax;
    Gain<> i;
    Gain<> Km;
    Sum<> U;

    PeripheralOutput<> mot1;


    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP