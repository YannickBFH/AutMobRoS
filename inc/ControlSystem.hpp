#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks for ControllSystem
    PeripheralInput<> enc1, enc2;           // Encoder-Inputsignal (1 and 2)

    Mux<2> mux1;
    Mux<2> mux2;

    Sum<2, eeros::math::Vector2> e;
    Gain<eeros::math::Vector2> Kp;
    D<eeros::math::Vector2> ed;
    Gain<eeros::math::Vector2> Kd;
    Sum<2, eeros::math::Vector2> qdd_c;
    Gain<eeros::math::Vector2> M;
    Saturation<eeros::math::Vector2> MQmax;
    Gain<eeros::math::Vector2> iInv;
    Gain<eeros::math::Vector2> KmInv;
    Gain<eeros::math::Vector2> R;

    D<eeros::math::Vector2> qd;
    Saturation<eeros::math::Vector2> qdmax;
    Gain<eeros::math::Vector2> i;
    Gain<eeros::math::Vector2> Km;
    Sum<2, eeros::math::Vector2> U;

    DeMux<2> deMux;

    PeripheralOutput<> mot1, mot2;


    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP