#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   encoder2("enc2"),
        cont(0.03 / 2.0 * M_PI),
        Qmax(0.03),
        iInv(104.0 / 3441.0),
        kMInv(1.0 / 8.44e-3),
        R(8.0),
        motor1("motor1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    encoder2.setName("Encoder 2");
    cont.setName("scaling");
    Qmax.setName("limitvalue");
    iInv.setName("inverse gear-ratio");
    kMInv.setName("inverse motor-constant");
    R.setName("resistance");
    motor1.setName("Motor 1");

    // Name all signals
    encoder2.getOut().getSignal().setName("q2[rad]");
    cont.getOut().getSignal().setName("Q1[NM]");
    Qmax.getOut().getSignal().setName("Q1[NM]");
    iInv.getOut().getSignal().setName("T1[NM]");
    kMInv.getOut().getSignal().setName("I1[A]");
    R.getOut().getSignal().setName("U1[V]");

    // Connect signals (I modyfied the Controller from exercise 2)
    cont.getIn().connect(encoder2.getOut());
    Qmax.getIn().connect(cont.getOut());
    iInv.getIn().connect(Qmax.getOut());
    kMInv.getIn().connect(iInv.getOut());
    R.getIn().connect(kMInv.getOut());
    motor1.getIn().connect(R.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(encoder2);
    timedomain.addBlock(cont);
    timedomain.addBlock(Qmax);
    timedomain.addBlock(iInv);
    timedomain.addBlock(kMInv);
    timedomain.addBlock(R);
    timedomain.addBlock(motor1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}