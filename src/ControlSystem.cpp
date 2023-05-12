#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   encoder2("enc2"),
        cont(21.2 / 2.0 * M_PI),
        qdmax(21.2),
        i(3441.0 / 104.0),
        kM(8.44e-3),
        motor1("motor1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    encoder2.setName("Encoder 2");
    cont.setName("scaling");
    qdmax.setName("limitvalue");
    i.setName("gear-ratio");
    kM.setName("motor-constant");
    motor1.setName("Motor 1");

    // Name all signals
    encoder2.getOut().getSignal().setName("q2[rad]");
    cont.getOut().getSignal().setName("qd1[rad/s]");
    qdmax.getOut().getSignal().setName("qd1[rad/s]");
    i.getOut().getSignal().setName("om1[rad/s]");
    kM.getOut().getSignal().setName("U[V]");

    // Connect signals (I modyfied the Controller from exercise 2)
    cont.getIn().connect(encoder2.getOut());
    qdmax.getIn().connect(cont.getOut());
    i.getIn().connect(qdmax.getOut());
    kM.getIn().connect(i.getOut());
    motor1.getIn().connect(kM.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(encoder2);
    timedomain.addBlock(cont);
    timedomain.addBlock(qdmax);
    timedomain.addBlock(i);
    timedomain.addBlock(kM);
    timedomain.addBlock(motor1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}