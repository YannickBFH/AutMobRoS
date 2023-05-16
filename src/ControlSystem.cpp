#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   enc1("enc1"),
        enc2("enc2"),
        fwKinOdom(0.15),
        RvRx(1.0),
        omegaR(0.0),
        invKin(0.15),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    enc1.setName("Encoder 1");
    enc2.setName("Encoder 2");
    E.setName("E");
    Ed.setName("Ed");
    fwKinOdom.setName("forward Kinematics");
    invKin.setName("inverse Kinematics");

    // Name all signals
    enc1.getOut().getSignal().setName("q1 [m]");
    enc1.getOut().getSignal().setName("q2 [m]");
    E.getOut().getSignal().setName("q [m]");
    Ed.getOut().getSignal().setName("qd [m/s]");

    // Connect signals (I modyfied the Controller from exercise 2)
    E.getIn(0).connect(enc1.getOut());
    E.getIn(1).connect(enc2.getOut());
    Ed.getIn().connect(E.getOut());
    fwKinOdom.getIn().connect(Ed.getOut());

    invKin.getInRvRx().connect(RvRx.getOut());
    invKin.getInOmegaR().connect(omegaR.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(enc1);
    timedomain.addBlock(enc2);
    timedomain.addBlock(E);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(RvRx);
    timedomain.addBlock(omegaR);
    timedomain.addBlock(invKin);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}