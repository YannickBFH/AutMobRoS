#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   enc1("enc1"),
        enc2("enc2"),
        fwKinOdom(0.15),
        RvRx(1.0),
        omegaR(0.0),
        invKin(0.15),
        g(21.2 / 2.0 / M_PI),
        controller(1 / dt, 0.7, 2.3, 3441.0 / 104.0 / 0.04 * 3441.0 / 104.0 / 0.04 * 6.8e-8, 0.1),
        invMot(0.1 / 0.04, 21.2 * 0.04, 3441.0 / 104.0 / 0.04, 8.44e-3, 8.0),
        M1("motor1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    enc1.setName("Encoder 1");
    enc2.setName("Encoder 2");
    E.setName("E");
    Ed.setName("Ed");
    fwKinOdom.setName("forward Kinematics");
    invKin.setName("inverse Kinematics");
    d.setName("d");
    g.setName("g");
    controller.setName("controller");
    invMot.setName("invMotMod");
    M1.setName("M1");


    // Name all signals
    enc1.getOut().getSignal().setName("q1 [m]");
    enc1.getOut().getSignal().setName("q2 [m]");
    E.getOut().getSignal().setName("q [m]");
    Ed.getOut().getSignal().setName("qd [m/s]");
    d.getOut().getSignal().setName("q1d [m/s]");
    g.getOut().getSignal().setName("q1d_d [rad/s]");
    invMot.getOut().getSignal().setName("U [V]");

    // Connect signals (I modyfied the Controller from exercise 2)
    E.getIn(0).connect(enc1.getOut());
    E.getIn(1).connect(enc2.getOut());
    Ed.getIn().connect(E.getOut());
    fwKinOdom.getIn().connect(Ed.getOut());

    invKin.getInRvRx().connect(RvRx.getOut());
    invKin.getInOmegaR().connect(omegaR.getOut());

    d.getIn().connect(enc1.getOut());
    g.getIn().connect(enc2.getOut());
    controller.getIn(0).connect(g.getOut());
    controller.getIn(1).connect(d.getOut());
    invMot.getIn(0).connect(controller.getOut(0));
    invMot.getIn(1).connect(controller.getOut(1));
    M1.getIn().connect(invMot.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(enc1);
    timedomain.addBlock(enc2);
    timedomain.addBlock(E);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(RvRx);
    timedomain.addBlock(omegaR);
    timedomain.addBlock(invKin);
    timedomain.addBlock(d);
    timedomain.addBlock(g);
    timedomain.addBlock(controller);
    timedomain.addBlock(invMot);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}