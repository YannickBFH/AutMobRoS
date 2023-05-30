#include "ControlSystem.hpp"


ControlSystem::ControlSystem(double dt)
    :   enc1("enc1"),
        enc2("enc2"),
        fwKinOdom(0.15, 0.17),
        tcpVecPosCont(10.0, 0.7, 0.1),
        invKin(0.17, 0.15),
        controller(1 / dt, 0.7, 2.3, 3441.0 / 104.0 / 0.04 * 3441.0 / 104.0 / 0.04 * 6.8e-8, 0.1),
        invMot(0.1 / 0.04, 21.2 * 0.04, 3441.0 / 104.0 / 0.04, 8.44e-3, 8.0),
        M1("motor1"),
        M2("motor2"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    enc1.setName("Encoder 1");
    enc2.setName("Encoder 2");
    mux.setName("Mux");
    Ed.setName("Ed");
    fwKinOdom.setName("forward Kinematics");
    invKin.setName("inverse Kinematics");
    controller.setName("controller");
    invMot.setName("invMotMod");
    demux.setName("DeMux");
    M1.setName("M1");
    M2.setName("M2");


    // Name all signals
    enc1.getOut().getSignal().setName("q1 [m]");
    enc1.getOut().getSignal().setName("q2 [m]");
    mux.getOut().getSignal().setName("q [m]");
    Ed.getOut().getSignal().setName("V_w [m/s]");

    fwKinOdom.getOutGvT().getSignal().setName("V_T [m/s]");
    fwKinOdom.getOutGrT().getSignal().setName("r_T [m]");
    fwKinOdom.getOutPhi().getSignal().setName("phi [rad]");
    fwKinOdom.getOutOmegaT().getSignal().setName("omega_T [rad/s]");

    invKin.getOut().getSignal().setName("V_w_d [m/s]");

    invMot.getOut().getSignal().setName("U [V]");

    demux.getOut(0).getSignal().setName("U1 [V]");
    demux.getOut(1).getSignal().setName("U2 [V]");

    // Connect signals (I modyfied the Controller from exercise 2)
    mux.getIn(0).connect(enc1.getOut());
    mux.getIn(1).connect(enc2.getOut());
    Ed.getIn().connect(mux.getOut());

    fwKinOdom.getIn().connect(Ed.getOut());

    tcpVecPosCont.getIn().connect(fwKinOdom.getOutGrT());

    invKin.getInGvTc().connect(tcpVecPosCont.getOut());
    invKin.getInPhi().connect(fwKinOdom.getOutPhi());

    controller.getIn(0).connect(invKin.getOut());
    controller.getIn(1).connect(Ed.getOut());

    invMot.getIn(0).connect(controller.getOut(0));
    invMot.getIn(1).connect(controller.getOut(1));

    demux.getIn().connect(invMot.getOut());

    M1.getIn().connect(demux.getOut(0));
    M2.getIn().connect(demux.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(enc1);
    timedomain.addBlock(enc2);
    timedomain.addBlock(mux);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
     timedomain.addBlock(tcpVecPosCont);
    timedomain.addBlock(invKin);
    timedomain.addBlock(controller);
    timedomain.addBlock(invMot);
    timedomain.addBlock(demux);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}