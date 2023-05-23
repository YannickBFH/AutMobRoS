#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   enc1("enc1"),
        enc2("enc2"),
        Kp(((1.0 / dt) / 3.2) * ((1.0 / dt) / 3.2)),
        Kd(2.0 * 0.7 * ((1.0 / dt) / 3.2)),
        M(((3441.0 / 104.0) * (3441.0 / 104.0)) * 6.8e-8),
        MQmax(0.1),
        iInv(104.0/3441.0),
        KmInv(1.0/8.44e-3),
        R(8.0),
        qdmax(21.2),
        i(3441.0/104.0),
        Km(8.44e-3),
        mot1("motor1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    enc1.setName("Encoder 1");
    enc2.setName("Encoder 2");
    e.setName("e");
    ed.setName("ed");
    Kp.setName("Kp");
    Kd.setName("Kd");
    qdd_c.setName("qdd_c");
    M.setName("M");
    MQmax.setName("MQmax");
    iInv.setName("iInv");
    KmInv.setName("KmInv");
    R.setName("R");
    qd.setName("qd");
    qdmax.setName("qdmax");
    i.setName("i");
    Km.setName("Km");
    U.setName("U");
    mot1.setName("Motor 1");
    

    // Name all signals
    enc1.getOut().getSignal().setName("q1 [rad]");
    enc2.getOut().getSignal().setName("q2 [rad]");
    e.getOut().getSignal().setName("e [rad]");
    ed.getOut().getSignal().setName("ed [rad/s]");
    Kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
    Kd.getOut().getSignal().setName("qdd_cd [rad/s^2]");
    qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
    M.getOut().getSignal().setName("Q [Nm]");
    MQmax.getOut().getSignal().setName("Q1 [Nm]");
    iInv.getOut().getSignal().setName("T1 [Nm]");
    KmInv.getOut().getSignal().setName("I1 [A]");
    R.getOut().getSignal().setName("U [V]");
    qd.getOut().getSignal().setName("qd1 [rad/s]");
    qdmax.getOut().getSignal().setName("qd1 [rad/s]");
    i.getOut().getSignal().setName("om1 [rad/s]");
    Km.getOut().getSignal().setName("Uom [V]");
    U.getOut().getSignal().setName("U1 [V]");

    // Connect signals
    e.getIn(0).connect(enc2.getOut());
    e.getIn(1).connect(enc1.getOut());
    e.negateInput(1);
    ed.getIn().connect(e.getOut());
    Kp.getIn().connect(e.getOut());
    Kd.getIn().connect(ed.getOut());
    qdd_c.getIn(0).connect(Kp.getOut());
    qdd_c.getIn(1).connect(Kd.getOut());
    M.getIn().connect(qdd_c.getOut());
    MQmax.getIn().connect(M.getOut());
    iInv.getIn().connect(MQmax.getOut());
    KmInv.getIn().connect(iInv.getOut());
    R.getIn().connect(KmInv.getOut());

    qd.getIn().connect(enc1.getOut());
    qdmax.getIn().connect(qd.getOut());
    i.getIn().connect(qdmax.getOut());
    Km.getIn().connect(i.getOut());

    U.getIn(0).connect(R.getOut());
    U.getIn(1).connect(Km.getOut());


    mot1.getIn().connect(U.getOut());
    

    // Add blocks to timedomain
    timedomain.addBlock(enc1);
    timedomain.addBlock(enc2);
    timedomain.addBlock(e);
    timedomain.addBlock(Kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(Kd);
    timedomain.addBlock(qdd_c);
    timedomain.addBlock(M);
    timedomain.addBlock(MQmax);
    timedomain.addBlock(iInv);
    timedomain.addBlock(KmInv);
    timedomain.addBlock(R);
    timedomain.addBlock(qd);
    timedomain.addBlock(qdmax);
    timedomain.addBlock(i);
    timedomain.addBlock(Km);
    timedomain.addBlock(U);
    timedomain.addBlock(mot1);
    

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}