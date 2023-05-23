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
        mot2("motor2"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    enc1.setName("Encoder 1");
    enc2.setName("Encoder 2");
    mux1.setName("Mux 1");
    mux2.setName("Mux 2");
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
    deMux.setName("Demux");
    mot1.setName("Motor 1");
    mot2.setName("Motor 2");
    

    // Name all signals
    enc1.getOut().getSignal().setName("q1 [rad]");
    enc2.getOut().getSignal().setName("q2 [rad]");
    mux1.getOut().getSignal().setName("q_mux1 [rad]");
    mux2.getOut().getSignal().setName("q_mux2 [rad]");
    e.getOut().getSignal().setName("e [rad]");
    ed.getOut().getSignal().setName("ed [rad/s]");
    Kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
    Kd.getOut().getSignal().setName("qdd_cd [rad/s^2]");
    qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
    M.getOut().getSignal().setName("Q [Nm]");
    MQmax.getOut().getSignal().setName("Q [Nm]");
    iInv.getOut().getSignal().setName("T [Nm]");
    KmInv.getOut().getSignal().setName("I [A]");
    R.getOut().getSignal().setName("U [V]");
    qd.getOut().getSignal().setName("qd [rad/s]");
    qdmax.getOut().getSignal().setName("qd [rad/s]");
    i.getOut().getSignal().setName("om [rad/s]");
    Km.getOut().getSignal().setName("Uom [V]");
    U.getOut().getSignal().setName("U [V]");
    deMux.getOut(0).getSignal().setName("U [V]");
    deMux.getOut(1).getSignal().setName("U [V]");

    // Connect signals
    mux1.getIn(0).connect(enc2.getOut());
    mux1.getIn(1).connect(enc1.getOut());
    mux2.getIn(0).connect(enc1.getOut());
    mux2.getIn(1).connect(enc2.getOut());
    e.getIn(0).connect(mux1.getOut());
    e.getIn(1).connect(mux2.getOut());
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

    qd.getIn().connect(mux2.getOut());
    qdmax.getIn().connect(qd.getOut());
    i.getIn().connect(qdmax.getOut());
    Km.getIn().connect(i.getOut());

    U.getIn(0).connect(R.getOut());
    U.getIn(1).connect(Km.getOut());

    deMux.getIn().connect(U.getOut());

    mot1.getIn().connect(deMux.getOut(0));
    mot2.getIn().connect(deMux.getOut(1));
    

    // Add blocks to timedomain
    timedomain.addBlock(enc1);
    timedomain.addBlock(enc2);
    timedomain.addBlock(mux1);
    timedomain.addBlock(mux2);
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
    timedomain.addBlock(deMux);
    timedomain.addBlock(mot2);
    timedomain.addBlock(mot1);
    

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}