#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt) // Here I build the scheme I drawed in preparation
    // Initializing
    : encoder("enc1"),
      motor("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    encoder.setName("Encoder");
    controller.setName("Controller");
    motor.setName("Motor");

    // Name all signals
    encoder.getOut().getSignal().setName("phy [rad]");
    controller.getOut().getSignal().setName("U [V]");

    // Connect signals
    controller.getIn().connect(encoder.getOut());
    controller.getIn().connect(controller.getOut());

    // Add blocks to timedomain (in the same way, as they are executed)
    timedomain.addBlock(encoder);
    timedomain.addBlock(controller);
    timedomain.addBlock(motor);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}