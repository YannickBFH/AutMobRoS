#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   sensor("quat1"),
        servo("servo1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    sensor.setName("IMU-Sensor");
    controller.setName("Controller");
    servo.setName("Servo");

    // Name all signals
    sensor.getOut().getSignal().setName("theta [rad]");
    controller.getOut().getSignal().setName("phi [rad]");

    // Connect signals
    controller.getIn().connect(sensor.getOut());
    servo.getIn().connect(controller.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(sensor);
    timedomain.addBlock(controller);
    timedomain.addBlock(servo);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}