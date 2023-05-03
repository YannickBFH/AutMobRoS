#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    :   sensor("quat1"),
        servo("servo1"),
        timedomain("Main time domain", dt, true)
{
    // Name all blocks
    sensor.setName("IMU-Sensor");
    controller.setName("Controller");
    constant.setName("Constant");
    servo.setName("Servo");

    // Name all signals
    sensor.getOut().getSignal().setName("theta [rad]");
    controller.getOut().getSignal().setName("phi [rad]");
    constant.getOut().getSignal().setName("servo setpoint [rad]");

    // Connect signals
    controller.getIn().connect(sensor.getOut());
    servo.getIn().connect(constant.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(sensor);
    timedomain.addBlock(controller);
    timedomain.addBlock(constant);
    timedomain.addBlock(servo);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}