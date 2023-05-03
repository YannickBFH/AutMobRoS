#ifndef moveServoTo_HPP_
#define moveServoTo_HPP_

#include <eeros/sequencer/Step.hpp>
#include "ControlSystem.hpp"

class MoveServoTo : public eeros::sequencer::Step
{
public:
    // Initiate all Elements
    MoveServoTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : cs(cs), eeros::sequencer::Step(name, caller)
    {
        log.info() << "Step created: " << name;
    }

    int operator()(double pos)
    {
        this->pos = pos;
        return start();
    }

    int action()
    {
        // Define action
        log.info() << "Moving to " << pos << "rad."; 
        cs.constant.setValue(pos);
        return 0;
    }

private:
    // Define variables, conditions, monitors, exception sequences, ...
    ControlSystem &cs;
    double pos;

};

#endif // moveServoTo_HPP_
