#ifndef ORIENTATIONEXCEPTION_HPP_
#define ORIENTATIONEXCEPTION_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include "ControlSystem.hpp"
#include <eeros/sequencer/Condition.hpp>


class CheckOrientation : public eeros::sequencer::Condition
{
public:
    // Initiate all Elements
    CheckOrientation(double angle, ControlSystem &cs) : angle(angle), cs(cs) {}
    bool validate() 
    { 
        return abs(cs.sensor.getOut().getSignal().getValue()) > angle; 
    }

private:
    ControlSystem &cs;
    double angle;
};

class OrientationException : public eeros::sequencer::Sequence
{
public:
    // Initiate all Elements
    OrientationException(std::string name, eeros::sequencer::Sequence *caller,
                         ControlSystem &cs, CheckOrientation checkOrientation)
        : cs(cs), checkOrientation(checkOrientation),
          eeros::sequencer::Sequence(name, caller, true) // Sequence in blocking = true
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        log.warn() << "Orientation around x is outside of the allowed range!";
        return 0;
    }

    // Condition needs to be fulfilled to get out of sequence
    bool checkExitCondition()
    {
        return !checkOrientation.validate();
    }

private:
    ControlSystem &cs;
    CheckOrientation checkOrientation;
};


#endif // ORIENTATIONEXCEPTION_HPP_
