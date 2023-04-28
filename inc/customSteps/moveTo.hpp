#ifndef moveTo_HPP_
#define moveTo_HPP_

#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include "ControlSystem.hpp"

class moveTo : public eeros::sequencer::Step
{
public:
    moveTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : eeros::sequencer::Step(name, caller), cs(cs) 
    {
        log.info() << "Step created: " << name;
    }

    int operator()(double x, double y)
    {
        this->target[0] = x;
        this->target[1] = y;
        return start();
    }

    bool checkPreCondition()
    {
        return cs.pathPlanner.getCurrentPosition() != target;
    }


    int action() // Nr. 2
    {
        // do something
        cs.pathPlanner.setTarget(target);
        return 0;
    }

    bool checkExitCondition()
    {
        return cs.pathPlanner.endReached();
    }

private:
    // Define variables, conditions, monitors, exception sequences, ... Nr. 1
    ControlSystem &cs;
    eeros::math::Vector2 target;
};

#endif // moveTo_HPP_
