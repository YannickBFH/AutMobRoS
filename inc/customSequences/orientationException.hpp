#ifndef orientationException_HPP_
#define orientationException_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include "ControlSystem.hpp"


class OrientationException : public eeros::sequencer::Sequence
{
public:
    OrientationException(std::string name, eeros::sequencer::Sequence *caller,
                         ControlSystem &cs, CheckOrientation checkOrientation)
        : cs(cs), checkOrientation(checkOrientation),
          eeros::sequencer::Sequence(name, caller, true)
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        log.warn() << "Orientation around x is outside of the allowed range!";
        return 0;
    }

    bool checkExitCondition()
    {
        return !checkOrientation.validate();
    }

private:
    ControlSystem &cs;
    CheckOrientation checkOrientation;
};


#endif // OrientationException_HPP_
