#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MyRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include <customSteps/moveTo.hpp>

//
class CheckPath : public eeros::sequencer::Condition
{
    public:
    // Initialise any neccessary attributes
    CheckPath(ControlSystem &cs) : cs(cs) 
    {

    }
    // Implement the validate method, return true if the monitor should fire
    bool validate()
    {
        return cs.lidar.pathBlocked();
    }

    private:
    ControlSystem &cs;
};
//

class MainSequence : public eeros::sequencer::Sequence
{
public:
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 MyRobotSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

        //
          checkPath(cs),
          pathMonitor("path monitor", this, checkPath,
          eeros::sequencer::SequenceProp::resume, &pathBlockedException),
          pathBlockedException("path blocked exception", this),
        //
          sleep("Sleep", this)
    {
        //
        addMonitor(&pathMonitor);
        //
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            sleep(1.0);
            // log.info() << cs.myGain.getOut().getSignal();
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    MyRobotSafetyProperties &sp;

    //
    CheckPath checkPath;
    eeros::sequencer::Monitor pathMonitor;
    PathBlockedException pathBlockedException; // not implemented
    //

    eeros::sequencer::Wait sleep;
};

#endif // MAINSEQUENCE_HPP_