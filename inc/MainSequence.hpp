#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MyRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>


class MainSequence : public eeros::sequencer::Sequence
{
public:
    // Initiate all Elements
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 MyRobotSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

          sleep("Sleep", this)

         
          
          // Initiate seperate Sequence (Execption-Sequence)


          // Initiate new Monitor and define what it does
          
          
    {
        // Add Monitor to Sequece

        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            // Define structure of main-Sequence
            sleep(1.0);
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutPhi().getSignal();
        }
        return 0;
    }

private:

    // Define all Variables
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    MyRobotSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
};

#endif // MAINSEQUENCE_HPP_