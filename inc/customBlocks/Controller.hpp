#ifndef Controller_HPP_
#define Controller_HPP_

#include <eeros/control/Blockio.hpp>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class Controller : public Blockio<1,1,T>   // Set the number of inputs and outputs
{
public:
    Controller() : theta{0.0}, kp{2.0}, phi{0.0}
    {
        // Connect subblocks, initialize variables, ...

    }

    // Implement getter functions for the subsystem inputs


    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks

        // Calculate output-value
        phi = kp * this->in.getSignal().getValue();

        // Set output-value
        this->out.getSignal().setValue(phi);

        // set timestamp for output-value
        this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());

    }

protected:
    // Define intermediate variables and subblocks
    double theta, kp, phi;
};

#endif //Controller_HPP_
