#ifndef Controller_HPP_
#define Controller_HPP_

// To see how to define it as a subsystem, look at the video

#include <eeros/control/Blockio.hpp>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class Controller : public Blockio<1,1,T>   // Set the number of inputs and outputs NR. 1
{
public:
    Controller() // Nr. 3
    : phi_S{0.0}, kp{2.0}, U{0.0}
    {
        // Connect subblocks, initialize variables, ...
    }

    // Implement getter functions for the subsystem inputs
    void setSetpoint(double phi)
    {
        phi_S = phi;
    }

    virtual void run() // Nr. 4
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks

        U = kp*(phi_S - this->in.getSignal().getValue());
        this->out.getSignal().setValue(U);
        this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    }

protected:
    // Define intermediate variables and subblocks Nr. 2
    double phi_S, kp, U;
};

#endif //Controller_HPP_
