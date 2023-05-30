#ifndef PATHPLANNING_HPP_
#define PATHPLANNING_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <cmath>
#include <mutex>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class PathPlanning : public Block   // We define it as a Block, because it is a subsystem
{
public:
    PathPlanning(double k1, double k2, double k3,
                 double xT = 0.0, double yT = 0.0, double phiT = 0.0,
                 double tolPos = 5e-3, double tolRot = M_PI / 1000.0) 
    :   GrR(this),
        phi(this),
        k1(k1),
        k2(k2),
        k3(k3),
        xT(xT),
        yT(yT),
        phiT(phiT),
        tolPos(tolPos),
        tolRot(tolRot),
        targetReached(true),
        enabled(false),
        RvRx_d(this),
        omegaR_d(this)
    {
        // Connect subblocks, initialize variables, ...

        // Name all relevant Signals (Output)
        this->RvRx_d.getSignal().setName("RvRx_d [m/s]");
        this->omegaR_d.getSignal().setName("omegaR_d [rad/s]");

    }

    // Implement getter functions for the subsystem inputs

    Input<eeros::math::Vector2> &getInGrR()
    {
        return GrR;
    }

    Input<> &getInphi()
    {
        return phi;
    }

    // Implement getter functions for the subsystem outputs

    Output<> &geOutRvRx_d()
    {
        return RvRx_d;
    }

    Output<> &getOutomegaR_d()
    {
        return omegaR_d;
    }

    virtual void run()
    {
        // Check if the controller is enabled and the target position is not reached
        // If true: run the algorithm
        // Else: set the translational and angular velocities to 0

        if(enabled && !targetReached)
        {
            std::lock_guard<std::mutex> lock(mtx);

            // Calculate the distance to the target rho
            rho = sqrt(square(xT -GrR.getSignal().getValue()[0]) + square(yT -GrR.getSignal().getValue()[1]));
           
            // Check if we reached the target (rho = 0 (with tolerance))
            if(rho <= tolPos)
            {
                targetReached = true; 

                // Set Output to 0
                RvRx_d.getSignal().setValue(0.0);
                omegaR_d.getSignal().setValue(0.0);
            }
            else
            {
                // Calculate the angles
                gamma = atan2(yT - GrR.getSignal().getValue()[1], xT - GrR.getSignal().getValue()[0]) - phi.getSignal().getValue();
                gamma = constrainAngle(gamma);

                delta = gamma - phi.getSignal().getValue() - phiT;
                delta = constrainAngle(delta);

                // Check if orientation is correct
                if (fabs(gamma) <= tolRot)
                {
                    // Set Output-Angle (Formula without gamma)
                    omegaR_d.getSignal().setValue(k2 * gamma + k1 * cos(gamma) * (gamma + k3 * delta));
                }
                else
                {
                    // Set Output-Angle (Formula with gamma)
                    omegaR_d.getSignal().setValue(k2 * gamma + k1 * sin(gamma) * cos(gamma) * (gamma + k3 * delta) / gamma);
                }

                // Set the translational velocity
                RvRx_d.getSignal().setValue(k1 * rho * cos(gamma));
                
            }
        }
        else
        {
            RvRx_d.getSignal().setValue(0.0);
            omegaR_d.getSignal().setValue(0.0);
        }

        // Set timestamps of the output signals
        RvRx_d.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
        omegaR_d.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
    }

    void enable(void) 
    { 
        enabled = true; 
    }

    void disable(void) 
    { 
        enabled = false; 
    }

    // Set new Target position and orientation
     void setTarget(double xT, double yT, double phiT)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->xT = xT;
        this->yT = yT;
        this->phiT = phiT;
        this->targetReached = false;
    }

    // Get the path planner status
    bool getStatus(void) 
    { 
        return targetReached; 
    }




protected:
    // Define intermediate variables and subblocks
    std::mutex mtx;                     // provide synchronization, which means only one thread can access the object at the same time
    Input<eeros::math::Vector2> GrR;    // Input as a Vector (x and y)
    Input<> phi;                        // Orientation of the Robot
    double k1, k2, k3, xT, yT, phiT, rho, gamma, delta, tolPos, tolRot;
    bool targetReached, enabled;
    Output<> RvRx_d, omegaR_d; 


    private:
    
    // calculate the square
    double square(double x) 
    { 
        return x * x; 
    }

    // constrain an angle to +-pi
    double constrainAngle(double x)
    {
        x = fmod(x + M_PI, 2.0 * M_PI);

        if (x < 0.0)
            x += 2.0 * M_PI;
        return x - M_PI;
    }

};

#endif //PATHPLANNING_HPP_
