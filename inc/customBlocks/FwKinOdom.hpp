#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Output.hpp>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class FwKinOdom : public Block   // It is set to "Block", because its a Subsystem
{
public:

    /**
     * Some Instructions 
     * @brief           Construct a new Fw Kin Odom object
     * 
     * @param B         distance between the two wheels
     * @param GrRInit   Initial global robot position
     * @param phiInit   Initial global robot orientation
     */

    FwKinOdom(double B, eeros::math::Vector2 GrRInit = 0.0, double phiInit = 0.0)
     : B(B),
        // Inputvector for speed at the wheels
        vw(this),
        // Jacobianmatrix from Wheels to Robot 
        RJW([&,B] 
                          {
            eeros::math::Matrix<3,2> M = {0.5, 0.0, -1.0/B, 0.5, 0.0, 1.0/B};
            eeros::math::Vector3 out = M*RJW.getIn().getSignal().getValue();
            for (int i = 0; i < 3; i++)
            {
                RJW.getOut(i).getSignal().setValue(out(i));
                RJW.getOut(i).getSignal().setTimestamp(RJW.getIn().getSignal().getTimestamp());
            } }),
        // Rotationsmatrix from robot to baseframe
         GRR([&]()
              {
            double phi = GRR.getIn(2).getSignal().getValue();
            double cphi = std::cos(phi);
            double sphi = std::sin(phi);

            eeros::math::Matrix<3,3> M = {cphi, sphi, 0.0, -sphi, cphi, 0.0, 0.0, 0.0, 1.0};
            eeros::math::Vector3 v = {GRR.getIn(0).getSignal().getValue(), GRR.getIn(1).getSignal().getValue(), 0.0};
            eeros::math::Vector3 out = M*v;
            
            GRR.getOut().getSignal().setValue(out.getSubMatrix<2,1>(0, 0));
            GRR.getOut().getSignal().setTimestamp(GRR.getIn(0).getSignal().getTimestamp()); })
        
    {
        // Connect subblocks, initialize variables, ...
        
        // Set initial integrator values
        GrR.setInitCondition(GrRInit);
        phi.setInitCondition(phiInit);

        // Name all blocks
        RJW.setName("FwKinOdom->RJW (Jabobianmatrix)");
        GRR.setName("FwKinOdom->GRR (Rotationsmatrix)");
        GrR.setName("FwKinOdom->GrR (Global robot position)");
        phi.setName("FwKinOdom->phi (Global robot orientation)");

        // Name all signals
        RJW.getOut(0).getSignal().setName("RvRx [m/s]");
        RJW.getOut(1).getSignal().setName("RvRy [m/s]");
        RJW.getOut(2).getSignal().setName("omegaR [rad/s]");

        GRR.getOut().getSignal().setName("GvR [m/s]");

        GrR.getOut().getSignal().setName("GrR [m]");

        phi.getOut().getSignal().setName("phi [rad]");

        // Connect all signals
        RJW.getIn().connect(vw);
        GRR.getIn(0).connect(RJW.getOut(0));
        GRR.getIn(1).connect(RJW.getOut(1));
        GRR.getIn(2).connect(phi.getOut());
        phi.getIn().connect(RJW.getOut(2));
        GrR.getIn().connect(GRR.getOut());
    }

    // Implement getter functions for the subsystem inputs
    /**
     * @return Input<eeros::math::Vector2>& wheel velocity input
     */
    Input<eeros::math::Vector2> &getIn() { return vw; }

    // Implement getter function for the subsystem outputs
    
    /**
     * @return Output<eeros::math::Vector2>& global robot velocity
     */
    Output<eeros::math::Vector2> &getOutGvR() { return GRR.getOut(); }

    /**
     * @return Output<eeros::math::Vector2>& global robot position
     */
    Output<eeros::math::Vector2> &getOutGrR() { return GrR.getOut(); }

    /**
     * @return Output<>& global robot orientation
     */
    Output<> &getOutPhi() { return phi.getOut(); }

    /**
     * @return Output<>& global robot angular velocity
     */
    Output<> &getOutOmegaR() { return RJW.getOut(2); }

    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks
        RJW.run();
        phi.run();
        GRR.run();
        GrR.run();
    }

     /**
     * @brief Enable the integrators
     * 
     */
    void enable(void)
    {
        GrR.enable();
        phi.enable();
    }

    /**
     * @brief Disable the integrators
     * 
     */
    void disable(void)
    {
        GrR.disable();
        phi.disable();
    }

    /**
     * @brief Set position and orientation
     * 
     * @param GrR global robot position
     * @param phi global robot orientation
     */
    void setPose(eeros::math::Vector2 GrR, double phi)
    {
        this->GrR.setInitCondition(GrR);
        this->phi.setInitCondition(phi);
    }


protected:
    // Define intermediate variables and subblocks
    double B;                                           // Distance between Wheels (Paramterinput)
    InputSub<eeros::math::Vector2> vw;                  // Speed of the Wheels (Input)
    Blockio<1, 3, eeros::math::Vector2, double> RJW;    // Jacobianmatrix
    Blockio<3, 1, double, eeros::math::Vector2> GRR;    // Rotationsmatrix
    I<eeros::math::Vector2> GrR;                        // Global robot position (Output) (Defined as Integrationsblock)
    I<> phi;                                            // Global robot orientation (Output) (Defined as Integrationsblock)
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
