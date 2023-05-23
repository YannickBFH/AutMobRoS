#ifndef PI_Controller_HPP_
#define PI_Controller_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
// Include header files for the subblocks

using namespace eeros::control;

/**
 * @brief PI velocity controller class
 *
 * @tparam T output type (default double)
 */

template <typename T = double>
class PI_Controller : public Block      // We define it as a Block, because it is a subsystem
{
public:
    /**
     * @brief Construct a new Controller object
     *
     * @param om0 natural frequency
     * @param D lehr's damping ratio
     * @param M mass matrix
     * @param eLimit integrator limit
     */
    PI_Controller(double omega, double D, double M, T elimit) 
    : qd(this),
      KP(2.0 * D * omega),
      KI(omega * omega),
      M(M)
    {
        // Connect subblocks, initialize variables, ...
        init(elimit);

    }

    /**
     * @brief Construct a new Controller object
     *
     * @param fTask task frequency
     * @param D lehr's damping ratio
     * @param s safety factor
     * @param M mass matrix
     * @param eLimit integrator limit
     */
    PI_Controller(double fTask, double D, double s, double M, T eLimit)
        : qd(this),
          KP(fTask / s),
          KI(fTask / 2.0 / s / D * fTask / 2.0 / s / D),
          M(M)
    {
        init(eLimit);
    }


    // Implement getter functions for the subsystem inputs
    /**
     * @brief Get the In object
     *
     * @param index index
     * @return Input<T>& index 0: qd_d, index 1: qd
     */
    virtual Input<T> &getIn(uint8_t index)
    {
        if (index == 0)
        {
            return ed.getIn(0);
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    // Implement getter functions for the subsystem output
    /**
     * @brief Get the Out object
     *
     * @param index index
     * @return Output<T>& index 0: Q, index 1: qd
     */
    virtual Output<T> &getOut(uint8_t index)
    {
        if (index == 0)
        {
            return M.getOut();
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }


    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks

        ed.run();
        KP.run();
        e.run();
        KI.run();
        qddC.run();
        M.run();

    }

     /**
     * @brief enable integrator
     * 
     */
    void enable()
    {
        e.enable();
    }

    /**
     * @brief disable integrator
     * 
     */
    void disable()
    {
        e.disable();
    }

    
    /**
     * @brief sets the position error limit
     * 
     * @param eLimit position error limit
     */
    void setELimit(T eLimit)
    {
        e.setLimit(eLimit, -eLimit);
    }

protected:
    // Define intermediate variables and subblocks
    InputSub<T> qd;         // Inputvariable
    Sum<2, T> ed, qddC;     // Sum
    Gain<T> KP, KI, M;      // Gain
    I<T> e;                 // Integral

    private:
    /**
     * @brief init method
     *
     * @param eLimit integrator limit
     */
    void init(T eLimit)
    {
        // Name all blocks
        ed.setName("controller->ed");
        KP.setName("controller->KP");
        e.setName("controller->e");
        KI.setName("controller->KI");
        qddC.setName("controller->qddC");
        M.setName("controller->M");

        // Name all signals
        ed.getOut().getSignal().setName("ed [rad/s]");
        KP.getOut().getSignal().setName("qddCP [rad/s^2]");
        e.getOut().getSignal().setName("e [rad]");
        KI.getOut().getSignal().setName("qddCI [rad/s^2]");
        qddC.getOut().getSignal().setName("qddC [rad/s^2]");
        M.getOut().getSignal().setName("Q [Nm]");

        // Connect signals
        ed.getIn(1).connect(qd);
        ed.negateInput(1);
        KP.getIn().connect(ed.getOut());
        e.getIn().connect(ed.getOut());
        KI.getIn().connect(e.getOut());
        qddC.getIn(0).connect(KP.getOut());
        qddC.getIn(1).connect(KI.getOut());
        M.getIn().connect(qddC.getOut());

        // Additional configuration
        T eInit = 0.0;
        e.setInitCondition(eInit);
        e.setLimit(eLimit, -eLimit);
    }
};

#endif //PI_Controller_HPP_
