#ifndef TCPVECPOSCONT_HPP_
#define TCPVECPOSCONT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class TCPVecPosCont  : public Blockio<1, 1, eeros::math::Vector2>
{
public:
    TCPVecPosCont (double fClock, double D, double vmax, double tol = 1e-3) 
    : K(fClock / 3.2 / 2.0 / D),
          GvTcMax(vmax),
          tol(tol),
          enabled(false)
    {
        // Connect subblocks, initialize variables, ...

        // Name all relevant Signals (Output)
        this->out.getSignal().setName("GvTc [m/s]");

    }

    virtual void run()
    {
        if (enabled)
        {
            std::lock_guard<std::mutex> lock(mtx);

            eeros::math::Vector2 GeTd = GrTd - this->getIn().getSignal().getValue();

            double GeTdNorm = std::sqrt(GeTd(0) * GeTd(0) + GeTd(1) * GeTd(1));

            double GvTc = K * GeTdNorm;

            if (GvTc > GvTcMax)
            {
                GvTc = GvTcMax;
            }
            else if (GvTc < -GvTcMax)
            {
                GvTc = -GvTcMax;
            }
            if (GeTdNorm > 0.0)
            {
                this->getOut().getSignal().setValue(GvTc * (GeTd / GeTdNorm));
            }
            else
            {
                this->getOut().getSignal().setValue(0.0);
            }
            status = (GeTdNorm < tol);
        }
        else
        {
            this->getOut().getSignal().setValue(0.0);
        }
        this->getOut().getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
    }

    void enable(void) 
    { 
        this->enabled = true; 
    }

    void disable(void) 
    { 
        this->enabled = false; 
    }

    // Set new Target position and orientation
    void setTarget(eeros::math::Vector2 GrTd)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->GrTd = GrTd;
        status = false;
    }


    // Get the path planner status
    bool getStatus(void) 
    { 
        return status; 
    }


protected:
    // Define intermediate variables and subblocks
    std::mutex mtx;
    eeros::math::Vector2 GrTd;
    double K, GvTcMax, tol;
    bool enabled, status;
};

#endif //TCPVECPOSCONT_HPP_
