#include <cnoid/SimpleController>

class TurretController : public cnoid::SimpleController
{
private:
    cnoid::Link* joints[2];
    double q_ref[2];
    double q_prev[2];
    double dt;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        joints[0] = io->body()->link("TURRET_Y");
        joints[1] = io->body()->link("TURRET_P");

        for (int i = 0; i < 2; i++) {
            joints[i]->setActuationMode(cnoid::Link::JOINT_TORQUE);
            io->enableIO(joints[i]);
            q_ref[i] = q_prev[i] = joints[i]->q();
        }

        dt = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        // PD gains
        static const double P = 200.0;
        static const double D = 50.0;

        for (int i = 0; i < 2; i++) {
            double q = joints[i]->q(); // input
            double dq = (q - q_prev[i]) / dt;
            double dq_ref = 0.0;
            joints[i]->u() = P * (q_ref[i] - q) + D * (dq_ref - dq); // output
            q_prev[i] = q;
        }
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController)
