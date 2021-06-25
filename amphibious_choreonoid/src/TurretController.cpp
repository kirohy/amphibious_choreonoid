#include <cnoid/SimpleController>

class TurretController : public cnoid::SimpleController {
  private:
    cnoid::Link *joints_[2];
    double q_ref_[2];
    double q_prev_[2];
    double dt_;

  public:
    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        joints_[0] = io->body()->link("TURRET_Y");
        joints_[1] = io->body()->link("TURRET_P");

        for (int i = 0; i < 2; i++) {
            joints_[i]->setActuationMode(cnoid::Link::JOINT_TORQUE);
            io->enableIO(joints_[i]);
            q_ref_[i] = q_prev_[i] = joints_[i]->q();
        }

        dt_ = io->timeStep();

        return true;
    }

    virtual bool control() override {
        // PD gains
        static const double P = 200.0;
        static const double D = 50.0;

        for (int i = 0; i < 2; i++) {
            double q = joints_[i]->q(); // input
            double dq = (q - q_prev_[i]) / dt_;
            double dq_ref = 0.0;
            joints_[i]->u() = P * (q_ref_[i] - q) + D * (dq_ref - dq); // output
            q_prev_[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController)
